"""Software-only reset authorization and whole-attempt motion evidence."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.runtime import StationRuntime, public
from robol2l_deploy.cap.experiment import ExperimentController
from robol2l_deploy.cap.reset import ResetMotionAudit
from robol2l_deploy.contracts import Trajectory, ExecutionResult
from robol2l_deploy.errors import ErrorCode, ApiError


class FirstAttemptTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.runtime = StationRuntime(MockAdapter(), Path(self.tmp.name))
        self.controller = ExperimentController(self.runtime, lambda: self)
        self.label = 'first_attempt_fixture'
        self.requests = []
        self.source = 'result = get_robot_state()'
        self.finish = False

    def decide(self, messages):
        context = json.loads(messages[-1]['content'][0]['text'])['experiment']
        self.requests.append(context)
        self.assertFalse(self.runtime.allow_motion)  # Model calls never enable motion.
        if context['attempts'] == 0:
            self.assertEqual(self.runtime.adapter.executions, 0)
        if context['phase'] == 'review' and self.finish:
            return dict(kind='finish', reason='fixture finished')
        return dict(kind='program', reason='fixture program', source=self.source)

    def tearDown(self):
        self.controller.stop.set()
        if self.controller.thread: self.controller.thread.join(15)
        self.runtime.close()
        self.tmp.cleanup()

    def wait(self):
        self.controller.thread.join(15)
        self.assertFalse(self.controller.busy())
        return self.controller.snapshot()

    def start(self):
        self.controller.start('Prepared scene', 'Fixture criterion', True)
        return self.wait()

    def test_start_freezes_operator_pose_and_runs_first_program_without_return(self):
        self.runtime.adapter.q[:] = .123
        with patch.object(self.controller, '_restore_initial', side_effect=AssertionError('initial return')):
            state = self.start()
        self.assertEqual(state['attempts'], 1)
        self.assertEqual(state['status'], 'waiting_reset')  # Only after v00 review.
        self.assertEqual(self.runtime.adapter.executions, 0)
        np.testing.assert_allclose(state['initial']['observation']['robot_state']['joint_positions']['primary'], .123)
        nodes = [s['node'] for s in state['steps']]
        self.assertEqual(nodes[:6], ['START', 'SETUP', 'CLICK', 'SNAP', 'OBSERVE', 'PLAN'])
        self.assertNotIn('RESET', nodes)
        self.assertNotIn('CONFIRM', nodes)
        self.assertTrue(self.requests[0]['initial_scene_confirmed'])
        self.assertTrue(self.requests[0]['first_attempt_without_return'])
        self.assertFalse(self.requests[0]['reset_confirmed'])
        self.assertFalse(state.get('reset_policy', {}).get('automatic_small_motion'))

    def test_first_valid_program_can_move_without_an_extra_confirmation(self):
        self.source = 'result = move_to_joints([0.1, 0, 0, 0, 0, 0, 0])'
        self.finish = True
        state = self.start()
        self.assertEqual(state['status'], 'completed')
        self.assertEqual(state['attempts'], 1)
        self.assertEqual(self.runtime.adapter.executions, 1)
        self.assertFalse(self.runtime.allow_motion)
        self.assertFalse(any(i['kind'] == 'arm_return_result' for i in state['interventions']))

    def test_retry_returns_to_same_first_snapshot_after_explicit_authorization(self):
        self.runtime.adapter.q[:] = .12
        initial = self.start()['initial']
        self.runtime.adapter.q[:] = .22
        self.finish = True
        self.controller.authorize_reset(self.controller.state['session_id'], True, expected_attempt=1)
        state = self.wait()
        self.assertEqual(state['status'], 'completed')
        self.assertEqual(state['initial'], initial)
        self.assertEqual(state['attempts'], 2)
        np.testing.assert_allclose(self.runtime.adapter.q, .12)
        self.assertEqual(sum(i['kind'] == 'arm_return_result' for i in state['interventions']), 1)

    def test_pause_or_fault_during_initial_capture_never_generates_or_moves(self):
        observe = self.controller.observe
        def capture(name):
            result = observe(name)
            self.controller.stop.set()
            return result
        with patch.object(self.controller, 'observe', side_effect=capture):
            state = self.start()
        self.assertEqual(state['status'], 'paused')
        self.assertEqual(state['attempts'], 0)
        self.assertIsNotNone(state['initial'])
        self.assertFalse(self.requests)
        self.assertEqual(self.runtime.adapter.executions, 0)

    def test_fault_after_initial_read_never_generates_or_clears_fault(self):
        observe = self.controller.observe
        def capture(name):
            result = observe(name)
            self.runtime.faulted = True
            return result
        with patch.object(self.controller, 'observe', side_effect=capture):
            state = self.start()
        self.assertEqual(state['status'], 'waiting_operator')
        self.assertTrue(self.runtime.faulted)
        self.assertFalse(self.requests)
        self.assertEqual(self.runtime.adapter.executions, 0)

    def test_capture_failure_never_generates_or_moves(self):
        with patch.object(self.controller, 'observe', side_effect=RuntimeError('camera fixture')):
            state = self.start()
        self.assertEqual(state['status'], 'error')
        self.assertFalse(self.requests)
        self.assertEqual(state['attempts'], 0)
        self.assertEqual(self.runtime.adapter.executions, 0)

    def test_restart_preserves_first_snapshot_without_auto_continuation(self):
        initial = self.start()['initial']
        calls = len(self.requests)
        restored = ExperimentController(self.runtime, lambda: self)
        self.assertEqual(restored.state['status'], 'interrupted')
        self.assertEqual(restored.state['initial'], initial)
        self.assertIsNone(restored.thread)
        self.assertEqual(len(self.requests), calls)
        self.assertEqual(self.runtime.adapter.executions, 0)


class ResetTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.runtime = StationRuntime(MockAdapter(), Path(self.tmp.name))
        self.source = 'result = get_robot_state()'
        self.kind = 'program'
        self.label = 'mock_reset_model'
        self.controller = ExperimentController(self.runtime, lambda: self)

    def decide(self, messages):
        phase = json.loads(messages[-1]['content'][0]['text'])['experiment']['phase']
        kind = self.kind if phase == 'review' else 'program'
        return dict(kind=kind, reason='software test', **({'source': self.source} if kind == 'program' else {}))

    def tearDown(self):
        self.controller.stop.set()
        if self.controller.thread: self.controller.thread.join(20)
        self.runtime.close()
        self.tmp.cleanup()

    def wait(self):
        self.controller.thread.join(20)
        self.assertFalse(self.controller.busy())
        return self.controller.snapshot()

    def start(self):
        # Existing reset-gate records remain supported after the startup change.
        # These tests isolate retry/return behavior; FirstAttemptTests exercises
        # the real, unpatched Start -> first execution path below.
        with patch.object(self.controller, '_cycle',
                          side_effect=lambda: self.controller.save('waiting_reset')):
            self.controller.start('Test reset', 'Mock completion', True)
            return self.wait()

    def authorize(self, attempt=None):
        s = self.controller.state
        self.controller.authorize_reset(s['session_id'], True, expected_attempt=s['attempts'] if attempt is None else attempt)
        return self.wait()

    def test_no_motion_auto_retries_to_budget_and_never_replaces_initial(self):
        original = self.start()['initial']
        self.runtime.adapter.q[:] = .1
        state = self.authorize()
        self.assertEqual(state['status'], 'exhausted')
        self.assertEqual(state['attempts'], 8)
        self.assertEqual(state['initial'], original)
        kinds = [i['kind'] for i in state['interventions']]
        self.assertEqual(kinds.count('human_authorized_reset'), 1)
        self.assertEqual(kinds.count('automatic_small_motion_reset'), 7)
        self.assertEqual(kinds.count('arm_return_result'), 8)
        self.assertTrue(all(v['reset_motion']['eligible'] for v in state['versions']))
        self.assertFalse(self.runtime.allow_motion)
        self.assertTrue(np.allclose(self.runtime.adapter.q, 0))

    def test_significant_motion_waits_and_next_authorization_returns_first_pose(self):
        self.start()
        self.source = 'result = move_to_joints([0.1, 0, 0, 0, 0, 0, 0])'
        state = self.authorize()
        self.assertEqual(state['status'], 'waiting_reset')
        self.assertEqual(state['attempts'], 1)
        self.assertFalse(state['versions'][0]['reset_motion']['eligible'])
        self.kind = 'finish'
        self.source = 'result = get_robot_state()'
        state = self.authorize()
        self.assertEqual(state['status'], 'completed')
        self.assertTrue(np.allclose(state['versions'][1]['before']['observation']['robot_state']['joint_positions']['primary'], 0))

    def test_return_failure_does_not_generate_or_consume_attempt(self):
        self.start()
        with patch.object(self.runtime.adapter, 'execute', return_value=ExecutionResult(False, 0,
                error=ApiError(ErrorCode.PLANNING_FAILED, 'test_failure'))):
            state = self.authorize()
        self.assertEqual(state['status'], 'waiting_operator')
        self.assertEqual(state['attempts'], 0)
        self.assertFalse(self.runtime.allow_motion)

    def test_success_receipt_without_actual_return_does_not_continue(self):
        self.start()
        self.runtime.adapter.q[:] = .2
        with patch.object(self.runtime.adapter, 'execute', return_value=ExecutionResult(True, 1)):
            state = self.authorize()
        self.assertEqual(state['status'], 'waiting_operator')
        self.assertEqual(state['attempts'], 0)

    def test_pause_during_return_does_not_continue(self):
        self.start()
        original = self.runtime.adapter.execute
        def execute(*args):
            result = original(*args)
            self.controller.stop.set()
            return result
        with patch.object(self.runtime.adapter, 'execute', side_effect=execute):
            state = self.authorize()
        self.assertEqual(state['status'], 'paused')
        self.assertEqual(state['attempts'], 0)

    def test_stale_click_rejected_without_motion(self):
        self.start()
        with self.assertRaisesRegex(ValueError, 'stale_reset'):
            self.authorize(attempt=1)
        self.assertEqual(self.runtime.adapter.executions, 0)

    def audit(self):
        before = self.runtime.get_robot_state()
        return ResetMotionAudit(public(before), before), before

    def test_out_and_back_path_is_not_zero_motion(self):
        audit, before = self.audit()
        path = np.zeros((3, 7)); path[1, 0] = .015
        t = Trajectory(path, .1, before.joint_names['primary'], 'mock', False, before, embodiment=before.embodiment)
        audit.event('trajectory_reference', {'trajectory': t})
        self.assertFalse(audit.result()['eligible'])
        self.assertAlmostEqual(audit.result()['max_joint_travel_rad'], .03)

    def test_small_verified_move_is_eligible(self):
        audit, before = self.audit()
        path = np.zeros((2, 7)); path[1, 0] = .001
        t = Trajectory(path, .1, before.joint_names['primary'], 'mock', False, before, embodiment=before.embodiment)
        audit.event('trajectory_reference', {'trajectory': t})
        audit.event('dispatch', {'operation': 'execute_trajectory'})
        audit.event('execution', {'result': ExecutionResult(True, 2)})
        self.assertTrue(audit.result()['eligible'])

    def test_gripper_and_unknown_outcome_never_auto_continue(self):
        for kind, data in [('dispatch', {'operation': 'set_grippers'}), ('outcome_unknown', {})]:
            audit, _ = self.audit()
            audit.event(kind, data)
            self.assertFalse(audit.result()['eligible'])

    def test_already_far_from_initial_is_not_eligible_even_if_stationary(self):
        audit, _ = self.audit()
        self.runtime.adapter.q[:] = .1
        audit.sample(self.runtime.get_robot_state())
        self.assertFalse(audit.result()['eligible'])

    def test_arx_right_arm_motion_is_reported_without_project_threshold(self):
        from robol2l_deploy.contracts import RobotState, SynchronizedTrajectory
        pose = self.runtime.adapter.pose
        state = RobotState({'left': np.zeros(6), 'right': np.zeros(6)},
                           {'left': np.zeros(6), 'right': np.zeros(6)},
                           {'left': pose, 'right': pose}, {'left': 1., 'right': 1.}, 'base',
                           embodiment='arx_x5_2023_dual')
        audit = ResetMotionAudit(public(state), state)
        path = np.zeros((3, 6)); path[1, 1] = .1
        trajectory = SynchronizedTrajectory({'left': np.zeros((3, 6)), 'right': path}, .1,
            state.joint_names, 'mock', False, state, embodiment=state.embodiment)
        audit.event('trajectory_reference', {'trajectory': trajectory})
        self.assertTrue(audit.result()['eligible'])
        self.assertAlmostEqual(audit.result()['max_joint_travel_rad'], .2)

    def test_scene_changed_during_review_requires_human(self):
        self.start()
        self.controller.state['reset_policy'] = {'automatic_small_motion': True}
        self.runtime.adapter.q[:] = .1
        self.assertFalse(self.controller._restore_initial(automatic=True))
        self.assertEqual(self.controller.state['reason'], 'scene_changed_before_automatic_reset')
        self.assertEqual(self.runtime.adapter.executions, 0)
        self.assertFalse(self.runtime.allow_motion)

    def test_unknown_return_stops_without_next_program(self):
        self.start()
        from robol2l_deploy.runtime import OutcomeUnknown
        with patch.object(self.runtime.adapter, 'execute', side_effect=OutcomeUnknown('test_unknown')):
            state = self.authorize()
        self.assertEqual(state['status'], 'waiting_operator')
        self.assertEqual(state['attempts'], 0)
        self.assertTrue(self.runtime.faulted)

    def test_restart_never_auto_resumes_armed_reset_policy(self):
        self.start()
        self.controller.state['reset_policy'] = {'automatic_small_motion': True}
        self.controller.save('waiting_reset')
        restored = ExperimentController(self.runtime, lambda: self)
        self.assertEqual(restored.snapshot()['status'], 'interrupted')
        self.assertIsNone(restored.thread)
        self.assertEqual(self.runtime.adapter.executions, 0)
