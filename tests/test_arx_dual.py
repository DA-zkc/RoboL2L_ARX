import json
from contextlib import contextmanager
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
import sys
from unittest.mock import patch

import numpy as np

from robol2l_deploy.adapters import make_station_adapter
from robol2l_deploy.adapters.arx_dual import EMBODIMENT
from robol2l_deploy.cap.experiment import ExperimentController
from robol2l_deploy.runtime import StationRuntime
from test_arx_adapter import FakeArm, FakeKinematics


CONFIG = Path(__file__).resolve().parents[1] / 'config/arx-x5-2023-dual.json'


def fixture_station():
    from robol2l_deploy.adapters.arx_dual import DualARXStation
    import json
    station = DualARXStation(dict(json.loads(CONFIG.read_text()), cameras={}, extrinsics=None, payload=None, commissioning=None, grasp_geometry=None), directory=CONFIG.parent)
    station.synthetic = True
    for unit in station.arms.values():
        unit.kin = FakeKinematics()
        unit.driver = SimpleNamespace(arm=FakeArm(), allow_motion=True, close=lambda: None)
    return station


def wrist(angle):
    return [0., 0., 0., 0., 0., angle]


class DualARXTests(unittest.TestCase):
    def setUp(self):
        tmp = TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.station = fixture_station()
        self.runtime = StationRuntime(self.station, tmp.name, allow_motion=True)
        self.addCleanup(self.runtime.close)
        self.left = self.station.arms['left'].driver.arm
        self.right = self.station.arms['right'].driver.arm
        self.targets = {'left': wrist(.02), 'right': wrist(-.02)}

    def test_public_speed_factor_point_one_slows_both_arms_without_changing_default(self):
        from robol2l_deploy.cap.api import PublicAPI
        from robol2l_deploy.cap.tasks import MockReachTask
        from dataclasses import replace
        from robol2l_deploy.contracts import MotionStrategy
        from robol2l_deploy.adapters.arx import TIME_QUANTUM_S
        api = PublicAPI(self.runtime, MockReachTask.context)
        tools = api.tools()
        strategy = MotionStrategy(ik_solver='station', pose_planner='station',
                                  time_dilation_factor=.1)
        # Use actual public planning paths with synthetic devices, never execute.
        # This covers the factor's direction and propagation, not just prompt text.
        for arm in ('left', 'right'):
            pose = self.station.get_robot_state().end_effector_poses[arm]
            pose = replace(pose, position=np.asarray(pose.position) + [.2, 0, 0])
            for goal in (wrist(.2), pose):
                default = tools['plan_motion'](goal, arm=arm, strategy=None)
                slow = tools['plan_motion'](goal, arm=arm, strategy=strategy)
                again = tools['plan_motion'](goal, arm=arm, strategy=None)
                for plan in (default, slow, again):
                    self.assertTrue(plan.ok, plan.error)
                self.assertEqual(slow.diagnostics['speed_factor'], .1)
                # Each plan rounds separately; multiplying a rounded default
                # can differ by up to ten native time quanta.
                self.assertAlmostEqual(slow.trajectory.dt_s, 10 * default.trajectory.dt_s,
                                       delta=10 * TIME_QUANTUM_S)
                self.assertEqual(again.diagnostics['speed_factor'], 1.)
                self.assertEqual(again.trajectory.dt_s, default.trajectory.dt_s)
        targets = {'left': wrist(.2), 'right': wrist(-.2)}
        default = tools['plan_synchronized_motion'](targets)
        slow = tools['plan_synchronized_motion'](targets, strategy=strategy)
        self.assertTrue(default.ok, default.error)
        self.assertTrue(slow.ok, slow.error)
        self.assertAlmostEqual(slow.trajectory.dt_s, 10 * default.trajectory.dt_s)
        self.assertFalse(self.left.commands or self.right.commands)

    def test_recorded_return_speed_reaches_planner_without_changing_home_default(self):
        from robol2l_deploy.contracts import ExecutionResult
        from robol2l_deploy.adapters.arx import TIME_QUANTUM_S
        captured = []
        def capture(trajectory):
            captured.append(trajectory)
            return ExecutionResult(True, 0)
        with patch.object(self.runtime, 'execute_trajectory', side_effect=capture):
            self.runtime.return_recorded_joints(self.targets)
            self.runtime.return_recorded_joints(self.targets, speed_factor=.1)
            self.runtime.return_recorded_joints(self.targets)
        self.assertAlmostEqual(captured[1].dt_s, 10 * captured[0].dt_s,
                               delta=10 * TIME_QUANTUM_S)
        self.assertEqual(captured[0].dt_s, captured[2].dt_s)
        self.assertIsNone(captured[1].gripper_positions)
        for side in ('left', 'right'):
            np.testing.assert_allclose(captured[1].joint_positions[side][-1], self.targets[side])
        self.assertFalse(self.left.commands or self.right.commands)

    def test_ik_residual_survives_both_arm_wrappers(self):
        state = self.station.get_robot_state()
        for side in ('left', 'right'):
            for converged in (False, True):
                with self.subTest(side=side, converged=converged), patch.object(
                        self.station.arms[side].kin, 'ik', return_value=(np.zeros(6), converged, .123)):
                    result = self.runtime.solve_ik(state.end_effector_poses[side], arm=side)
                self.assertEqual(result.ok, converged)
                self.assertEqual(result.arm, side)
                self.assertEqual(result.diagnostics['sdk_residual'], .123)
                self.assertEqual(result.embodiment, EMBODIMENT)
        self.assertFalse(self.left.commands or self.right.commands)

    def test_pair_keeps_healthy_controller_on_peer_connection_failure(self):
        lock_events, closed = [], []
        @contextmanager
        def lock():
            lock_events.append('acquired')
            try:
                yield
            finally:
                lock_events.append('released')

        def sdk(config):
            if config['can_port'] == 'can3':
                raise RuntimeError('right SDK init failed')
            arm = FakeArm()
            arm.damp = lambda *args, **kwargs: True
            arm.close = lambda: closed.append(config['can_port'])
            return arm

        pair = make_station_adapter(CONFIG, EMBODIMENT, allow_connection=True)
        self.addCleanup(lambda: pair.close(hardware_supported=True))
        for unit in pair.arms.values():
            unit.kin = FakeKinematics()
        with (patch('robol2l_deploy.adapters.arx_dual.hardware_lock', lock),
              patch('robol2l_deploy.adapters.arx.ARXDriver.realtime_ready', return_value=True),
              patch.dict(sys.modules, bimanual=SimpleNamespace(SingleArm=sdk))):
            with self.assertRaisesRegex(Exception, 'right SDK init failed'):
                pair.get_robot_state()
        self.assertEqual(lock_events, ['acquired'])
        self.assertEqual(closed, [])
        with self.assertRaisesRegex(RuntimeError, 'support_confirmation_required'):
            pair.close()
        self.assertEqual(closed, [])
        self.assertEqual(len(pair.cameras), 3)
        self.assertTrue(all(not owner.stop.is_set() for owner in pair.cameras.values()))
        pair.close(hardware_supported=True)
        self.assertEqual(lock_events, ['acquired', 'released'])
        self.assertEqual(closed, ['can1'])
        pair.close()
        self.assertTrue(all(owner.stop.is_set() for owner in pair.cameras.values()))

    def test_reset_quantizes_only_a_known_measured_boundary(self):
        self.station.arms['left'].kin.q_min = np.zeros(6)
        recorded = {'left': wrist(-.001), 'right': wrist(.01)}
        targets = self.station.reset_targets(recorded)
        self.assertEqual(targets['left'][-1], 0.)
        self.assertEqual(recorded['left'][-1], -.001)
        self.assertFalse(self.runtime.plan_synchronized_motion(recorded).ok)
        self.assertEqual(self.station.reset_targets(dict(recorded, left=wrist(-.01)))['left'][-1], 0.)

    def test_state_preserves_sdk_fault_and_offline_joint_details(self):
        from robol2l_deploy.errors import ApiError
        self.left.fault = 'fixture fault'
        self.left.offline_joints = ['joint4']
        with self.assertRaises(ApiError) as caught:
            self.station.get_robot_state()
        self.assertEqual(dict(caught.exception.details), dict(
            arm='left', port='can1', fault='fixture fault', offline_joints=['joint4']))
        self.assertFalse(self.left.commands or self.right.commands)

    def test_encoder_only_single_and_pair(self):
        obs = self.runtime.get_observation()
        self.assertFalse(obs.cameras)
        self.assertEqual(obs.robot_state.end_effector_poses['right'].frame, 'right_base')
        single = self.runtime.move_to_joints(wrist(.01), arm='right')
        self.assertTrue(single.ok, single.error)
        self.assertEqual(len(self.left.commands), 0)
        result = self.runtime.move_synchronized(self.targets)
        self.assertTrue(result.ok, result.error)
        self.assertEqual(result.final_errors, {'left': 0., 'right': 0.})
        self.assertEqual(self.left.commands[0][-1], self.right.commands[-1][-1])
        self.assertEqual(set(result.diagnostics['per_arm']), set(self.targets))

    def test_bad_peer_and_stale_idle_arm_block_before_dispatch(self):
        plan = self.runtime.plan_synchronized_motion(dict(self.targets, right=wrist(3.)))
        self.assertFalse(plan.ok)
        self.assertIn('right', plan.error.message)
        self.assertIn('joint6=', plan.error.message)
        plan = self.runtime.plan_motion(wrist(.01), arm='left')
        self.right.q[-1] += .1
        self.assertTrue(self.runtime.execute_trajectory(plan.trajectory).ok)
        self.assertTrue(self.left.commands)
        self.assertFalse(self.right.commands)

    def test_partial_dispatch_latches_and_survives_restart(self):
        with patch.object(self.right, 'set_joint_positions', return_value=False):
            result = self.runtime.move_synchronized(self.targets)
        self.assertFalse(result.ok)
        self.assertEqual(result.diagnostics['outcome'], 'partial_dispatch')
        rows = result.diagnostics['per_arm']
        self.assertEqual([rows[s]['dispatch'] for s in ('left','right')], ['accepted','rejected'])
        self.assertFalse(self.left.protected or self.right.protected)
        self.assertTrue(rows['left']['support']['request_accepted'])
        self.assertFalse(rows['left']['support']['hold_verified'])
        self.assertEqual(rows['right']['support']['strategy'], 'retain_position_control')
        self.assertTrue(self.runtime.faulted)
        self.assertFalse(self.runtime.move_synchronized(self.targets).ok)
        reopened = StationRuntime(fixture_station(), self.runtime.directory)
        self.addCleanup(reopened.close)
        self.assertTrue(reopened.faulted)
        self.assertEqual(len(self.left.commands), 2)  # Original motion plus hold request.

    def test_experiment_repairs_after_reset_and_returns_both(self):
        def decide(messages):
            ctx = json.loads(messages[1]['content'][0]['text'])
            attempt = ctx['experiment']['attempts']
            if ctx['experiment']['phase'] == 'review':
                report = ctx['history'][-1]['feedback']['result']
                if not report['ok']:
                    self.assertIn('right', report['error']['message'])
                    return dict(kind='program', reason='Correct the right joint target', source='result=0')
                return dict(kind='finish', reason='Both encoder deltas verified')
            targets = self.targets if attempt else dict(self.targets, right=wrist(3.))
            return dict(kind='program', reason='Plan both wrists', source=f'result=move_synchronized({targets!r})')

        controller = ExperimentController(self.runtime, lambda: SimpleNamespace(label='software_fixture', decide=decide))
        session = controller.start('Move wrists in opposite directions', 'Verify encoder deltas', True)['session_id']
        controller.thread.join(10)
        self.assertEqual(controller.state['status'], 'waiting_reset')
        for attempt, expected in enumerate(('waiting_reset', 'completed')):
            if attempt:
                controller.confirm_reset(session, True, expected_attempt=attempt)
                controller.thread.join(15)
            self.assertEqual(controller.state['status'], expected, controller.state.get('reason'))
            if attempt == 0:
                self.assertFalse(self.left.commands or self.right.commands)
                self.left.q[1], self.right.q[1] = .01, -.01
                controller.return_to_initial(session, True)
                controller.thread.join(10)
                self.assertEqual(controller.state['status'], 'waiting_reset')
                self.assertEqual(controller.state['attempts'], 1)
                self.assertEqual(self.left.q[1], 0.)
                self.assertEqual(self.right.q[1], 0.)
        np.testing.assert_allclose(self.left.q, self.targets['left'])
        np.testing.assert_allclose(self.right.q, self.targets['right'])
