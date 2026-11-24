"""Offline installed-SDK reference semantics and removal of station policy gates."""
from pathlib import Path
from threading import RLock
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from robol2l_deploy.adapters.arx_native import query
from robol2l_deploy.cap.experiment import ExperimentController
import test_arx_adapter
import test_arx_owner
import test_experiment_pipeline


class NativeQueryTests(unittest.TestCase):
    def setUp(self):
        from bimanual.core.otg import OnlineTrajectory
        from bimanual.hardware.controller import ArmController
        self.q = np.zeros(6)
        otg = OnlineTrajectory(6, .002, np.ones(6), np.ones(6)*3, np.ones(6)*10)
        otg.reset(self.q)
        self.ctrl = SimpleNamespace(_cmd_lock=RLock(), otg=otg, _q_target=self.q.copy(),
            _q=np.ones(6), _grip_traj=None,
            TRAJ_DECIMALS=ArmController.TRAJ_DECIMALS, TRAJ_QUANTUM=ArmController.TRAJ_QUANTUM)
        for name in ('plan_duration', 'quantize_duration'):
            setattr(self.ctrl, name, MethodType(getattr(ArmController, name), self.ctrl))
        self.arm = SimpleNamespace(_ctrl=self.ctrl)

    def test_native_duration_and_path_use_native_limits_without_commanding(self):
        target = np.ones(6)*.1
        before = self.ctrl._q_target.copy()
        duration = query(self.arm, 'plan_duration', start=self.q, target=target, minimum=0.)
        self.assertEqual(duration, self.ctrl.plan_duration(self.q, target, 0.))
        path = query(self.arm, 'joint_path', start=self.q, target=target, minimum=duration)
        np.testing.assert_allclose(path[0], self.q)
        np.testing.assert_allclose(path[-1], target, atol=1e-12)
        np.testing.assert_array_equal(self.ctrl._q_target, before)
        np.testing.assert_array_equal(self.ctrl._q, np.ones(6))
        self.assertLess(duration, 1.)  # Historical 0.1 rad/s cap required >= 1 second.

    def test_reference_completion_does_not_claim_measured_arrival(self):
        result = query(self.arm, 'reference_status')
        self.assertTrue(result['joints_finished'])
        self.assertFalse(result['physical_arrival_verified'])
        self.ctrl._q_target[:] = .1
        self.ctrl._grip_traj = object()
        result = query(self.arm, 'reference_status')
        self.assertFalse(result['joints_finished'])
        self.assertFalse(result['gripper_finished'])
        with self.assertRaisesRegex(ValueError, 'unknown_arx_native_query'):
            query(self.arm, 'set_joint_target')


class AdapterPolicyTests(unittest.TestCase):
    def setUp(self):
        self.fixture = test_arx_adapter.ARXAdapterTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def test_arx_system_prompt_has_no_residual_default_slowdown(self):
        from robol2l_deploy.cap.model import messages
        api = test_arx_adapter.PublicAPI(self.fixture.runtime, test_arx_adapter.MockReachTask.context)
        observation = test_experiment_pipeline.MockAdapter().get_observation()
        system = messages(api, observation, [], {})[0]['content']
        self.assertNotIn('Use 0.05', system)
        self.assertNotIn('0<s<=1', system)
        self.assertIn('installed SDK planner timing', system)

    def test_native_short_duration_and_long_explicit_duration(self):
        adapter = self.fixture.adapter
        trajectory, _ = adapter.plan_motion(np.ones(6)*.01)
        self.assertLess(trajectory.dt_s, .5)
        self.assertEqual(adapter.speed_factor, 1.)
        self.assertIsNone(adapter.max_speed_factor)
        adapter.prepare_segment(np.array([np.zeros(6), np.ones(6)*.1]), 121.)
        adapter.prepare_segment(np.array([np.zeros(6), np.ones(6)*.1]), .033)
        _, plan = adapter.plan_motion(np.ones(6)*.01, speed_factor=2.)
        self.assertEqual(plan['speed_factor'], 2.)


class OwnerQueryTests(unittest.TestCase):
    def test_fixed_read_queries_do_not_need_motion_authorization(self):
        fixture = test_arx_owner.OwnerTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        token = fixture.attach()
        result = fixture.request(token, 'sdk', side='left', method='native_query',
                                 kwargs=dict(name='reference_status'))
        self.assertTrue(result['joints_finished'])
        self.assertFalse(fixture.arms['left'].commands)
        with self.assertRaisesRegex(ValueError, 'unknown_arx_native_query'):
            fixture.request(token, 'sdk', side='left', method='native_query', kwargs=dict(name='close'))


class ExperimentPolicyTests(unittest.TestCase):
    def setUp(self):
        self.fixture = test_experiment_pipeline.ExperimentTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.tearDown)
        with patch.object(self.fixture.runtime.adapter, 'describe', return_value={'embodiment':'arx_x5_2023_dual'}):
            controller = ExperimentController(self.fixture.runtime, lambda: self.fixture.model)
        self.assertEqual(controller.maximum_attempts, 8)
        self.fixture.controller = controller
        self.fixture.start()
        self.controller = controller

    def test_eight_program_limit_and_operator_speed_remains_advisory(self):
        for n in range(1, 9):
            state = self.fixture.confirm()
            self.assertEqual(state['attempts'], n)
            self.assertEqual(state['status'], 'exhausted' if n == 8 else 'waiting_reset')
        with self.assertRaises(ValueError):
            self.fixture.confirm()
        self.assertEqual(len(state['versions']), 8)
        self.assertFalse(self.fixture.runtime.allow_motion)
        request, context = self.controller.build_request(self.fixture.runtime.get_observation(), 'review')
        self.assertEqual(context['experiment']['remaining'], 0)
        advice = context['experiment']['operator_speed_recommendation']
        self.assertEqual(advice['time_dilation_factor'], self.controller.state['speed_factor'])
        self.assertFalse(advice['enforced_by_host'])
        self.assertNotIn('0<s<=1', advice['native_domain'])
        self.assertNotIn('verified negligible', request[0]['content'])
        self.assertIn('ninth program is never dispatched', request[0]['content'])
        self.assertIn('at most eight executed programs v00..v07', request[0]['content'])

    def test_arx_recommendation_is_serialized_without_clamping(self):
        import json
        for speed in (.02, 1.25):
            self.controller.state['speed_factor'] = speed
            request, _ = self.controller.build_request(self.fixture.runtime.get_observation(), 'generate')
            sent = json.loads(request[1]['content'][0]['text'])
            advice = sent['experiment']['operator_speed_recommendation']
            self.assertEqual(advice['time_dilation_factor'], speed)
            self.assertFalse(advice['enforced_by_host'])
            self.assertIn('advisory, not a host-enforced cap', request[0]['content'])

    def test_previously_over_budget_record_only_reviews_without_dispatch(self):
        self.controller.state['attempts'] = 11
        self.controller.save()
        with patch.object(self.fixture.runtime.adapter, 'describe', return_value={'embodiment':'arx_x5_2023_dual'}):
            restored = ExperimentController(self.fixture.runtime, lambda: self.fixture.model)
        self.assertEqual(restored.snapshot()['maximum_attempts'], 8)
        with patch.object(restored, '_authorize') as authorize, patch.object(restored, '_restore_initial') as reset:
            restored._cycle()
        authorize.assert_not_called()
        reset.assert_not_called()
        self.assertEqual(restored.state['attempts'], 11)
        self.assertEqual(restored.state['status'], 'exhausted')
        _, context = restored.build_request(self.fixture.runtime.get_observation(), 'review')
        self.assertEqual(context['experiment']['remaining'], 0)

    def test_arx_start_still_accepts_advisory_speed_above_one(self):
        self.controller.end(self.controller.state['session_id'])
        with patch.object(self.controller, '_cycle'):
            self.controller.start('Mock task', 'Mock criteria', True, speed_factor=1.25)
            state = self.fixture.wait()
        self.assertEqual(state['speed_factor'], 1.25)
        self.assertEqual(state['maximum_attempts'], 8)

    def test_invalid_program_retry_has_no_three_response_gate_and_assist_is_optional(self):
        model = Mock()
        model.decide.side_effect = [dict(kind='program', reason='fixture', source='if')]*4 + [
            dict(kind='assist', reason='Please clarify the target')]
        value = self.controller._decide(model, self.fixture.runtime.get_observation(), 'generate')
        self.assertEqual(value['kind'], 'assist')
        self.assertEqual(model.decide.call_count, 5)
        self.assertEqual(self.controller.state['gate']['rejected'], 4)
