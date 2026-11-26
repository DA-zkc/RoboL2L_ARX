"""Real SDK math plus synthetic devices; no CAN, cameras or live services."""
import json
from pathlib import Path
from threading import Event, RLock, Thread
from types import SimpleNamespace
import unittest

import numpy as np

from robol2l_deploy.adapters.arx_native import query, ReferenceUpdateMonitor
from robol2l_deploy.errors import ErrorCode
import test_arx_dual


class NativeReferenceErrorTests(unittest.TestCase):
    def test_normal_sdk_update_still_finishes_with_monitor_installed(self):
        from bimanual.core.otg import OnlineTrajectory
        otg = OnlineTrajectory(6, .005, np.ones(6), np.ones(6)*3, np.ones(6)*10)
        otg.reset(np.zeros(6))
        target = np.ones(6)*.1
        arm = SimpleNamespace(_ctrl=SimpleNamespace(_cmd_lock=RLock(), otg=otg,
                                                    _q_target=target, _grip_traj=None))
        query(arm, 'reference_status')
        for _ in range(1000):
            otg.step(target)
            receipt = query(arm, 'reference_status')
            self.assertIsNone(receipt['reference_error'])
            if receipt['joints_finished']:
                break
        self.assertTrue(receipt['joints_finished'])
        self.assertFalse(receipt['physical_arrival_verified'])

    def test_swallowed_native_update_error_is_reported_without_changing_sdk_behavior(self):
        from bimanual.core.otg import OnlineTrajectory
        case = json.loads((Path(__file__).parent/'fixtures/arx-native-update-error.json').read_text())
        otg = OnlineTrajectory(6, case['dt'], np.array(case['max_velocity']),
                               np.array(case['max_acceleration']), np.array(case['max_jerk']))
        otg.reset(np.array(case['current_position']))
        otg._inp.current_velocity = case['current_velocity']
        otg._inp.current_acceleration = case['current_acceleration']
        target = np.array(case['target_position'])
        ctrl = SimpleNamespace(_cmd_lock=RLock(), otg=otg, _q_target=target, _grip_traj=None)
        arm = SimpleNamespace(_ctrl=ctrl)
        before = otg.position.copy()
        self.assertIsNone(query(arm, 'reference_status')['reference_error'])
        # Native SDK catches RuckigError and returns its previous reference.
        result = otg.step(target, min_duration=case['minimum_duration'])
        np.testing.assert_array_equal(result, before)
        receipt = query(arm, 'reference_status')
        self.assertFalse(receipt['joints_finished'])
        self.assertEqual(receipt['reference_error']['type'], 'RuckigError')
        self.assertIn('[ruckig]', receipt['reference_error']['message'])
        self.assertFalse(receipt['physical_arrival_verified'])
        # A replacement target cannot inherit the previous target's failure.
        ctrl._q_target = before.copy()
        self.assertIsNone(query(arm, 'reference_status')['reference_error'])

    def test_monitor_preserves_result_exception_and_arguments(self):
        class Engine:
            response = -110
            def update(self, input, output):
                self.arguments = (input, output)
                if isinstance(self.response, Exception):
                    raise self.response
                return self.response
        engine = Engine()
        monitor = ReferenceUpdateMonitor(engine)
        input, output = SimpleNamespace(target_position=[0]*6), object()
        self.assertEqual(monitor.update(input, output), -110)
        self.assertEqual(monitor.error['code'], -110)
        self.assertEqual(engine.arguments, (input, output))
        exception = RuntimeError('native fixture error')
        engine.response = exception
        with self.assertRaises(RuntimeError) as caught:
            monitor.update(input, output)
        self.assertIs(caught.exception, exception)
        engine.response = 0
        self.assertEqual(monitor.update(input, output), 0)
        self.assertIsNone(monitor.error)


class RuntimeReferenceWaitTests(unittest.TestCase):
    def setUp(self):
        self.fixture = test_arx_dual.DualARXTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def test_native_reference_error_returns_failed_receipt_and_retains_support(self):
        f = self.fixture
        original = f.right.native_query
        def native(name, **args):
            if name == 'reference_status':
                return dict(joints_finished=False, gripper_finished=True,
                            reference_error={'type':'RuckigError', 'message':'native fixture error'})
            return original(name, **args)
        f.right.native_query = native
        plan = f.runtime.plan_motion(test_arx_dual.wrist(.2), arm='right')
        result = f.runtime.execute_trajectory(plan.trajectory)
        self.assertFalse(result.ok)
        self.assertEqual(result.error.code, ErrorCode.CONTROLLER_FAILED)
        self.assertEqual(result.error.details['type'], 'RuckigError')
        self.assertEqual(result.diagnostics['per_arm']['right']['native_reference']['reference_error']['message'],
                         'native fixture error')
        self.assertTrue(result.diagnostics['per_arm']['right']['support']['request_accepted'])
        self.assertTrue(f.runtime.faulted)
        self.assertFalse(f.left.commands)

    def test_explicit_abort_exits_wait_and_does_not_claim_completion(self):
        f = self.fixture
        entered = Event()
        original = f.right.native_query
        def native(name, **args):
            if name == 'reference_status':
                entered.set()
                return dict(joints_finished=False, gripper_finished=True)
            return original(name, **args)
        f.right.native_query = native
        results = []
        def work():
            with f.runtime.program_lease():
                plan = f.runtime.plan_motion(test_arx_dual.wrist(.2), arm='right')
                results.append(f.runtime.execute_trajectory(plan.trajectory))
        worker = Thread(target=work, daemon=True)
        worker.start()
        self.assertTrue(entered.wait(2))
        f.runtime.request_execution_abort()
        worker.join(2)
        self.assertFalse(worker.is_alive())
        result, = results
        self.assertFalse(result.ok)
        self.assertEqual(result.error.code, ErrorCode.CANCELLED)
        self.assertFalse(result.error.details['completion_verified'])
        self.assertEqual(result.diagnostics['outcome'], 'operator_cancelled_unverified')
        self.assertTrue(result.diagnostics['per_arm']['right']['support']['request_accepted'])
        self.assertFalse(f.left.commands)
