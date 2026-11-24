"""Synthetic replay of the 2026-11-23 blocked close; never opens a CAN driver."""
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from robol2l_deploy.runtime import StationRuntime
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cap.physical_task import DualJointTask
from robol2l_deploy.cap.program import ProgramExecutor
from test_arx_dual import fixture_station


class Clock:
    now = 0.

    def monotonic(self):
        return self.now

    def time(self):
        return 1800000000. + self.now

    def sleep(self, seconds):
        self.now += seconds


class GraspFeedbackTests(unittest.TestCase):
    def setUp(self):
        tmp = TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.station = fixture_station()
        self.root = tmp.name
        self.rt = StationRuntime(self.station, tmp.name, allow_motion=True)
        self.addCleanup(self.rt.close)
        self.arms = {side: unit.driver.arm for side, unit in self.station.arms.items()}
        for side, endpoints in [('left', (.02, 4.869344625009536)), ('right', (.01, 4.7793))]:
            self.station.arms[side].gripper = endpoints
            self.arms[side].gripper = endpoints[1]
        self.station.unsupported_operations.difference_update(
            {'set_gripper', 'set_grippers', 'open_gripper', 'close_gripper'})
        self.clock = Clock()
        patcher = patch('robol2l_deploy.adapters.arx_dual.time', self.clock)
        patcher.start()
        self.addCleanup(patcher.stop)

    def blocked(self, side='left', position=2.5545510032806895):
        arm = self.arms[side]
        original = arm.set_gripper_pos
        def command(target, duration):
            result = original(target, duration)
            arm.gripper = position
            return result
        arm.set_gripper_pos = command

    def test_recorded_block_width_is_feedback_not_contact_or_fault(self):
        self.blocked()
        result = self.rt.close_gripper(arm='left')
        self.assertTrue(result.ok, result.error)
        row = result.diagnostics['grippers']['left']
        self.assertEqual(row['completion'], 'sdk_reference_finished')
        self.assertEqual(row['measured_rad'], 2.5545510032806895)
        self.assertEqual(row['target_rad'], .02)
        self.assertIsNone(row['target_reached'])
        self.assertFalse(row['physical_arrival_verified'] or row['grasp_verified'])
        self.assertFalse(self.rt.faulted or self.arms['left'].protected)
        self.assertEqual(self.arms['left'].commands, [('gripper', .02, None)])

    def test_nonzero_target_deviation_and_velocity_do_not_create_fault(self):
        self.blocked(position=2.)
        self.arms['left'].get_gripper_vel = lambda: .2
        for position in (.2, 1.):
            result = self.rt.set_gripper(position, arm='left')
            self.assertTrue(result.ok, result.error)
            row = result.diagnostics['grippers']['left']
            self.assertGreater(row['error_rad'], .1)
            self.assertEqual(row['measured_velocity_rad_s'], .2)
            self.assertIsNone(row['target_reached'])
        self.assertFalse(self.rt.faulted)

    def test_native_reference_completion_not_project_timer_controls_return(self):
        self.blocked()
        original = self.arms['left'].native_query
        samples = []
        def query(name, **kw):
            if name != 'reference_status': return original(name, **kw)
            samples.append(self.clock.now)
            self.clock.now += 200.
            return dict(joints_finished=True, gripper_finished=len(samples) == 3)
        with patch.object(self.arms['left'], 'native_query', side_effect=query):
            result = self.rt.close_gripper(arm='left')
        self.assertTrue(result.ok, result.error)
        self.assertEqual(len(samples), 3)
        self.assertGreater(self.clock.now, 600.)
        self.assertFalse(result.diagnostics['grippers']['left']['physical_arrival_verified'])

    def test_nonfinite_feedback_is_an_interface_error(self):
        self.arms['left'].get_gripper_vel = lambda: float('nan')
        result = self.rt.close_gripper(arm='left')
        self.assertFalse(result.ok)
        self.assertIn('invalid_arx_gripper_velocity', result.error.message)

    def test_native_rejection_and_native_fault_still_stop(self):
        with patch.object(self.arms['left'], 'set_gripper_pos', return_value=False):
            self.assertFalse(self.station.move_hands({'left':0}, lambda:True).ok)
        self.arms['left'].fault = 'native fault'
        with self.assertRaises(Exception):
            self.station.check_hands({'left':0})
        self.assertFalse(self.arms['left'].commands)

    def test_confirmed_empty_closed_standard_and_endpoints_remain(self):
        result = self.rt.set_grippers({'left':0., 'right':0.})
        self.assertTrue(result.ok, result.error)
        for side, target in [('left', .02), ('right', .01)]:
            row = result.diagnostics['grippers'][side]
            self.assertEqual(row['target_rad'], target)
            self.assertTrue(row['empty_closed_verified'])
            self.assertFalse(row['grasp_verified'])

    def test_pause_still_prevents_dispatch(self):
        self.rt.paused = True
        self.assertFalse(self.rt.close_gripper(arm='left').ok)
        self.assertFalse(self.arms['left'].commands)

    def test_program_gets_unverified_completion_instead_of_host_fault_stop(self):
        from pathlib import Path
        self.blocked()
        report = ProgramExecutor(self.rt, DualJointTask('fixture grasp').context).execute(
            'r = set_gripper(0, arm="left")\n'
            's = get_robot_state()\n'
            'result = {"ok": r.ok, "feedback": r.diagnostics["grippers"]["left"], '
            '"opening": s.gripper_positions["left"]}', Path(self.root)/'policy')
        self.assertTrue(report['ok'], report['error'])
        self.assertTrue(report['result']['ok'])
        self.assertFalse(report['result']['feedback']['grasp_verified'])
        self.assertAlmostEqual(report['result']['opening'], .52265845, places=5)
        self.assertFalse(self.rt.faulted)

