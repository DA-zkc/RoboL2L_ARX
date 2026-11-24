"""Regression for the recorded held-cube collapse; all devices are synthetic."""
from dataclasses import replace
from tempfile import TemporaryDirectory
from unittest import TestCase
from unittest.mock import patch

import numpy as np

from robol2l_deploy.adapters.arx_support import preserve_support
from robol2l_deploy.runtime import StationRuntime
from test_arx_dual import fixture_station, wrist
from test_arx_gripper_grasp import Clock


class SupportTests(TestCase):
    def setUp(self):
        tmp = TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.station = fixture_station()
        self.rt = StationRuntime(self.station, tmp.name, allow_motion=True)
        self.addCleanup(self.rt.close)
        self.left = self.station.arms['left'].driver.arm
        self.right = self.station.arms['right'].driver.arm
        self.left.gripper = 2.513634685
        self.clock = Clock()
        p = patch('robol2l_deploy.adapters.arx_dual.time', self.clock)
        p.start()
        self.addCleanup(p.stop)

    def bias(self, xyz=0.):
        command = self.left.set_joint_positions
        def submit(**kwargs):
            result = command(**kwargs)
            self.left.q[4] += .02485064886  # Recorded J5 error, 1.424 degrees.
            self.left.q[0] += xyz
            return result
        self.left.set_joint_positions = submit

    def assert_support_retained(self):
        self.assertFalse(self.left.protected or self.right.protected)
        self.assertFalse(self.right.commands)
        self.assertEqual(self.left.gripper, 2.513634685)

    def test_raw_waypoint_reports_deviation_without_inventing_arrival_tolerance(self):
        plan = self.rt.plan_motion(wrist(.1), arm='left')
        self.bias()
        result = self.rt.execute_trajectory(plan.trajectory)
        self.assertTrue(result.ok, result.error)
        self.assertAlmostEqual(result.final_errors['left'], .02485064886)
        self.assertFalse(result.diagnostics['physical_arrival_verified'])
        self.assertFalse(self.rt.faulted)
        self.assert_support_retained()

    def test_pose_uses_caller_accuracy_without_hidden_joint_gate(self):
        pose = self.rt.get_robot_state().end_effector_poses['left']
        self.bias(xyz=.0059)
        result = self.rt.move_to_pose(replace(pose, position=pose.position + [.02, 0, 0]),
                                      arm='left', tolerance=.01)
        self.assertTrue(result.ok, result.error)
        self.assertAlmostEqual(result.final_errors['left'], .0059)
        self.assertFalse(self.rt.faulted)
        self.assert_support_retained()

    def test_strict_pose_tolerance_still_reports_miss_without_disabling(self):
        pose = self.rt.get_robot_state().end_effector_poses['left']
        self.bias(xyz=.0059)
        result = self.rt.move_to_pose(replace(pose, position=pose.position + [.02, 0, 0]),
                                      arm='left', tolerance=.001)
        self.assertFalse(result.ok)
        self.assertEqual(result.error.code.value, 'goal_tolerance_not_met')
        self.assertFalse(self.rt.faulted)
        self.assert_support_retained()

    def test_native_reference_pending_does_not_advance_waypoints(self):
        plan = self.rt.plan_motion(wrist(.1), arm='left')
        trajectory = replace(plan.trajectory, joint_positions=np.array([wrist(0), wrist(.05), wrist(.1)]))
        native = self.left.native_query
        checks = []
        def query(name, **kw):
            if name == 'reference_status':
                checks.append(len(self.left.commands))
                # The first reference remains active for two reads.
                return dict(joints_finished=len(checks)>2, gripper_finished=True)
            return native(name, **kw)
        with patch.object(self.left, 'native_query', side_effect=query):
            result = self.rt.execute_trajectory(trajectory)
        self.assertTrue(result.ok, result.error)
        self.assertEqual(checks[:3], [1, 1, 1])
        self.assertEqual(len(self.left.commands), 2)
        self.assert_support_retained()

    def test_velocity_is_reported_without_fault_or_replacement_command(self):
        command = self.left.set_joint_positions
        def moving(**kwargs):
            accepted = command(**kwargs)
            self.left.get_joint_velocities = lambda *a, **kw: np.full(6, .2)
            return accepted
        self.left.set_joint_positions = moving
        result = self.rt.move_to_joints(wrist(.1), arm='left')
        self.assertTrue(result.ok, result.error)
        self.assertFalse(self.rt.faulted)
        self.assertEqual(len(self.left.commands), 1)
        self.assert_support_retained()

    def test_idle_arm_velocity_does_not_block_dispatch_or_completion(self):
        # Recorded v04/v05 left J1 values; repeat symmetrically for either idle arm.
        for idle, active in (('left', 'right'), ('right', 'left')):
            for speed in (.08351648351648322, .1098901098901095):
                with self.subTest(idle=idle, speed=speed):
                    idle_arm = self.station.arms[idle].driver.arm
                    active_arm = self.station.arms[active].driver.arm
                    before = len(idle_arm.commands)
                    target = active_arm.q.copy()
                    target[-1] += .01
                    with patch.object(idle_arm, 'get_joint_velocities', return_value=np.array([speed,0,0,0,0,0])):
                        plan = self.rt.plan_motion(target, arm=active)
                        self.assertTrue(plan.ok, plan.error)
                        result = self.rt.execute_trajectory(plan.trajectory)
                    self.assertTrue(result.ok, result.error)
                    self.assertFalse(self.rt.faulted)
                    self.assertEqual(len(idle_arm.commands), before)
                    self.assertEqual(result.diagnostics['per_arm'][idle]['measured_velocities'][0], speed)
                    self.assertEqual(result.diagnostics['per_arm'][idle]['dispatch'], 'not_sent')

    def test_idle_position_change_still_reported_separately_from_velocity(self):
        submit = self.right.set_joint_positions
        def drift(**kwargs):
            result = submit(**kwargs)
            self.left.q[0] += .02
            return result
        with patch.object(self.right, 'set_joint_positions', side_effect=drift):
            result = self.rt.move_to_joints(wrist(.01), arm='right')
        self.assertTrue(result.ok, result.error)
        self.assertFalse(self.rt.faulted)
        self.assertAlmostEqual(result.diagnostics['per_arm']['left']['max_error_rad'], .02)

    def test_noop_side_in_synchronized_plan_has_no_idle_velocity_gate(self):
        for idle, active in (('left', 'right'), ('right', 'left')):
            with self.subTest(idle=idle):
                idle_arm = self.station.arms[idle].driver.arm
                goals = {side: unit.driver.arm.q.copy() for side, unit in self.station.arms.items()}
                goals[active][-1] += .01
                before = len(idle_arm.commands)
                with patch.object(idle_arm, 'get_joint_velocities', return_value=np.full(6, .11)):
                    state = self.rt.get_robot_state()
                    trajectory, payload = self.station._plan(state, goals, synchronized=True)
                    self.assertIsNone(self.station.check_execution(payload))
                    result = self.rt.execute_trajectory(trajectory)
                self.assertTrue(result.ok, result.error)
                self.assertFalse(self.rt.faulted)
                self.assertEqual(len(idle_arm.commands), before)
                self.assertEqual(result.diagnostics['per_arm'][idle]['dispatch'], 'not_sent')

    def test_native_fault_never_clears_or_reenables_controller(self):
        command = self.left.set_joint_positions
        def faulty(**kwargs):
            accepted = command(**kwargs)
            self.left.fault = 'motor fault'
            return accepted
        self.left.set_joint_positions = faulty
        result = self.rt.move_to_joints(wrist(.1), arm='left')
        self.assertFalse(result.ok)
        self.assertTrue(self.rt.faulted)
        support = result.diagnostics['per_arm']['left']['support']
        self.assertEqual(support['strategy'], 'native_safety_retained')
        self.assertFalse(support['available'])
        self.assertEqual(len(self.left.commands), 1)
        self.assertEqual(self.left.fault, 'motor fault')
        self.assert_support_retained()

    def test_non_joint_mode_invalid_feedback_and_rejected_hold_are_not_verified(self):
        unit = self.station.arms['left']
        status = self.left.get_status()
        for update in [dict(mode='PROTECT'), dict(q=np.full(6, np.nan)), dict(offline=[2])]:
            with self.subTest(update=update), patch.object(self.left, 'get_status', return_value=dict(status, **update)):
                receipt = preserve_support(unit, cancel_motion=True)
                self.assertFalse(receipt['available'] or receipt['hold_verified'])
                self.assertFalse(self.left.commands)
        with patch.object(self.left, 'set_joint_positions', return_value=False):
            receipt = preserve_support(unit, cancel_motion=True)
        self.assertFalse(receipt['available'] or receipt['request_accepted'] or receipt['hold_verified'])

    def test_unknown_submission_requests_hold_but_stays_faulted(self):
        command = self.left.set_joint_positions
        def unknown(**kwargs):
            accepted = command(**kwargs)
            if len(self.left.commands) == 1:
                raise TimeoutError('missing command response')
            return accepted
        self.left.set_joint_positions = unknown
        result = self.rt.move_to_joints(wrist(.1), arm='left')
        self.assertFalse(result.ok)
        self.assertTrue(self.rt.faulted)
        row = result.diagnostics['per_arm']['left']
        self.assertEqual(row['dispatch'], 'unknown')
        self.assertTrue(row['support']['request_accepted'])
        self.assert_support_retained()
