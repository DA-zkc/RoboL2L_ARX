"""Adapter contract checks with fake devices only."""
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
from robol2l_deploy.adapters.franky import FrankyDriver
from robol2l_deploy.adapters.franka_common import robot_state
from robol2l_deploy.runtime import OutcomeUnknown


class FrankyAdapterTests(unittest.TestCase):
    def setUp(self):
        self.path = SimpleNamespace(duration=1., sample=lambda t: ([t]*7, [0.1]*7, [0.01]*7))
        self.motion = Mock()
        self.motion.report.return_value = dict(completed=True, ok=True, control_duration_s=1.02)
        self.motion.trace.return_value = dict(q=[[0]*7], reference_time_s=[0.])
        self.library = SimpleNamespace(StationJointTrajectory=Mock(return_value=self.path),
            StationJointMotion=Mock(return_value=self.motion), Robot=Mock(), Gripper=Mock())
        self.driver = FrankyDriver('not-a-device', library=self.library)

    def test_construction_and_planning_do_not_connect_and_motion_requires_opt_in(self):
        plan = self.driver.plan([[0]*7, [.01]*7])
        self.assertEqual(plan['backend'], 'franky.RuckigJointTrajectory')
        self.library.Robot.assert_not_called()
        self.driver.robot=Mock()
        self.driver.gripper=SimpleNamespace(state=SimpleNamespace(max_width=.08))
        for action in (lambda: self.driver.execute(plan['token']),
                       lambda: self.driver.move_gripper(.04), lambda: self.driver.grasp(.04, 5)):
            with self.assertRaisesRegex(ValueError, 'not_commissioned'):
                action()
        self.assertIn(plan['token'], self.driver.trajectories)

    def test_execute_uses_registered_curve_and_synchronous_library_motion(self):
        self.driver.allow_motion = True
        plan = self.driver.plan([[0]*7, [.01]*7])
        self.driver.robot = Mock()
        with patch.object(self.driver, 'read', return_value={'q':[1]*7}):
            receipt = self.driver.execute(plan['token'])
        self.library.StationJointMotion.assert_called_once_with(self.path)
        self.driver.robot.move.assert_called_once_with(self.motion,
            asynchronous=False, limit_rate=False, cutoff_frequency=100.)
        self.assertTrue(receipt['ok'])
        self.assertIn(plan['token'], self.driver.trajectories)
        self.driver.robot.recover_from_errors.assert_not_called()

    def test_control_failure_preserves_trace_and_does_not_recover(self):
        self.driver.allow_motion = True
        plan = self.driver.plan([[0]*7, [.01]*7])
        self.driver.robot = Mock()
        self.driver.robot.move.side_effect = RuntimeError('simulated lost receipt')
        with tempfile.TemporaryDirectory() as folder:
            self.driver.trace_directory = Path(folder)
            with self.assertRaises(OutcomeUnknown) as caught:
                self.driver.execute(plan['token'])
            evidence = caught.exception.diagnostics
            self.assertEqual(evidence['driver'], 'franky')
            self.assertTrue(Path(evidence['control_trace']['path']).exists())
        self.driver.robot.recover_from_errors.assert_not_called()

    def test_operator_recovery_calls_the_library_once_and_rereads_state(self):
        self.driver.robot = Mock()
        with patch.object(self.driver, 'read', return_value={'q':[0]*7,'has_errors':False}) as read:
            state = self.driver.recover()
        self.driver.robot.recover_from_errors.assert_called_once_with()
        read.assert_called_once_with()
        self.assertFalse(state['has_errors'])

    def test_negative_terminal_receipt_is_not_promoted_to_success(self):
        self.driver.allow_motion = True
        plan = self.driver.plan([[0]*7, [.01]*7])
        self.driver.robot = Mock()
        self.motion.report.return_value = dict(completed=True, ok=False, control_duration_s=4.)
        with patch.object(self.driver, 'read', return_value={'q':[1]*7}):
            self.assertFalse(self.driver.execute(plan['token'])['ok'])

    def test_hand_uses_franky_gripper_with_existing_width_and_force_semantics(self):
        self.driver.allow_motion = True
        self.driver.robot = Mock()
        self.driver.gripper = Mock()
        self.driver.gripper.state = SimpleNamespace(is_grasped=True, width=.04,max_width=.08)
        self.assertTrue(self.driver.grasp(.04, 5)['is_grasped'])
        self.driver.gripper.grasp.assert_called_once_with(0., .02, 5., 0., .08)
        self.driver.move_gripper(.08)
        self.driver.gripper.move.assert_called_once_with(.08, .02)
        self.driver.gripper.homing.assert_not_called()

    def test_both_state_representations_keep_the_same_base_transform(self):
        matrix = np.eye(4); matrix[:3, 3] = [.1, -.2, .3]
        state = SimpleNamespace(q=[0]*7, q_d=[0]*7, dq=[0]*7, dq_d=[0]*7,
            O_T_EE=matrix.flatten(order='F'), O_T_EE_c=matrix.flatten(order='F'),
            F_T_EE=np.eye(4).flatten(), EE_T_K=np.eye(4).flatten(),
            cartesian_contact=[0]*6, O_F_ext_hat_K=[0]*6, tau_ext_hat_filtered=[0]*7,
            tau_J=[0]*7, tau_J_d=[0]*7, control_command_success_rate=1.,
            m_ee=.73, m_load=0., m_total=.73, F_x_Cee=[0]*3, F_x_Cload=[0]*3,
            F_x_Ctotal=[0]*3, I_ee=np.zeros(9), I_load=np.zeros(9), current_errors=False,
            robot_mode=1)
        hand = SimpleNamespace(width=.08, max_width=.08, is_grasped=False, temperature=25.)
        panda = robot_state(state, hand, 'panda_py')
        state.O_T_EE = SimpleNamespace(matrix=matrix)
        state.I_ee = np.zeros((3, 3))
        franky = robot_state(state, hand, 'franky')
        for key in ('T_base_ee', 'gripper', 'load', 'mode_name'):
            self.assertEqual(panda[key], franky[key])

    def test_factory_does_not_construct_or_fall_back_to_other_backend(self):
        from robol2l_deploy.adapters.drivers import make_driver
        with patch('robol2l_deploy.adapters.franky.FrankyDriver') as selected:
            result = make_driver('franky', 'not-a-device')
            self.assertIs(result, selected.return_value)
            selected.assert_called_once_with('not-a-device')
        with self.assertRaisesRegex(ValueError, 'unknown_robot_backend'):
            make_driver('uninstalled', 'not-a-device')

    def test_read_uses_error_bits_instead_of_truthy_errors_object(self):
        errors=SimpleNamespace(__getstate__=lambda: (False, False))
        self.assertTrue(bool(errors))
        self.driver.robot=SimpleNamespace(state=SimpleNamespace(current_errors=errors))
        self.driver.gripper=SimpleNamespace(state=object())
        with patch('robol2l_deploy.adapters.franky.robot_state') as convert:
            self.driver.read()
            self.assertFalse(convert.call_args.kwargs['has_errors'])
            errors.__getstate__=lambda: (False, True)
            self.driver.read()
            self.assertTrue(convert.call_args.kwargs['has_errors'])
            errors.__getstate__=lambda: None
            with self.assertRaisesRegex(ValueError,'invalid_franky_error_flags'):self.driver.read()


if __name__ == '__main__':
    unittest.main()
