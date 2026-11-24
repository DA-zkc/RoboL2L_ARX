import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from robol2l_deploy.adapters import make_station_adapter
from robol2l_deploy.adapters.arx import ARXStationAdapter, EMBODIMENT, JOINT_NAMES
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cap.program import ProgramExecutor
from robol2l_deploy.cap.tasks import MockReachTask
from robol2l_deploy.cameras import Frame
from robol2l_deploy.contracts import Trajectory
from robol2l_deploy.errors import ApiError
from robol2l_deploy.runtime import StationRuntime


class FakeArm:
    fault = None
    offline_joints = []

    def __init__(self):
        self.q = np.zeros(6)
        self.gripper = 1.
        self.commands = []
        self.protected = False

    def get_joint_positions(self, joint_names):
        if joint_names != JOINT_NAMES:
            raise AssertionError('Unexpected joint names')
        return self.q.copy()

    def get_joint_velocities(self, joint_names):
        return np.zeros(6)

    def get_gripper_pos(self):
        return self.gripper

    def get_gripper_vel(self):
        return 0.

    def native_query(self, name, **arguments):
        if name == 'reference_status':
            return dict(joints_finished=True, gripper_finished=True,
                        source='synthetic_native_reference', physical_arrival_verified=False)
        if name == 'plan_duration':
            return max(arguments['minimum'], float(np.max(np.abs(
                np.asarray(arguments['target'])-arguments['start']))))
        if name == 'joint_path':
            return np.linspace(arguments['start'], arguments['target'],
                               int(np.ceil(arguments['minimum']/.01))+1).tolist()
        raise ValueError(name)

    def get_status(self):
        return dict(mode='PROTECT' if self.protected else 'JOINT', fault=self.fault,
                    offline=self.offline_joints, q=self.q.copy(),
                    v=self.get_joint_velocities(JOINT_NAMES))

    def set_joint_positions(self, *, positions, joint_names, duration):
        self.commands.append(('arm', positions.copy(), duration))
        self.q = positions.copy()
        return True

    def set_gripper_pos(self, target, duration):
        self.commands.append(('gripper', target, duration))
        self.gripper = target
        return True

    def protect_mode(self):
        self.protected = True
        return True


class FakeKinematics:
    q_min = np.full(6, -2.)
    q_max = np.full(6, 2.)
    v_max = np.ones(6)
    origin = np.eye(4)
    origin[0, 3] = .1
    T_home = SimpleNamespace(homogeneous=origin)
    T_home_inv = SimpleNamespace(homogeneous=np.linalg.inv(origin))

    def fk(self, q):
        transform = np.eye(4)
        transform[:3, 3] = q[:3]
        return SimpleNamespace(homogeneous=transform)

    def ik(self, pose, q_init):
        joints = q_init.copy()
        joints[:3] = pose[:3]
        return joints, True, 0.


class ARXAdapterTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name)
        self.arm = FakeArm()
        self.driver = SimpleNamespace(arm=self.arm, allow_motion=True, close=lambda: None)
        self.adapter = self.make_adapter()
        self.runtime = StationRuntime(self.adapter, self.folder / 'runtime', allow_motion=True)
        self.addCleanup(self.runtime.close)

    def make_adapter(self, port='can1', **settings):
        adapter = ARXStationAdapter(dict(can_port=port, **settings), directory=self.folder, driver=self.driver)
        adapter.kin = FakeKinematics()
        adapter.synthetic = True
        return adapter

    def fast_feedback(self, duration):
        return patch('robol2l_deploy.adapters.arx.time.monotonic', side_effect=(0., duration))

    def test_factory_defaults_to_disconnected_and_hides_uncalibrated_tools(self):
        config = Path(__file__).resolve().parents[1] / 'config/arx-x5-2023.json'
        adapter = make_station_adapter(config, EMBODIMENT)
        self.addCleanup(adapter.close)
        with self.assertRaisesRegex(ApiError, 'explicit_enable'):
            adapter.get_robot_state()
        api = PublicAPI(self.runtime, MockReachTask.context)
        self.assertNotIn('get_observation', api.tools())
        self.assertNotIn('set_gripper', api.tools())
        self.assertNotIn('close_gripper', api.tools())
        self.assertNotIn('step', api.tools())
        self.assertIn('move_to_joints', api.tools())

    def test_base_frame_fk_and_ik_translate_the_sdk_home_origin(self):
        state = self.runtime.get_robot_state()
        self.assertEqual(state.embodiment, EMBODIMENT)
        np.testing.assert_allclose(state.end_effector_poses['primary'].position, [.1, 0, 0])
        plan = self.runtime.plan_motion(dict(position=[.12, 0, 0], quaternion_wxyz=[1, 0, 0, 0], frame='base'))
        self.assertTrue(plan.ok, plan.error)
        self.assertEqual(plan.trajectory.joint_positions.shape, (2, 6))
        np.testing.assert_allclose(plan.trajectory.joint_positions[-1], [.02, 0, 0, 0, 0, 0])

    def test_ik_preserves_native_residual_on_success_failure_and_worker_boundary(self):
        # A converged result is not rejected by a new project residual cutoff.
        for converged, residual in ((True, .37), (False, 1.602628)):
            with self.subTest(converged=converged), patch.object(
                    self.adapter.kin, 'ik', return_value=(np.zeros(6), converged, residual)):
                report = ProgramExecutor(self.runtime, MockReachTask.context).execute(
                    "result = solve_ik(Pose([.1,0,0],[1,0,0,0],'base'), backend='station')",
                    self.folder / ('ik-success' if converged else 'ik-failure'))
                self.assertTrue(report['ok'], report['error'])
                result = report['result']
                self.assertEqual(result['ok'], converged)
                self.assertEqual(result['diagnostics']['sdk_residual'], residual)
                self.assertEqual(result['diagnostics']['sdk_converged'], converged)
                self.assertEqual(result['embodiment'], EMBODIMENT)
                if not converged:
                    self.assertIsNone(result['joint_positions'])
                    self.assertEqual(result['error']['details']['sdk_residual'], residual)
                    self.assertTrue(result['error']['recoverable'])
                self.assertFalse(self.arm.commands)

    def test_pre_solver_rejection_does_not_invent_a_native_residual(self):
        with patch.object(self.adapter.kin, 'ik') as solve:
            result = self.adapter.solve_ik(dict(position=[.1,0,0], quaternion_wxyz=[1,0,0,0], frame='wrong'))
        solve.assert_not_called()
        self.assertFalse(result.ok)
        self.assertEqual(dict(result.diagnostics), {})

    def test_cap_worker_executes_six_joint_move_with_measured_receipt(self):
        executor = ProgramExecutor(self.runtime, MockReachTask.context)
        report = executor.execute('result = move_to_joints([0, 0, 0, 0, 0, 0.01])', self.folder / 'program')
        self.assertTrue(report['ok'], report['error'])
        self.assertTrue(report['synthetic'])
        self.assertTrue(report['result']['ok'], report['result'].get('error'))
        diagnostics = report['result']['diagnostics']
        self.assertEqual(diagnostics['native_receipt']['source'], 'synthetic_ARX_X5_2023')
        self.assertEqual(len(diagnostics['native_receipt']['q']), 6)
        self.assertEqual(diagnostics['endpoint_accuracy']['max_joint_error_rad'], 0.)
        self.assertEqual(len(self.arm.commands), 1)

    def test_illegal_limits_waypoints_and_stale_starts_never_dispatch(self):
        self.assertFalse(self.runtime.plan_motion([3, 0, 0, 0, 0, 0]).ok)
        state = self.runtime.get_robot_state()
        path = Trajectory(np.zeros((3, 6)), 1., JOINT_NAMES, 'user', False, state, embodiment=EMBODIMENT)
        self.assertFalse(self.runtime.execute_trajectory(path).ok)
        plan = self.runtime.plan_motion([.05, 0, 0, 0, 0, 0])
        self.assertTrue(plan.ok, plan.error)
        self.arm.q[0] = .1
        self.assertTrue(self.runtime.execute_trajectory(plan.trajectory).ok)
        self.assertEqual(len(self.arm.commands), 1)

    def test_gripper_calibration_can_reverse_encoder_direction(self):
        adapter = self.make_adapter(gripper={'closed_rad': 4., 'open_rad': 1.})
        self.addCleanup(adapter.close)
        with self.fast_feedback(2.):
            self.assertTrue(adapter.set_gripper(.25).ok)
        self.assertEqual(self.arm.commands, [('gripper', 3.25, None)])
        self.assertAlmostEqual(adapter.get_robot_state().gripper_positions['primary'], .25)

    def test_encoder_boundary_tolerance_applies_only_to_motion_start(self):
        self.adapter.kin.q_min = np.array([-2., 0., 0., -2., -2., -2.])
        self.arm.q[2] = -.001
        target = [0., 0., 0., 0., 0., .02]
        plan = self.runtime.plan_motion(target, interpolation_dt_s=.8)
        self.assertTrue(plan.ok, plan.error)
        self.assertEqual(plan.trajectory.joint_positions[0, 2], -.001)
        self.assertEqual(plan.trajectory.joint_positions[-1, 2], 0.)
        illegal_target = target.copy()
        illegal_target[2] = -.001
        self.assertFalse(self.runtime.plan_motion(illegal_target).ok)
        self.assertTrue(self.runtime.execute_trajectory(plan.trajectory).ok)
        self.arm.q[2] = -.01
        self.assertTrue(self.runtime.plan_motion(target).ok)

    def test_wrist_observation_uses_the_x5_end_effector_transform(self):
        mount = np.eye(4)
        mount[1, 3] = .05
        calibration = self.folder / 'wrist.json'
        calibration.write_text(json.dumps(dict(camera_serial='fixture', units='meters',
            parent_frame='EE', status='validated', T_EE_camera=mount.tolist(),
            intrinsics={'width': 2, 'height': 2})))
        adapter = self.make_adapter(cameras={'wrist': {'driver': 'gemini', 'serial': 'fixture',
                                                       'calibration': calibration.name}})
        self.addCleanup(adapter.close)
        frame = Frame(np.zeros((2, 2, 3), dtype=np.uint8), np.ones((2, 2)),
            dict(serial='fixture', fx=1., fy=1., ppx=0., ppy=0., width=2, height=2), 1.)
        with patch.object(adapter.cameras['wrist'], 'latest', return_value=frame):
            obs = adapter.get_observation()
        self.assertEqual(obs.cameras['wrist'].camera_pose.frame, 'base')
        np.testing.assert_allclose(obs.cameras['wrist'].camera_pose.position, [.1, .05, 0.])

    def test_home_reference_is_scoped_to_the_selected_arm(self):
        self.runtime.get_robot_state()
        other = StationRuntime(self.make_adapter('can3'), self.runtime.directory)
        self.addCleanup(other.close)
        self.arm.q[0] = .1
        other.get_robot_state()
        self.assertEqual(other.home['device_binding'], EMBODIMENT + ':can3')
        self.assertEqual(other.home['joint_positions']['primary'][0], .1)
