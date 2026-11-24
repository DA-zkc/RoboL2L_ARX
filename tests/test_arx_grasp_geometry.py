"""Synthetic-only TCP/FK/IK/receipt consistency, with nontrivial rotations."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from robol2l_deploy.adapters.arx_grasp_geometry import GraspGeometry
from robol2l_deploy.adapters.arx_gripper_model import MODEL, official_gripper_record
from robol2l_deploy.adapters.arx_dual import DualARXStation
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cap.program import ProgramExecutor
from robol2l_deploy.cap.tasks import MockReachTask
from robol2l_deploy.runtime import StationRuntime
from test_arx_adapter import FakeKinematics
from test_arx_dual import CONFIG, fixture_station


def transform(xyz, rpy):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


class RotatingKinematics(FakeKinematics):
    def fk(self, q):
        return SimpleNamespace(homogeneous=transform(q[:3], q[3:]))

    def ik(self, pose, q_init):
        return np.asarray(pose).copy(), True, 0.


class GraspGeometryTests(unittest.TestCase):
    def setUp(self):
        temp = TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.station = fixture_station()
        self.addCleanup(self.station.close)
        self.transforms = {
            'left': transform([.14, .012, -.025], [.2, -.4, .1]),
            'right': transform([.16, -.008, .018], [-.1, .3, -.2]),
        }
        self.data = dict(schema=1, station='arx-x5-2023-dual', ports=self.station.ports,
                         sides={side: dict(verified=True, opening_invariant=True,
                                          source='synthetic test fixture, NOT hardware calibration',
                                          T_link6_from_grasp_tcp=t.tolist())
                                for side, t in self.transforms.items()})
        self.path = self.root/'geometry.json'
        self.install()
        for i, unit in enumerate(self.station.arms.values()):
            unit.kin = RotatingKinematics()
            unit.driver.arm.q[:] = [.1, -.03, .2, .15, -.2, .1 + .1*i]

    def install(self):
        self.path.write_text(json.dumps(self.data))
        self.station.grasp_geometry = GraspGeometry(self.path, self.station.ports)

    def runtime(self):
        runtime = StationRuntime(self.station, self.root/'runtime', allow_motion=True)
        self.addCleanup(runtime.close)
        return runtime

    def test_missing_geometry_does_not_claim_tcp_or_change_flange(self):
        self.station.grasp_geometry = GraspGeometry(None, self.station.ports)
        self.station.arms['left'].gripper = (.02, 4.869344625)
        self.station.arms['right'].gripper = (.01, 4.7793)
        state = self.station.get_robot_state()
        capabilities = self.station.describe()
        self.assertTrue(capabilities['gripper_calibrated'])
        self.assertFalse(capabilities['grasp_geometry_ready'])
        for side, unit in self.station.arms.items():
            np.testing.assert_allclose(state.end_effector_poses[side].as_matrix(),
                                       unit._pose(unit.driver.arm.q).as_matrix())
            self.assertEqual(capabilities['end_effector_references'][side], 'link6')
            self.assertFalse(unit.driver.arm.commands)

    def test_configuration_rejects_nonrigid_and_wrong_binding(self):
        bad_values = [
            ('T_link6_from_grasp_tcp', np.diag([1, 1, -1, 1]).tolist()),
            ('T_link6_from_grasp_tcp', [[float('nan')]*4]*4),
            ('T_link6_from_grasp_tcp', np.eye(3).tolist()),
        ]
        for key, value in bad_values:
            with self.subTest(key=key, value=value):
                data = deepcopy(self.data)
                data['sides']['left'][key] = value
                self.path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    GraspGeometry(self.path, self.station.ports)
        self.path.write_text(json.dumps(dict(self.data, ports={'left': 'can3', 'right': 'can1'})))
        with self.assertRaisesRegex(ValueError, 'binding'):
            GraspGeometry(self.path, self.station.ports)

    def test_metadata_and_finite_offset_do_not_add_project_limits(self):
        for key, value in [('verified', False), ('opening_invariant', False), ('source', ''),
                           ('T_link6_from_grasp_tcp', transform([140, 0, 0], [0, 0, 0]).tolist())]:
            with self.subTest(key=key):
                data = deepcopy(self.data)
                data['sides']['left'][key] = value
                self.path.write_text(json.dumps(data))
                GraspGeometry(self.path, self.station.ports)

    def test_partial_geometry_is_explicit_and_does_not_change_the_other_arm(self):
        self.data['sides']['right'] = None
        self.install()
        refs = self.station.describe()['end_effector_references']
        self.assertEqual(refs, {'left': 'grasp_tcp', 'right': 'link6'})
        self.assertFalse(self.station.describe()['grasp_geometry_ready'])
        for side, pose in self.station.get_robot_state().end_effector_poses.items():
            q = self.station.arms[side].driver.arm.q
            solution = self.station.solve_ik(pose, arm=side)
            self.assertTrue(solution.ok, solution.error)
            np.testing.assert_allclose(solution.joint_positions, q, atol=1e-12)

    def test_rotated_tcp_state_ik_and_shared_right_base_round_trip(self):
        base = transform([.01, -.6, .008], [.01, -.02, .3])
        self.station.extrinsics = {'T_left_base_from_right_base': base}
        state = self.station.get_robot_state()
        for side, unit in self.station.arms.items():
            native = unit._pose(unit.driver.arm.q).as_matrix()
            expected = native @ self.transforms[side]
            if side == 'right':
                expected = base @ expected
            pose = state.end_effector_poses[side]
            np.testing.assert_allclose(pose.as_matrix(), expected, atol=1e-12)
            solved = self.station.solve_ik(pose, arm=side)
            self.assertTrue(solved.ok, solved.error)
            np.testing.assert_allclose(solved.joint_positions, unit.driver.arm.q, atol=1e-12)
            self.assertFalse(unit.driver.arm.commands)
        self.assertFalse(self.station.solve_ik(replace(pose, frame='right_base'), arm='left').ok)

    def test_public_single_pair_execution_and_goal_checks_use_tcp(self):
        self.station.extrinsics = {'T_left_base_from_right_base': transform([0, -.6, .01], [0, 0, .2])}
        rt = self.runtime()
        api = PublicAPI(rt, MockReachTask.context, providers={})
        targets = {s: replace(p, position=p.position + [.003, -.002, .004])
                   for s, p in rt.get_robot_state().end_effector_poses.items()}
        plan = api.tools()['plan_motion'](targets['right'], arm='right')
        self.assertTrue(plan.ok, plan.error)
        self.assertEqual(plan.diagnostics['end_effector_references']['right'], 'grasp_tcp')
        result = api.tools()['move_to_pose'](targets['right'], arm='right', tolerance=1e-7)
        self.assertTrue(result.ok, result.error)
        result = api.tools()['move_synchronized'](targets, tolerance=1e-7)
        self.assertTrue(result.ok, result.error)
        for side, unit in self.station.arms.items():
            row = result.diagnostics['per_arm'][side]
            self.assertEqual(row['pose_reference'], 'grasp_tcp')
            np.testing.assert_allclose(row['measured_pose'].as_matrix(),
                row['measured_flange_pose'].as_matrix() @ self.transforms[side], atol=1e-12)
            self.assertLess(result.diagnostics['goal_checks'][side]['position_error_m'], 1e-10)
            np.testing.assert_allclose(rt.get_robot_state().end_effector_poses[side].as_matrix(),
                                       targets[side].as_matrix(), atol=1e-10)
        self.assertEqual(api.environment.get_planning_context().end_effector_links,
                         {'left': 'grasp_tcp', 'right': 'grasp_tcp'})
        self.assertIn('Do not add a finger offset again', api.describe()['end_effector_semantics'])

    def test_camera_extrinsics_stay_on_flange_and_binding_tracks_geometry_contents(self):
        wrist = transform([.06, .01, .1], [.2, .4, -.1])
        self.station.extrinsics = {
            'T_left_base_from_right_base': transform([0, -.6, 0], [0, 0, .2]),
            'T_right_flange_from_right_wrist_optical': wrist,
        }
        unit = self.station.arms['right']
        flange = unit._pose(unit.driver.arm.q).as_matrix()
        expected = self.station.extrinsics['T_left_base_from_right_base'] @ flange @ wrist
        np.testing.assert_allclose(self.station.camera_pose('right_wrist', flange).as_matrix(), expected)
        old_calibration = self.station.calibration_binding()
        old_binding = self.station.device_binding()
        self.data['sides']['right']['T_link6_from_grasp_tcp'][0][3] += .01
        self.install()
        self.assertEqual(self.station.calibration_binding(), old_calibration)
        self.assertNotEqual(self.station.device_binding(), old_binding)
        np.testing.assert_allclose(self.station.camera_pose('right_wrist', flange).as_matrix(), expected)
        descriptor = self.station.describe()
        descriptor['grasp_geometry']['right']['geometry']['T_link6_from_grasp_tcp'][0][3] = 9
        self.assertNotEqual(self.station.grasp_geometry.records['right']['T_link6_from_grasp_tcp'][0][3], 9)

    def test_real_config_loads_model_without_hardware_and_generated_code_sees_tcp(self):
        config = json.loads(CONFIG.read_text())
        station = DualARXStation(config, directory=CONFIG.parent)
        self.addCleanup(station.close)
        self.assertTrue(station.describe()['grasp_geometry_ready'])
        for row in station.describe()['grasp_geometry'].values():
            self.assertFalse(row['physically_verified'])
            self.assertEqual(row['geometry']['verification'], 'official_model')
        self.assertTrue(all(unit.driver._arm is None for unit in station.arms.values()))
        rt = self.runtime()
        expected = rt.get_robot_state().end_effector_poses['left'].position.tolist()
        report = ProgramExecutor(rt, MockReachTask.context).execute(
            'result = get_robot_state().end_effector_poses["left"].position.tolist()', self.root/'policy')
        self.assertTrue(report['ok'], report['error'])
        np.testing.assert_allclose(report['result'], expected)
        self.assertTrue(all(not unit.driver.arm.commands for unit in self.station.arms.values()))

    def test_official_pad_center_axes_and_invariance_across_model_opening(self):
        record = official_gripper_record()
        tcp = np.array(record['T_link6_from_grasp_tcp'])
        np.testing.assert_allclose(tcp[:3, 3], [.149335045, -.000002, -.000713876], atol=1e-9)
        np.testing.assert_array_equal(tcp[:3, 2], [1, 0, 0])  # Approach along native +X.
        np.testing.assert_array_equal(tcp[:3, 0], [0, 1, 0])  # Closing axis native Y.
        pads = record['contact_geometry']['pads']
        for opening in [0, .005, .022, .044]:
            centers = [np.array(p['centroid_link6_m']) + np.array(p['axis_link6'])*opening
                       for p in pads]
            np.testing.assert_allclose(np.mean(centers, axis=0), tcp[:3, 3], atol=1e-12)
        # Foremost tips protrude beyond the TCP, which matters near a table.
        self.assertAlmostEqual(pads[0]['bounds_link6_m'][1][0] - tcp[0, 3], .008234957, places=8)
        self.assertFalse(record['contact_geometry']['motor_angle_to_width_calibrated'])

    def test_official_source_integrity_and_config_tamper_are_rejected(self):
        import shutil
        copied = self.root/'model'
        shutil.copytree(MODEL, copied)
        mesh = copied/'meshes/link7.STL'
        mesh.write_bytes(mesh.read_bytes() + b'changed')
        with self.assertRaisesRegex(ValueError, 'hash_mismatch'):
            official_gripper_record(copied)
        self.data['sides'] = {s: official_gripper_record() for s in self.station.ports}
        self.data['sides']['left']['T_link6_from_grasp_tcp'][0][3] += .01
        with self.assertRaisesRegex(ValueError, 'official_model_grasp_geometry_mismatch'):
            self.install()

    def test_official_tcp_with_native_sdk_fk_ik_is_offline_and_preserves_endpoints(self):
        station = DualARXStation(json.loads(CONFIG.read_text()), directory=CONFIG.parent)
        self.addCleanup(station.close)
        for side, unit in station.arms.items():
            expected_endpoints = (.02, 4.869344625009536) if side == 'left' else (.01, 4.7793)
            np.testing.assert_allclose(unit.gripper, expected_endpoints, atol=1e-10)
            for q in [np.array([.2, .6, 1.2, .3, .1, .2]), np.array([-.3, .8, 1., .5, -.2, -.4])]:
                flange = replace(unit._pose(q), frame=side+'_base')
                tcp = station.grasp_geometry.from_flange(side, flange)
                solved = station.solve_ik(tcp, seed=q, arm=side)
                self.assertTrue(solved.ok, solved.error)
                actual = station.grasp_geometry.from_flange(side, unit._pose(solved.joint_positions))
                np.testing.assert_allclose(actual.as_matrix(), tcp.as_matrix(), atol=1e-6)
            self.assertIsNone(unit.driver._arm)


if __name__ == '__main__':
    unittest.main()
