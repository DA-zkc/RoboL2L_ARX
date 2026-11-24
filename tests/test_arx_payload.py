"""Payload math, model binding, and SDK configuration tests; no physical devices."""
from contextlib import nullcontext
from copy import deepcopy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

from robol2l_deploy.adapters.arx import ARXDriver
from robol2l_deploy.adapters.arx_dual import DualARXStation
from robol2l_deploy.adapters.arx_payload import PayloadModels, combine, compose_urdf, file_sha256, read_link6

try:
    from bimanual import SingleArm
    from bimanual.core.model import get_arm_model
    from bimanual.core.kinematics import Kinematics
except ImportError:
    SingleArm = None

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'config/arx-x5-2023-dual-payload.json'


class PayloadTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads(CONFIG.read_text())
        self.extr = json.loads((CONFIG.parent / 'arx-x5-2023-dual-extrinsics.json').read_text())
        self.base = (CONFIG.parent / self.cfg['base_urdf']['path']).read_bytes()

    def load(self, path=CONFIG, **kwargs):
        return PayloadModels(path, kwargs.get('ports', self.extr['ports']),
                             kwargs.get('cameras', self.extr['cameras']),
                             kwargs.get('extrinsics', self.extr['transforms']))

    def temporary_config(self, cfg):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        cfg = deepcopy(cfg)
        cfg['base_urdf']['path'] = str((CONFIG.parent / cfg['base_urdf']['path']).resolve())
        for spec in cfg['sides'].values():
            spec['urdf_path'] = str((CONFIG.parent / spec['urdf_path']).resolve())
        path = Path(folder.name) / 'payload.json'
        path.write_text(json.dumps(cfg))
        return path

    def test_parallel_axis_and_weighted_center(self):
        mass, center, tensor = combine([(1, [-1, 0, 0], np.eye(3)), (3, [1, 0, 0], 2*np.eye(3))])
        self.assertEqual(mass, 4)
        np.testing.assert_allclose(center, [.5, 0, 0])
        np.testing.assert_allclose(tensor, np.diag([3, 6, 6]))

    def test_rejects_invalid_physical_inertia(self):
        for mass, inertia in [(float('nan'), np.eye(3)), (-1, np.eye(3)), (1, np.diag([1, 1, 4]))]:
            with self.assertRaises(ValueError):
                combine([(mass, [0, 0, 0], inertia)])

    def test_only_inertial_changes_and_gripper_not_counted_twice(self):
        loaded = self.load()
        for side in ('left', 'right'):
            root = ET.fromstring(Path(loaded.paths[side]).read_bytes())
            base = ET.fromstring(self.base)
            mass, _, _ = read_link6(root)
            self.assertAlmostEqual(mass - read_link6(base)[0], .068)
            for tree in (base, root):
                link = tree.find("link[@name='link6']")
                link.remove(link.find('inertial'))
            self.assertEqual(ET.tostring(base), ET.tostring(root))
        self.assertNotEqual(loaded.hashes['left'], loaded.hashes['right'])
        self.assertFalse(loaded.describe()['physically_verified'])
        self.assertEqual(loaded.describe()['sides']['left']['bracket']['status'], 'unknown')

    def test_rotates_camera_inertia_and_com(self):
        spec = deepcopy(self.cfg['sides']['left'])
        spec['T_link6_from_camera_optical'] = [[0,-1,0,.1],[1,0,0,.2],[0,0,1,.3],[0,0,0,1]]
        camera = dict(self.cfg['camera'], box_dimensions_m=[.02,.04,.06], nominal_com_optical_m=[.01,.02,.03])
        data, report = compose_urdf(self.base, spec, camera)
        np.testing.assert_allclose(report['camera_com_link6_m'], [.08,.21,.33])
        camera_tensor = np.diag(.068/12*np.array([.02**2+.06**2, .04**2+.06**2, .02**2+.04**2]))
        expected = combine([read_link6(ET.fromstring(self.base)), (.068, [.08,.21,.33], camera_tensor)])
        for actual, value in zip(read_link6(ET.fromstring(data)), expected):
            np.testing.assert_allclose(actual, value)

    def test_bracket_is_configurable_and_requires_source(self):
        spec = deepcopy(self.cfg['sides']['left'])
        # Synthetic mass properties; never a claim about the installed bracket.
        part = dict(mass_kg=.02, com_link6_m=[.06,0,.08], inertia_com_link6_kg_m2=(np.eye(3)*1e-5).tolist())
        spec['additional_components'] = [part]
        with self.assertRaisesRegex(ValueError, 'requires_source'):
            compose_urdf(self.base, spec, self.cfg['camera'])
        part['source'] = 'synthetic test only'
        _, result = compose_urdf(self.base, spec, self.cfg['camera'])
        self.assertAlmostEqual(result['link6_mass_kg'], .677997204780952)

    def test_rejects_binding_changes_and_stale_generated_models(self):
        with self.assertRaisesRegex(ValueError, 'station_binding'):
            self.load(ports=dict(left='can3', right='can1'))
        with self.assertRaisesRegex(ValueError, 'camera_mount_binding'):
            self.load(cameras=dict(self.extr['cameras'], left_wrist='different'))
        transforms = deepcopy(self.extr['transforms'])
        transforms['T_left_flange_from_left_wrist_optical'][0][3] += .001
        with self.assertRaisesRegex(ValueError, 'camera_mount_binding'):
            self.load(extrinsics=transforms)
        cfg = deepcopy(self.cfg)
        cfg['camera']['mass_kg'] += .001
        with self.assertRaisesRegex(ValueError, 'generated_urdf_changed'):
            self.load(self.temporary_config(cfg))
        cfg = deepcopy(self.cfg)
        cfg['base_urdf']['sha256'] = 'bad'
        with self.assertRaisesRegex(ValueError, 'base_urdf_changed'):
            self.load(self.temporary_config(cfg))

    def test_driver_passes_model_to_sdk_and_checks_before_connection(self):
        models = self.load()
        driver = ARXDriver('can1', allow_connection=True, acquire_lock=nullcontext,
                           urdf_path=models.paths['left'], urdf_sha256=models.hashes['left'])
        sentinel = object()
        factory = unittest.mock.Mock(return_value=sentinel)
        with patch.dict('sys.modules', {'bimanual': SimpleNamespace(SingleArm=factory)}), patch.object(driver, 'realtime_ready', return_value=True):
            self.assertIs(driver.arm, sentinel)
        factory.assert_called_once_with(dict(type=0, can_port='can1', urdf_path=models.paths['left']))
        driver._arm = None  # No real SDK object was created; avoid fake close methods.
        with self.assertRaisesRegex(ValueError, 'urdf_changed'):
            ARXDriver('can1', urdf_path=models.paths['left'], urdf_sha256='bad')
        driver.urdf_sha256 = 'bad'
        with patch.dict('sys.modules', {'bimanual': SimpleNamespace(SingleArm=factory)}), patch.object(driver, 'realtime_ready', return_value=True):
            with self.assertRaisesRegex(ValueError, 'urdf_changed'):
                _ = driver.arm
        self.assertEqual(factory.call_count, 1)

    def test_payload_changes_device_binding_but_not_handeye_binding(self):
        cfg = json.loads((CONFIG.parent / 'arx-x5-2023-dual.json').read_text())
        cfg['commissioning'] = None  # Hardware-independent fixture; no SDK needed to validate Home.
        station = DualARXStation(cfg, directory=CONFIG.parent)
        default = DualARXStation(dict(cfg, payload=None), directory=CONFIG.parent)
        self.addCleanup(station.close)
        self.addCleanup(default.close)
        self.assertEqual(station.calibration_binding(), default.calibration_binding())
        self.assertNotEqual(station.device_binding(), default.device_binding())
        for side, unit in station.arms.items():
            self.assertEqual(unit.driver.model_config()['urdf_path'], station.payload.paths[side])
            self.assertIsNone(unit.driver._arm)

    @unittest.skipIf(SingleArm is None, 'installed SDK needed for offline model check')
    def test_sdk_fk_unchanged_and_gravity_matches_added_camera(self):
        models = self.load()
        def kin(path):
            model = get_arm_model(dict(type=0, urdf_path=str(Path(path).resolve())))
            return Kinematics(model.urdf_path, model.ee_link, model.urdf_joint_names,
                              verbose=False, doc_limits_deg=model.doc_limits_deg)
        base = kin(CONFIG.parent / self.cfg['base_urdf']['path'])
        for side, path in models.paths.items():
            payload = kin(path)
            point = np.array(models.record['sides'][side]['computed']['camera_com_link6_m'] + [1.])
            for q in ([.1,1.1,1.2,-.2,.3,.1], [-.2,.4,.7,.5,-.3,.2], [.4,1.5,.9,-.4,.2,-.6]):
                q = np.array(q)
                np.testing.assert_allclose(base.fk(q).homogeneous, payload.fk(q).homogeneous, atol=1e-12)
                np.testing.assert_array_equal(base.q_min, payload.q_min)
                np.testing.assert_array_equal(base.q_max, payload.q_max)
                def height(joints):
                    return (base.T_home.homogeneous @ base.fk(joints).homogeneous @ point)[2]
                # Independently differentiate gravitational potential energy.
                expected = np.array([.068*9.81*(height(q+axis*1e-6)-height(q-axis*1e-6))/2e-6
                                     for axis in np.eye(6)])
                np.testing.assert_allclose(payload.gravity_torque(q)-base.gravity_torque(q), expected, atol=1e-7)

    @unittest.skipIf(SingleArm is None, 'installed SDK needed for constructor wiring check')
    def test_sdk_forwards_override_to_controller_without_creating_controller(self):
        models = self.load()
        class Intercepted(Exception):
            pass
        received = []
        def intercept(*args, **kwargs):
            received.append(kwargs)
            raise Intercepted()
        # Execute only the wrapper with a plain Python self; intercept before any CAN/controller construction.
        for side, path in models.paths.items():
            with patch.dict(SingleArm.__init__.__globals__, ArmController=intercept):
                with self.assertRaises(Intercepted):
                    SingleArm.__init__(SimpleNamespace(), dict(type=0, can_port='NO_HARDWARE', urdf_path=path))
            self.assertEqual(received[-1]['model'].urdf_path, path)
            self.assertEqual(received[-1]['rate_hz'], 200.)


if __name__ == '__main__':
    unittest.main()
