import json
from pathlib import Path
import tempfile
import time
import unittest

import cv2
import numpy as np

from robol2l_deploy.arx_preview import RGBDPreviews
from robol2l_deploy.adapters.arx_dual import DualARXStation
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cameras import CameraOwner, Frame
from robol2l_deploy.cap.physical_task import DualJointTask
from robol2l_deploy.cap.program import ProgramExecutor
from robol2l_deploy.cap.recording import AttemptRecording
from robol2l_deploy.runtime import StationRuntime
from test_arx_dual import CONFIG, fixture_station


class FrameSource:
    def __init__(self, serial):
        self.serial = serial
        self.opens = 0

    def open(self):
        self.opens += 1

    def read(self):
        time.sleep(.015)
        return Frame(np.full((48, 64, 3), 50, np.uint8), np.full((48, 64), .5),
                     dict(serial=self.serial, fx=50, fy=50, ppx=32, ppy=24, captured_s=time.time()),
                     time.monotonic())

    def close(self):
        pass


class RGBDControlsTest(unittest.TestCase):
    def test_calibrated_observation_and_right_arm_coordinate_conversion(self):
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config = json.loads(CONFIG.read_text())
            station = DualARXStation(config,directory=CONFIG.parent)
            fake = fixture_station()
            self.addCleanup(fake.close)
            station.arms = fake.arms
            right = station.arms['right']
            right.driver.arm.q[:3] = [.02,.01,.03]
            previews = RGBDPreviews(config['cameras'],root/'bindings.json')
            previews.owners = {k:CameraOwner(FrameSource(v.source.serial)) for k,v in previews.owners.items()}
            station.previews = previews
            rt = StationRuntime(station,root/'runtime')
            self.addCleanup(rt.close)
            api = PublicAPI(rt,DualJointTask('Verify calibration').context,providers={})
            obs = api.get_observation()
            self.assertEqual(obs.robot_state.base_frame,'left_base')
            actual = obs.robot_state.end_effector_poses['right']
            base = station.extrinsics['T_left_base_from_right_base']
            flange = right._pose(right.driver.arm.q).as_matrix()
            np.testing.assert_allclose(actual.as_matrix(),
                base @ flange @ station.grasp_geometry.transforms['right'],atol=1e-12)
            camera = obs.cameras['right_wrist']
            self.assertEqual(camera.camera_pose.frame,'left_base')
            np.testing.assert_allclose(camera.camera_pose.as_matrix(),
                base @ flange @ station.extrinsics['T_right_flange_from_right_wrist_optical'],atol=1e-12)
            self.assertTrue(api.tools()['plan_motion'](actual,arm='right').ok)
            self.assertFalse(right.driver.arm.commands)
            with patch.object(right.driver.arm,'get_joint_velocities',return_value=np.ones(6)):
                self.assertIn('right_wrist', station.get_observation('right_wrist').cameras)
            owners = station.cameras
            owners['right_wrist'].source.serial = 'replaced-camera'
            with self.assertRaisesRegex(ValueError,'serial_mismatch'):
                station.get_observation('right_wrist')

    def test_shared_policy_preview_and_recording_without_extrinsics(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            station = fixture_station()
            serials = json.loads(CONFIG.read_text())['cameras']
            station._cfg['cameras'] = serials
            previews = RGBDPreviews(serials, root/'bindings.json')
            owners = {key: CameraOwner(FrameSource(owner.source.serial)) for key, owner in previews.owners.items()}
            previews.owners = owners
            station.previews = previews
            runtime = StationRuntime(station, root/'runtime')
            try:
                with AttemptRecording(runtime, root, serials, 'v00') as recording:
                    result = ProgramExecutor(runtime, DualJointTask('Observe RGB-D').context).execute(
                        """obs = get_observation()
result = {'frames': {n: [c.rgb.shape, c.depth_m.shape, c.camera_pose.frame] for n,c in obs.cameras.items()}, 'arms': list(obs.robot_state.joint_positions)}""",
                        root/'policy')
                    self.assertTrue(result['ok'], result['error'])
                    self.assertEqual(set(result['result']['arms']), {'left', 'right'})
                    for name, source in previews.describe()['roles'].items():
                        self.assertEqual(result['result']['frames'][name], [[48,64,3], [48,64], name+'_color_optical'])
                        self.assertIsNotNone(cv2.imdecode(np.frombuffer(previews.jpeg(name, source), np.uint8), 1))
                for name, row in recording.receipts().items():
                    self.assertGreater(row['frames'], 0)
                    self.assertTrue(row['written'])
                self.assertTrue(all(o.source.opens == 1 for o in owners.values()))
            finally:
                runtime.close()
