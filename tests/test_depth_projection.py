"""Software-only depth selection contracts, including the public program path."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from cap_harness.contracts import CameraObservation, Pose, Segmentation
from cap_harness.depth import DepthSelectionError
from cap_harness.geometry import mask_to_point_cloud, crop_point_cloud
from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cap.program import ProgramExecutor, compact
from robol2l_deploy.cap.tasks import MockReachTask
from robol2l_deploy.runtime import StationRuntime


class DepthSelectionTests(unittest.TestCase):
    def setUp(self):
        depth = np.full((8, 10), .4)
        depth[:, 6:] = 4.
        depth[2, 2] = 3.
        depth[3, 2] = 0.
        self.camera = CameraObservation(np.arange(240, dtype=np.uint8).reshape(8, 10, 3),
            depth, np.array([[100, 0, 5], [0, 100, 4], [0, 0, 1]]), 'camera',
            Pose([1, 2, 3], [1, 0, 0, 0], 'base'))
        self.mask = Segmentation(np.ones((8, 10), bool), 'unverified', 1., 'camera', 'camera')

    def project(self, **kwargs):
        return mask_to_point_cloud(self.camera, self.mask, **kwargs)

    def test_default_preserves_all_positive_points_in_row_order_and_colors(self):
        cloud = self.project()
        rows, cols = np.nonzero(self.camera.depth_m > 0)
        np.testing.assert_array_equal(cloud.points[:, 2], self.camera.depth_m[rows, cols])
        np.testing.assert_array_equal(cloud.colors, self.camera.rgb[rows, cols])
        evidence = cloud.diagnostics['projection']
        self.assertEqual(evidence['selected_pixels'], 80)
        self.assertEqual(evidence['retained_pixels'], 79)
        self.assertEqual(evidence['input_depth_m']['max'], 4.)
        self.assertFalse(evidence['depth_accuracy_verified'])
        self.assertFalse(cloud.points.flags.writeable)

    def test_seed_keeps_smaller_target_not_largest_component(self):
        cloud = self.project(seed_px=[1, 1], max_depth_jump_m=.03)
        self.assertEqual(len(cloud.points), 46)
        self.assertTrue(np.all(cloud.points[:, 2] == .4))
        other = self.project(seed_px=[8, 1], max_depth_jump_m=.03)
        self.assertEqual(len(other.points), 32)
        self.assertTrue(np.all(other.points[:, 2] == 4.))
        self.assertEqual(cloud.diagnostics['projection']['removed_by_connectivity'], 33)
        self.assertTrue(self.mask.mask.all())
        self.assertEqual(self.camera.depth_m[2, 2], 3.)

    def test_range_is_camera_z_and_diagnostics_survive_transform_crop(self):
        cloud = self.project(depth_range_m=[.4, .5], target_frame='base')
        self.assertTrue(np.all(cloud.points[:, 2] == 3.4))
        evidence = cloud.diagnostics['projection']
        self.assertEqual(evidence['removed_by_depth_range'], 33)
        self.assertEqual(evidence['output_depth_m']['max'], .4)
        cropped = crop_point_cloud(cloud, [0, 0, 3], [1, 3, 4])
        self.assertEqual(cropped.diagnostics['projection'], evidence)
        self.assertEqual(cropped.diagnostics['crop']['retained_points'], len(cropped.points))
        self.assertEqual(compact(cloud)['diagnostics']['projection']['retained_pixels'], 46)

    def test_no_diagonal_or_mask_exterior_bridge(self):
        mask = np.zeros((8, 10), bool)
        mask[0, 0] = mask[1, 1] = mask[1, 2] = True
        self.mask = replace(self.mask, mask=mask)
        self.assertEqual(len(self.project(seed_px=[0, 0], max_depth_jump_m=.1).points), 1)

    def test_gradual_ramp_is_local_continuity_not_total_range(self):
        self.camera = replace(self.camera, depth_m=np.tile(np.arange(10)*.01+.1, (8, 1)))
        cloud = self.project(seed_px=[0, 0], max_depth_jump_m=.011)
        self.assertEqual(len(cloud.points), 80)
        bounded = self.project(depth_range_m=[.1, .135], seed_px=[0, 0], max_depth_jump_m=.011)
        self.assertEqual(len(bounded.points), 32)

    def test_empty_selection_and_invalid_seed_never_fallback(self):
        for options in ({'depth_range_m': [.5, .6]},
                        {'seed_px': [2, 3], 'max_depth_jump_m': .1},
                        {'seed_px': [8, 1], 'max_depth_jump_m': .1, 'depth_range_m': [.1, .5]}):
            with self.subTest(options=options), self.assertRaises(DepthSelectionError) as ctx:
                self.project(**options)
            self.assertEqual(ctx.exception.diagnostics['retained_pixels'], 0)
        self.mask = replace(self.mask, mask=np.zeros((8, 10), bool))
        with self.assertRaises(DepthSelectionError) as ctx:
            self.project()
        self.assertIsNone(ctx.exception.diagnostics['input_depth_m'])

    def test_invalid_options_are_rejected(self):
        for options in ({'depth_range_m': [1, 0]}, {'depth_range_m': [0, float('inf')]},
                        {'depth_range_m': [-1, 1]}, {'depth_range_m': [.1]},
                        {'depth_range_m': [False, True]}, {'depth_range_m': [0, 1j]},
                        {'seed_px': [1, 1]}, {'max_depth_jump_m': .1},
                        {'seed_px': [1.5, 1], 'max_depth_jump_m': .1},
                        {'seed_px': [-1, 1], 'max_depth_jump_m': .1},
                        {'seed_px': [1, 1], 'max_depth_jump_m': 0},
                        {'seed_px': [1, 1], 'max_depth_jump_m': .1j},
                        {'seed_px': [1, 1], 'max_depth_jump_m': True}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.project(**options)


class DepthAPITests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.runtime = StationRuntime(MockAdapter(camera_names=('survey',)), self.temp.name)
        self.api = PublicAPI(self.runtime, MockReachTask().context, providers={})
        obs = self.api.get_observation()
        camera = obs.cameras['survey']
        yy, xx = np.indices(camera.depth_m.shape)
        depth = .4 + .01*np.sin(xx*.1)*np.cos(yy*.1)
        depth[50, 70] = 4.
        self.api.environment.observation = replace(obs, cameras={'survey': replace(camera, depth_m=depth)})
        self.mask = Segmentation(np.ones(depth.shape, bool), 'unverified', 1., 'survey', camera.frame)

    def tearDown(self):
        self.runtime.close()
        self.temp.cleanup()

    def test_localization_reports_filters_and_empty_failure(self):
        result = self.api.core.localize_object('fixture', segmentation=self.mask,
            depth_range_m=[.3, .5], seed_px=[60, 60], max_depth_jump_m=.02)
        self.assertTrue(result.ok, result.error)
        self.assertEqual(result.diagnostics['projection']['removed_by_depth_range'], 1)
        self.assertIs(result.segmentation, self.mask)
        failure = self.api.core.localize_object('fixture', segmentation=self.mask, depth_range_m=[5, 6])
        self.assertFalse(failure.ok)
        self.assertTrue(failure.error.recoverable)
        self.assertEqual(failure.diagnostics['projection']['retained_pixels'], 0)
        self.assertIsNone(failure.geometry)

    def test_isolated_program_can_choose_filters_and_read_evidence(self):
        executor = ProgramExecutor(self.runtime, MockReachTask().context)
        executor.api = self.api
        with patch.object(self.api.environment, 'get_observation', return_value=self.api.environment.observation):
            report = executor.execute('''
obs = get_observation()
mask = obs.cameras["survey"].depth_m > 0
cloud = mask_to_point_cloud(mask, "survey", "base", depth_range_m=[.3,.5], seed_px=[60,60], max_depth_jump_m=.02)
geometry = estimate_geometry(cloud)
result = {"removed": cloud.diagnostics["projection"]["removed_by_depth_range"], "points": len(cloud.points)}
''', Path(self.temp.name)/'program')
        self.assertTrue(report['ok'], report['error'])
        self.assertEqual(report['result'], {'removed': 1, 'points': 19199})
        self.assertEqual(report['calls'][1]['output']['diagnostics']['projection']['input_depth_m']['max'], 4.)
        self.assertIn('depth_range_m', self.api.describe()['perception_input_guidance']['depth'])

    def test_too_few_points_keeps_quality_evidence_on_geometry_failure(self):
        mask = np.zeros_like(self.mask.mask)
        mask[60, 60:63] = True
        result = self.api.core.localize_object('fixture', segmentation=replace(self.mask, mask=mask))
        self.assertFalse(result.ok)
        self.assertIsNone(result.geometry)
        self.assertEqual(result.diagnostics['projection']['retained_pixels'], 3)
        self.assertEqual(result.error.code.value, 'point_cloud_failed')


if __name__ == '__main__':
    unittest.main()
