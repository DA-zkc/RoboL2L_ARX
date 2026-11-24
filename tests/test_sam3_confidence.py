"""Software-only SAM3 acceptance and orchestration regression tests."""
import base64
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
from cap_harness.providers.sam3.client import Sam3Provider
from cap_harness.providers.sam3.policy import text_confidence_threshold
from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cap.tasks import MockReachTask
from robol2l_deploy.runtime import StationRuntime
from robol2l_deploy.providers.perception import SegmentationEvidence


def encoded(array):
    return base64.b64encode(np.ascontiguousarray(array).tobytes()).decode('ascii')


def text_result(mask, score):
    return dict(mask_base64=encoded(mask.astype(np.uint8)), shape=list(mask.shape),
                box=[0, 0, mask.shape[1], mask.shape[0]], score=score, label='query echo')


class Sam3ConfidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.runtime = StationRuntime(MockAdapter(camera_names=('survey',)), self.temp.name)
        self.client = Sam3Provider(max_retries=0)
        self.client._http.post_json = Mock()
        self.api = PublicAPI(self.runtime, MockReachTask().context, providers={'segmentation': self.client})
        self.obs = self.api.get_observation()
        self.camera = self.obs.cameras['survey']
        yy, xx = np.indices(self.camera.depth_m.shape)
        self.camera = replace(self.camera, depth_m=.4 + .02*np.sin(xx*.1)*np.cos(yy*.13))
        self.obs = replace(self.obs, cameras={'survey': self.camera})
        self.api.environment.observation = self.obs
        self.mask = np.zeros(self.camera.depth_m.shape, bool)
        self.mask[40:65, 60:90] = True

    def tearDown(self):
        self.runtime.close()
        self.temp.cleanup()

    def response(self, scores, **diagnostics):
        self.client._http.post_json.return_value = dict(
            results=[text_result(self.mask, score) for score in scores], diagnostics=diagnostics)

    def test_low_scores_and_absent_target_never_project(self):
        for score in (0.00004410743713378906, 0.01068115234375, 0.0079345703125, 0.5):
            with self.subTest(score=score):
                self.response([score])
                with patch.object(self.api.core, '_project_mask') as project:
                    result = self.api.core.localize_object('query', camera_name='survey')
                self.assertFalse(result.ok)
                self.assertEqual(result.error.code.value, 'no_segmentation')
                self.assertTrue(result.error.recoverable)
                self.assertEqual(result.diagnostics['raw_scores'], [score])
                self.assertEqual(result.diagnostics['text_confidence_threshold'], .5)
                project.assert_not_called()

    def test_filter_then_sort_never_promotes_low_candidate(self):
        self.response([.1, .8, .5, .97, .3, .7, .6, .9])
        result = self.client.segment_text(self.camera, 'object')
        self.assertEqual([s.score for s in result.segmentations], [.97, .9, .8, .7, .6])
        self.assertEqual(len(result.diagnostics['rejected_candidates']), 3)

    def test_service_cannot_lower_client_threshold(self):
        self.response([.3, .8], text_confidence_threshold=.2)
        result = self.client.segment_text(self.camera, 'object')
        self.assertEqual([s.score for s in result.segmentations], [.8])
        self.response([.8], text_confidence_threshold=.9)
        result = self.client.segment_text(self.camera, 'object')
        self.assertFalse(result.ok)
        self.assertEqual(result.diagnostics['text_confidence_threshold'], .9)

    def test_no_candidates_preserves_service_rejected_scores(self):
        self.response([], raw_scores=[.01, .001], text_confidence_threshold=.5)
        result = self.client.segment_text(self.camera, 'absent')
        self.assertFalse(result.ok)
        self.assertEqual(result.error.details['raw_scores'], [.01, .001])

    def test_actual_and_original_query_recorded_and_geometry_not_identity(self):
        self.response([.9609375])
        result = self.api.core.localize_object('用户原查询', camera_name='survey', model_query='orange fruit')
        self.assertTrue(result.ok, result.error)
        self.assertEqual(self.client._http.post_json.call_args.args[1]['text_prompt'], 'orange fruit')
        self.assertEqual(result.diagnostics['original_query'], '用户原查询')
        self.assertEqual(result.diagnostics['model_query'], 'orange fruit')
        self.assertFalse(result.diagnostics['class_identity_verified'])
        self.assertEqual(result.diagnostics['success_semantics'], 'selected_mask_geometry')

    def test_length_rejection_survives_localization(self):
        self.response([], rejection_reason='prompt_too_long', tokens_including_specials=67,
                      context_length=32, would_truncate=True, truncated=False)
        result = self.api.core.localize_object('长提示', camera_name='survey')
        self.assertFalse(result.ok)
        self.assertEqual(result.error.code.value, 'invalid_request')
        self.assertEqual(result.diagnostics['tokens_including_specials'], 67)
        self.assertTrue(result.error.recoverable)

    def test_point_quality_not_filtered_by_text_threshold_and_mask_reused(self):
        # Deliberately below .5: point quality and text confidence are not interchangeable.
        self.client._http.post_json.return_value = dict(scores=[.4], masks_base64=encoded(self.mask[None]),
            masks_shape=[1, *self.mask.shape], masks_dtype='bool')
        selected = self.api.core.segment_points('survey', [[70, 50]])
        self.assertTrue(selected.ok)
        self.assertFalse(selected.diagnostics['text_confidence_threshold_applied'])
        with patch.object(self.client, 'segment_text', side_effect=AssertionError('must not resegment')):
            result = self.api.core.localize_object('果实', segmentation=selected.segmentations[0])
            direct = self.api.core.mask_to_point_cloud(selected.segmentations[0], 'survey', 'base')
        self.assertTrue(result.ok, result.error)
        np.testing.assert_array_equal(result.point_cloud.points, direct.points)
        np.testing.assert_array_equal(result.segmentation.mask, self.mask)
        self.assertEqual(result.diagnostics['selection_source'], 'caller_selected_mask')
        self.assertEqual(self.client._http.post_json.call_count, 1)

    def test_empty_point_response_is_recoverable(self):
        self.client._http.post_json.return_value = dict(scores=[], masks_base64='', masks_shape=[0,0,0], masks_dtype='float32')
        result = self.client.segment_points(self.camera, [[70, 50]])
        self.assertEqual(result.error.code.value, 'no_segmentation')

    def test_empty_text_mask_and_nonfinite_scores_do_not_pass(self):
        self.response([.9])
        self.client._http.post_json.return_value['results'][0] = text_result(np.zeros_like(self.mask), .9)
        self.assertEqual(self.client.segment_text(self.camera, 'object').error.code.value, 'no_segmentation')
        self.response([float('nan')])
        self.assertEqual(self.client.segment_text(self.camera, 'object').error.code.value, 'perception_failed')

    def test_invalid_thresholds_fail_configuration(self):
        for value in (0, 1, -.1, float('nan'), float('inf'), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                text_confidence_threshold(value)

    def test_evidence_records_rejection_diagnostics(self):
        self.response([], raw_scores=[.000044], text_confidence_threshold=.5)
        evidence = SegmentationEvidence(self.client, self.api.environment)
        result = evidence.segment_text(self.camera, '原查询', model_query='fruit')
        self.assertFalse(result.ok)
        record = json.loads(next((self.runtime.directory/'perception').glob('*/result.json')).read_text())
        self.assertEqual(record['diagnostics']['raw_scores'], [.000044])
        self.assertEqual(record['original_query'], '原查询')
        self.assertEqual(record['model_query'], 'fruit')

    def test_reuse_rejects_conflicting_inputs_and_camera(self):
        from cap_harness.contracts import Segmentation
        mask = Segmentation(self.mask, 'selected', .9, 'survey', self.camera.frame)
        result = self.api.core.localize_object('object', segmentation=mask, model_query='object')
        self.assertEqual(result.error.code.value, 'invalid_request')
        result = self.api.core.localize_object('object', camera_name='missing', segmentation=mask)
        self.assertEqual(result.error.code.value, 'invalid_request')

    def test_tool_description_explains_capability_and_reuse(self):
        description = self.api.describe()['perception_input_guidance']
        self.assertIn('32 tokens', description['text'])
        self.assertIn('exactly one positive', description['points'])
        self.assertIn('segmentation=s', description['reuse'])

    def test_isolated_agent_program_recovers_with_point_and_reuses_mask(self):
        from robol2l_deploy.cap.program import ProgramExecutor
        self.client._http.post_json.side_effect = [
            dict(results=[text_result(self.mask, .000044)]),
            dict(scores=[.96], masks_base64=encoded(self.mask[None]),
                 masks_shape=[1, *self.mask.shape], masks_dtype='bool')]
        executor = ProgramExecutor(self.runtime, MockReachTask().context)
        executor.api = self.api
        report = executor.execute('''
r = localize_object("原查询", camera_name="survey", model_query="fruit")
if not r.ok:
    points = segment_points("survey", [[70,50]])
    if points.ok:
        s = points.segmentations[0]
        cloud = mask_to_point_cloud(s, "survey", "base")
        geometry = estimate_geometry(cloud)
        r = localize_object("原查询", camera_name="survey", segmentation=s)
result = {"ok":r.ok, "source":r.diagnostics["selection_source"]}
''', Path(self.temp.name)/'program')
        self.assertTrue(report['ok'], report['error'])
        self.assertEqual(report['result'], {'ok': True, 'source': 'caller_selected_mask'})
        self.assertEqual(self.client._http.post_json.call_count, 2)
        self.assertEqual(report['calls'][0]['output']['error']['code'], 'no_segmentation')
        self.assertFalse(self.runtime.faulted)


if __name__ == '__main__':
    unittest.main()
