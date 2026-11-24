"""Pinned tokenizer/processor, fake tensors only: no weights, GPU or HTTP service."""
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

try:
    import torch
    from PIL import Image
    from sam3.model.tokenizer_ve import SimpleTokenizer
    from cap_harness.providers.sam3 import service
except ImportError as exc:
    raise unittest.SkipTest(f'run in venv-sam3 with PYTHONPATH=.:vendor/sam3: {exc}')

import numpy as np


class Sam3ServiceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[1]
        cls.encoder = SimpleNamespace(context_length=32, tokenizer=SimpleTokenizer(
            bpe_path=str(root/'vendor/sam3/sam3/assets/bpe_simple_vocab_16e6.txt.gz')))

    def setUp(self):
        self.scores = torch.tensor([.0000441074, .01068115, .5, .9609375])
        self.model = SimpleNamespace(
            backbone=SimpleNamespace(language_backbone=self.encoder,
                forward_text=Mock(return_value={}), forward_image=Mock(return_value={})),
            inst_interactive_predictor=None,
            _get_dummy_prompt=Mock(return_value=None),
            forward_grounding=Mock(return_value=dict(
                pred_logits=torch.logit(self.scores).reshape(1,-1,1),
                presence_logit_dec=torch.tensor([[100.]]),
                pred_boxes=torch.tensor([[[.5,.5,.5,.5]]*4]),
                pred_masks=torch.ones(1,4,2,2))),
        )
        self.facade = service.GroundingDiagnosticsModel(self.model)
        processor = service.Sam3Processor(self.facade, device='cpu', resolution=8, confidence_threshold=.5)
        self.context = patch.multiple(service, _MODEL=self.model, _PROCESSOR=processor,
                                      _DEVICE='cpu', _TEXT_CONFIDENCE_THRESHOLD=.5)
        self.context.start()
        self.addCleanup(self.context.stop)
        self.image = Image.fromarray(np.zeros((8,8,3), np.uint8))

    def test_real_processor_threshold_retains_raw_scores_without_second_inference(self):
        response = service._segment_text(self.image, 'orange fruit')
        self.assertEqual(len(response.results), 1)
        self.assertAlmostEqual(response.results[0].score, .9609375, places=6)
        np.testing.assert_allclose(response.diagnostics['raw_scores'], self.scores.numpy(), rtol=1e-5)
        self.assertEqual(self.model.forward_grounding.call_count, 1)
        self.assertEqual(service._PROCESSOR.confidence_threshold, .5)
        self.assertEqual(response.diagnostics['text_confidence_threshold'], .5)

    def test_long_chinese_prompt_rejected_before_image_or_model(self):
        response = service._segment_text(self.image, '图像中央唯一的亮绿色立方体，包含绿色顶面和绿色正面')
        self.assertEqual(response.results, [])
        self.assertEqual(response.diagnostics['rejection_reason'], 'prompt_too_long')
        self.assertEqual(response.diagnostics['tokens_including_specials'], 67)
        self.assertFalse(response.diagnostics['truncated'])
        self.model.backbone.forward_image.assert_not_called()
        self.model.forward_grounding.assert_not_called()

    def test_31_token_chinese_query_is_not_categorically_rejected(self):
        response = service._segment_text(self.image, '图像中央的橙色柿子水果')
        self.assertEqual(response.diagnostics['tokens_including_specials'], 31)
        self.assertFalse(response.diagnostics['would_truncate'])
        self.assertEqual(len(response.results), 1)

    def test_all_below_threshold_retains_scores_and_rejection(self):
        self.model.forward_grounding.return_value['pred_logits'] = torch.full((1,4,1), -10.)
        response = service._segment_text(self.image, 'absent object')
        self.assertEqual(response.results, [])
        self.assertEqual(response.diagnostics['rejection_reason'], 'no_trusted_candidates')
        self.assertEqual(len(response.diagnostics['raw_scores']), 4)

    def test_exact_token_boundary(self):
        from cap_harness.providers.sam3.policy import prompt_diagnostics
        # Repeated common words each encode to one token in the pinned tokenizer.
        self.assertFalse(prompt_diagnostics(self.encoder, ' '.join(['fruit']*30))['would_truncate'])
        self.assertTrue(prompt_diagnostics(self.encoder, ' '.join(['fruit']*31))['would_truncate'])


if __name__ == '__main__':
    unittest.main()
