"""SAM3 segmentation service matching :mod:`cap_harness.providers.sam3.client`."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import asyncio
import base64
import functools
import io
import logging
from typing import Any

from fastapi import FastAPI, HTTPException
import numpy as np
from PIL import Image
from pydantic import BaseModel, Field
from sam3.model.sam3_image_processor import Sam3Processor
from sam3.model_builder import build_sam3_image_model
import torch
import uvicorn

from cap_harness.providers.service_runtime import install_service_runtime

from .wire import SEGMENT_PATH, SEGMENT_POINT_PATH
from .policy import (DEFAULT_TEXT_CONFIDENCE_THRESHOLD, TEXT_SCORE_SEMANTICS,
                     prompt_diagnostics, text_confidence_threshold)

LOGGER = logging.getLogger(__name__)
app = FastAPI(title="cap-harness SAM3 provider")
# Bound concurrent GPU/inference requests so one instance safely serves
# multiple simulator clients (limit from CAP_HARNESS_SERVICE_CONCURRENCY).
install_service_runtime(app, ready=lambda: _MODEL is not None and _PROCESSOR is not None)
_MODEL: Any | None = None
_PROCESSOR: Any | None = None
_DEVICE = "cuda"
_GPU_SEMAPHORE = asyncio.Semaphore(1)
_MAX_SEGMENTATIONS = 5
_TEXT_CONFIDENCE_THRESHOLD = DEFAULT_TEXT_CONFIDENCE_THRESHOLD


class GroundingDiagnosticsModel:
    """Observe unfiltered scores without changing the pinned processor's filter.

    Sam3Processor calls forward_grounding directly (not Module.forward), so a
    torch forward hook would miss it. This facade delegates the same single
    inference and retains only its scores, never a second set of full masks.
    """

    def __init__(self, model):
        self.model = model
        self.raw_scores = []

    def __getattr__(self, name):
        return getattr(self.model, name)

    def forward_grounding(self, **kwargs):
        output = self.model.forward_grounding(**kwargs)
        scores = (output["pred_logits"].sigmoid()
                  * output["presence_logit_dec"].sigmoid().unsqueeze(1)).squeeze(-1)
        self.raw_scores = _to_numpy(scores).reshape(-1).astype(float).tolist()
        return output


class SegmentRequest(BaseModel):
    image_base64: str
    text_prompt: str


class PointPromptRequest(BaseModel):
    image_base64: str
    point_coords: list[float]


class MaskData(BaseModel):
    mask_base64: str
    shape: list[int]
    box: list[float]
    score: float
    label: str


class SegmentResponse(BaseModel):
    results: list[MaskData]
    diagnostics: dict[str, Any] = Field(default_factory=dict)


class PointPromptResponse(BaseModel):
    scores: list[float]
    masks_base64: str
    masks_shape: list[int]
    masks_dtype: str


async def _run_serialized(function: Any, *args: Any) -> Any:
    async with _GPU_SEMAPHORE:
        loop = asyncio.get_running_loop()
        try:
            return await loop.run_in_executor(None, functools.partial(function, *args))
        finally:
            _release_cached_gpu_memory()


def _release_cached_gpu_memory() -> None:
    """Hand back the allocator's cached blocks after each inference.

    PyTorch keeps freed blocks in its own pool rather than returning them to the
    driver, which is right when a process owns the GPU and wrong when it shares
    one. This service shares: on a bench that also hosts a policy server, that
    server offloads itself to CPU while a CAP program runs, then reclaims ~22 GiB
    when the operator switches back.

    Measured on a 32 GiB card: idle this process holds ~4.2 GiB, and one
    CAP program's worth of segmentation grows it to 7.73 GiB that it then keeps.
    The policy server's reclaim needed 210 MiB and found 41 MiB free -- it lost
    by 170 MiB, and the control loop died mid-session taking the eval with it.
    The growth is cache, not live tensors, so returning it costs a slower first
    allocation on the next call and nothing else.
    """
    if _DEVICE.startswith("cuda") and torch.cuda.is_available():
        torch.cuda.empty_cache()


def _decode_image(payload: str) -> Image.Image:
    try:
        raw = base64.b64decode(payload, validate=True)
        return Image.open(io.BytesIO(raw)).convert("RGB")
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"Invalid image data: {exc}") from exc


def _to_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if getattr(value, "dtype", None) == torch.bfloat16:
        value = value.float()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


def _encode_raw(array: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(array).tobytes()).decode("ascii")


def _segment_text(image: Image.Image, prompt: str) -> SegmentResponse:
    diagnostics = dict(prompt_diagnostics(_MODEL.backbone.language_backbone, prompt),
                       provider="sam3", prompt_type="text",
                       score_semantics=TEXT_SCORE_SEMANTICS,
                       text_confidence_threshold=_TEXT_CONFIDENCE_THRESHOLD,
                       threshold_comparison=">", label_is_query=True)
    if not prompt.strip() or diagnostics["would_truncate"]:
        diagnostics["rejection_reason"] = "prompt_too_long" if prompt.strip() else "empty_prompt"
        return SegmentResponse(results=[], diagnostics=diagnostics)
    _PROCESSOR.model.raw_scores = []
    device_type = "cuda" if "cuda" in _DEVICE else "cpu"
    with torch.autocast(device_type, dtype=torch.bfloat16):
        state = _PROCESSOR.set_image(image)
        output = _PROCESSOR.set_text_prompt(state=state, prompt=prompt)

    diagnostics["raw_scores"] = _PROCESSOR.model.raw_scores

    masks = output.get("masks")
    boxes = output.get("boxes")
    scores = output.get("scores")
    if masks is None or boxes is None or scores is None:
        diagnostics["rejection_reason"] = "no_candidates"
        return SegmentResponse(results=[], diagnostics=diagnostics)

    mask_array = _to_numpy(masks)
    if mask_array.ndim == 4 and mask_array.shape[1] == 1:
        mask_array = mask_array[:, 0]
    box_array = _to_numpy(boxes)
    score_array = _to_numpy(scores).reshape(-1)
    results = [
        MaskData(
            mask_base64=_encode_raw(mask_array[index] > 0),
            shape=list(mask_array[index].shape),
            box=box_array[index].astype(float).tolist(),
            score=float(score_array[index]),
            label=prompt,
        )
        for index in range(len(score_array))
        if np.isfinite(score_array[index]) and score_array[index] > _TEXT_CONFIDENCE_THRESHOLD
        and np.any(mask_array[index] > 0)
    ]
    results.sort(key=lambda item: item.score, reverse=True)
    diagnostics["accepted_count"] = len(results)
    diagnostics["rejection_reason"] = None if results else "no_trusted_candidates"
    return SegmentResponse(results=results[:_MAX_SEGMENTATIONS], diagnostics=diagnostics)


def _segment_point(image: Image.Image, point: tuple[float, float]) -> PointPromptResponse:
    device_type = "cuda" if "cuda" in _DEVICE else "cpu"
    with torch.autocast(device_type, dtype=torch.bfloat16):
        state = _PROCESSOR.set_image(image)
        masks, scores, _ = _MODEL.predict_inst(
            state,
            point_coords=np.asarray([point], dtype=np.float32),
            point_labels=np.ones(1, dtype=np.int64),
            multimask_output=True,
        )

    mask_array = np.asarray(masks)
    score_array = np.asarray(scores).reshape(-1)
    if mask_array.size == 0 or score_array.size == 0:
        return PointPromptResponse(
            scores=[], masks_base64="", masks_shape=[0, 0, 0], masks_dtype="float32"
        )
    order = np.argsort(score_array)[::-1]
    order = order[:_MAX_SEGMENTATIONS]
    mask_array = mask_array[order]
    score_array = score_array[order]
    return PointPromptResponse(
        scores=score_array.astype(float).tolist(),
        masks_base64=_encode_raw(mask_array),
        masks_shape=list(mask_array.shape),
        masks_dtype=str(mask_array.dtype),
    )


@app.post(SEGMENT_PATH, response_model=SegmentResponse)
async def segment(request: SegmentRequest) -> SegmentResponse:
    if _PROCESSOR is None:
        raise HTTPException(status_code=503, detail="Model not initialized")
    try:
        return await _run_serialized(
            _segment_text, _decode_image(request.image_base64), request.text_prompt
        )
    except HTTPException:
        raise
    except Exception as exc:
        LOGGER.exception("SAM3 text segmentation failed")
        raise HTTPException(status_code=500, detail=f"Inference failed: {exc}") from exc


@app.post(SEGMENT_POINT_PATH, response_model=PointPromptResponse)
async def segment_point(request: PointPromptRequest) -> PointPromptResponse:
    if _PROCESSOR is None or _MODEL is None:
        raise HTTPException(status_code=503, detail="Model not initialized")
    if len(request.point_coords) != 2:
        raise HTTPException(status_code=400, detail="point_coords must contain [x, y]")
    try:
        point = (float(request.point_coords[0]), float(request.point_coords[1]))
        return await _run_serialized(_segment_point, _decode_image(request.image_base64), point)
    except HTTPException:
        raise
    except Exception as exc:
        LOGGER.exception("SAM3 point segmentation failed")
        raise HTTPException(status_code=500, detail=f"Inference failed: {exc}") from exc


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the cap-harness SAM3 service")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8114)
    parser.add_argument("--text-confidence-threshold", type=text_confidence_threshold,
                        default=DEFAULT_TEXT_CONFIDENCE_THRESHOLD)
    args = parser.parse_args()

    global _DEVICE, _MODEL, _PROCESSOR, _TEXT_CONFIDENCE_THRESHOLD
    _TEXT_CONFIDENCE_THRESHOLD = args.text_confidence_threshold
    _DEVICE = args.device
    if torch.cuda.is_available():
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        if "cuda" in args.device:
            torch.cuda.set_device(int(args.device.split(":")[-1]) if ":" in args.device else 0)
    LOGGER.info("Loading SAM3 model on %s", args.device)
    checkpoint = os.environ.get("SAM3_CHECKPOINT")
    if not checkpoint or not Path(checkpoint).is_file():
        raise RuntimeError("SAM3_CHECKPOINT must name an existing trained checkpoint")
    _MODEL = build_sam3_image_model(checkpoint_path=checkpoint, load_from_HF=False, enable_inst_interactivity=True).to(args.device)
    _PROCESSOR = Sam3Processor(GroundingDiagnosticsModel(_MODEL), device=args.device,
                              confidence_threshold=_TEXT_CONFIDENCE_THRESHOLD)
    LOGGER.info("SAM3 text confidence threshold > %s; point scores use separate semantics",
                _TEXT_CONFIDENCE_THRESHOLD)
    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
