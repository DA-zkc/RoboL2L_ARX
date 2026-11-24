"""Typed SAM3 client for the provider service wire format."""

from __future__ import annotations

import base64
import binascii
from collections.abc import Mapping
import struct
from typing import Any
import zlib

import numpy as np
import requests

from cap_harness.contracts import CameraObservation, Segmentation, SegmentationSet
from cap_harness.errors import ApiError, ErrorCode
from cap_harness.providers.http import HttpProviderClient, ProviderHttpError
from cap_harness.providers.sam3.wire import SEGMENT_PATH, SEGMENT_POINT_PATH
from .policy import (DEFAULT_TEXT_CONFIDENCE_THRESHOLD, TEXT_SCORE_SEMANTICS,
                     POINT_SCORE_SEMANTICS, text_confidence_threshold)

DEFAULT_SAM3_URL = "http://127.0.0.1:8114"
MAX_SEGMENTATIONS = 5


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    checksum = binascii.crc32(kind)
    checksum = binascii.crc32(payload, checksum) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)


def _encode_rgb_png(image: np.ndarray) -> str:
    """Encode an RGB array without adding Pillow as a runtime dependency."""
    if np.issubdtype(image.dtype, np.floating):
        rgb = np.rint(image * 255.0).astype(np.uint8)
    else:
        rgb = image.astype(np.uint8, copy=False)
    height, width, _ = rgb.shape
    scanlines = b"".join(b"\x00" + row.tobytes(order="C") for row in rgb)
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    png = (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", header)
        + _png_chunk(b"IDAT", zlib.compress(scanlines))
        + _png_chunk(b"IEND", b"")
    )
    return base64.b64encode(png).decode("ascii")


def _decode_raw_array(value: object, *, dtype: np.dtype[Any], shape: tuple[int, ...]) -> np.ndarray:
    if not isinstance(value, str):
        # Preserve the invalid-data ValueError contract.
        raise ValueError("base64 array payload must be a string")
    try:
        raw = base64.b64decode(value, validate=True)
    except (ValueError, binascii.Error) as exc:
        raise ValueError("invalid base64 array payload") from exc
    expected_bytes = int(np.prod(shape, dtype=np.int64)) * dtype.itemsize
    if len(raw) != expected_bytes:
        raise ValueError(
            f"decoded array has {len(raw)} bytes, expected {expected_bytes} for shape {shape}"
        )
    return np.frombuffer(raw, dtype=dtype).reshape(shape)


class Sam3Provider:
    """Typed client for SAM3 on port 8114.

    ``CameraObservation`` does not carry a separate dictionary key, so its stable
    camera frame is also used as the segmentation's camera identity.
    """

    def __init__(
        self,
        base_url: str = DEFAULT_SAM3_URL,
        *,
        timeout_s: float | tuple[float, float] = 120.0,
        max_retries: int = 2,
        retry_backoff_s: float = 0.1,
        session: requests.Session | None = None,
        text_threshold: float = DEFAULT_TEXT_CONFIDENCE_THRESHOLD,
    ) -> None:
        self.text_threshold = text_confidence_threshold(text_threshold)
        self._http = HttpProviderClient(
            base_url,
            timeout_s=timeout_s,
            max_retries=max_retries,
            retry_backoff_s=retry_backoff_s,
            session=session,
        )

    @property
    def base_url(self) -> str:
        return self._http.base_url

    def health(self) -> bool:
        return self._http.health()

    def segment_text(
        self,
        observation: CameraObservation,
        query: str,
        *,
        model_query: str | None = None,
    ) -> SegmentationSet:
        if not isinstance(observation, CameraObservation):
            return self._failure(ErrorCode.INVALID_REQUEST, "observation must be CameraObservation")
        if not isinstance(query, str) or not query.strip():
            return self._failure(ErrorCode.INVALID_REQUEST, "query must be a non-empty string")
        original_query = query
        if model_query is not None and (not isinstance(model_query, str) or not model_query.strip()):
            return self._failure(ErrorCode.INVALID_REQUEST, "model_query must be a non-empty string")
        query = (model_query if model_query is not None else query).strip()
        diagnostics = {"provider": "sam3", "query": original_query,
                       "original_query": original_query, "model_query": query,
                       "prompt_type": "text", "score_semantics": TEXT_SCORE_SEMANTICS,
                       "label_is_query": True, "client_text_confidence_threshold": self.text_threshold}
        try:
            response = self._http.post_json(
                SEGMENT_PATH,
                {"image_base64": _encode_rgb_png(observation.rgb), "text_prompt": query},
            )
            raw_results = response.get("results")
            if not isinstance(raw_results, list):
                # Preserve the invalid-data ValueError contract.
                raise ValueError("response field 'results' must be a list")

            service_diagnostics = response.get("diagnostics", {})
            if not isinstance(service_diagnostics, Mapping):
                raise ValueError("response diagnostics must be an object")
            service_threshold = service_diagnostics.get("text_confidence_threshold")
            threshold = max(self.text_threshold, text_confidence_threshold(service_threshold)
                            if service_threshold is not None else self.text_threshold)
            diagnostics.update(service_diagnostics)
            diagnostics.update(original_query=original_query, query=original_query, model_query=query,
                               provider="sam3", prompt_type="text", score_semantics=TEXT_SCORE_SEMANTICS,
                               text_confidence_threshold=threshold, threshold_comparison=">",
                               service_text_confidence_threshold=service_threshold,
                               client_text_confidence_threshold=self.text_threshold)
            diagnostics['token_length_checked'] = 'tokens_including_specials' in service_diagnostics
            if service_diagnostics.get("rejection_reason") in {"prompt_too_long", "empty_prompt"}:
                return self._failure(ErrorCode.INVALID_REQUEST,
                    "SAM3 requires a short, non-empty visual description; shorten model_query",
                    details=diagnostics)

            segmentations: list[Segmentation] = []
            for item in raw_results:
                if not isinstance(item, Mapping):
                    # Preserve the invalid-data ValueError contract.
                    raise ValueError("each result must be an object")
                shape = self._shape(item.get("shape"), dimensions=2)
                if shape != observation.rgb.shape[:2]:
                    raise ValueError(
                        f"mask shape {shape} does not match image shape {observation.rgb.shape[:2]}"
                    )
                mask = _decode_raw_array(
                    item.get("mask_base64"), dtype=np.dtype(np.uint8), shape=shape
                ).astype(bool)
                label = item.get("label", query)
                if not isinstance(label, str) or not label.strip():
                    label = query
                segmentations.append(
                    Segmentation(
                        mask=mask,
                        label=label,
                        score=float(item.get("score")),
                        camera_name=observation.frame,
                        frame=observation.frame,
                        box_xyxy=np.asarray(item.get("box"), dtype=np.float64),
                    )
                )
        except ProviderHttpError as exc:
            return SegmentationSet(ok=False, error=exc.error, diagnostics=diagnostics)
        except (TypeError, ValueError, KeyError) as exc:
            return self._failure(
                ErrorCode.PERCEPTION_FAILED,
                "SAM3 returned an invalid text-segmentation response",
                details=dict(diagnostics, error=str(exc)),
            )

        diagnostics["returned_scores"] = [s.score for s in segmentations]
        diagnostics.setdefault("raw_scores", diagnostics["returned_scores"])
        diagnostics["rejected_candidates"] = [
            {"score": s.score, "reason": "below_text_threshold" if s.score <= threshold else "empty_mask"}
            for s in segmentations if s.score <= threshold or not np.any(s.mask)]
        segmentations = [s for s in segmentations if s.score > threshold and np.any(s.mask)]
        if not segmentations:
            diagnostics["rejection_reason"] = "no_trusted_candidates"
            return self._failure(
                ErrorCode.NO_SEGMENTATION,
                "SAM3 found no trusted text candidate; revise the short description or select a point from the image",
                details=diagnostics,
            )
        segmentations.sort(key=lambda item: item.score, reverse=True)
        segmentations = segmentations[:MAX_SEGMENTATIONS]
        return SegmentationSet(
            ok=True,
            segmentations=segmentations,
            diagnostics=dict(diagnostics, rejection_reason=None),
        )

    def segment_points(
        self,
        observation: CameraObservation,
        points_px: np.ndarray,
        *,
        point_labels: np.ndarray | None = None,
    ) -> SegmentationSet:
        if not isinstance(observation, CameraObservation):
            return self._failure(ErrorCode.INVALID_REQUEST, "observation must be CameraObservation")
        try:
            points = np.asarray(points_px, dtype=np.float64)
        except (TypeError, ValueError) as exc:
            return self._failure(
                ErrorCode.INVALID_REQUEST,
                "points_px must be a finite Nx2 array",
                details={"error": str(exc)},
            )
        if points.shape != (1, 2) or not bool(np.all(np.isfinite(points))):
            return self._failure(
                ErrorCode.UNSUPPORTED,
                "the SAM3 service wire format supports exactly one finite point prompt",
            )
        if point_labels is not None:
            labels = np.asarray(point_labels)
            if labels.shape != (1,) or labels[0] not in (1, True):
                return self._failure(
                    ErrorCode.UNSUPPORTED,
                    "the SAM3 service wire format supports one positive point only",
                )
        x, y = (float(points[0, 0]), float(points[0, 1]))
        height, width = observation.rgb.shape[:2]
        if not (0.0 <= x < width and 0.0 <= y < height):
            return self._failure(ErrorCode.INVALID_REQUEST, "point prompt is outside the image")

        try:
            response = self._http.post_json(
                SEGMENT_POINT_PATH,
                {"image_base64": _encode_rgb_png(observation.rgb), "point_coords": [x, y]},
            )
            scores_raw = response.get("scores")
            if not isinstance(scores_raw, list):
                # Preserve the invalid-data ValueError contract.
                raise ValueError("response field 'scores' must be a list")
            scores = np.asarray(scores_raw, dtype=np.float64)
            if scores.ndim != 1 or not bool(np.all(np.isfinite(scores))):
                raise ValueError("scores must be a finite vector")
            if len(scores) == 0:
                return self._failure(ErrorCode.NO_SEGMENTATION,
                    "SAM3 found no segmentation for the point prompt",
                    details={"provider": "sam3", "prompt_type": "point", "point_px": (x, y),
                             "score_semantics": POINT_SCORE_SEMANTICS, "raw_scores": []})
            shape = self._shape(response.get("masks_shape"), dimensions=3)
            if shape[0] != len(scores) or shape[1:] != observation.rgb.shape[:2]:
                raise ValueError("point-mask shape does not match scores and image")
            dtype_name = response.get("masks_dtype", "float32")
            if dtype_name not in {"bool", "uint8", "float32", "float64"}:
                raise ValueError(f"unsupported masks_dtype {dtype_name!r}")
            masks = _decode_raw_array(
                response.get("masks_base64"), dtype=np.dtype(dtype_name), shape=shape
            )
            segmentations = [
                Segmentation(
                    mask=mask > 0,
                    label=f"point({x:g}, {y:g})",
                    score=float(score),
                    camera_name=observation.frame,
                    frame=observation.frame,
                )
                for mask, score in zip(masks, scores, strict=False)
                if np.any(mask > 0)
            ]
        except ProviderHttpError as exc:
            return SegmentationSet(ok=False, error=exc.error)
        except (TypeError, ValueError, KeyError) as exc:
            return self._failure(
                ErrorCode.PERCEPTION_FAILED,
                "SAM3 returned an invalid point-segmentation response",
                details={"provider": "sam3", "error": str(exc)},
            )

        if not segmentations:
            return self._failure(
                ErrorCode.NO_SEGMENTATION,
                "SAM3 found no segmentation for the point prompt",
                details={"provider": "sam3", "point_px": (x, y)},
            )
        segmentations.sort(key=lambda item: item.score, reverse=True)
        segmentations = segmentations[:MAX_SEGMENTATIONS]
        return SegmentationSet(
            ok=True,
            segmentations=segmentations,
            diagnostics={"provider": "sam3", "point_px": (x, y), "prompt_type": "point",
                         "score_semantics": POINT_SCORE_SEMANTICS, "raw_scores": scores.tolist(),
                         "text_confidence_threshold_applied": False,
                         "selection_requires_visual_confirmation": True},
        )

    @staticmethod
    def _shape(value: object, *, dimensions: int) -> tuple[int, ...]:
        if not isinstance(value, list | tuple) or len(value) != dimensions:
            raise ValueError(f"shape must contain {dimensions} dimensions")
        if any(isinstance(item, bool) or not isinstance(item, int) or item < 0 for item in value):
            raise ValueError("shape dimensions must be non-negative integers")
        return tuple(value)

    @staticmethod
    def _failure(
        code: ErrorCode,
        message: str,
        *,
        details: Mapping[str, object] | None = None,
    ) -> SegmentationSet:
        return SegmentationSet(
            ok=False,
            error=ApiError(code=code, message=message, details=details or {}),
            diagnostics=details or {},
        )


SAM3Provider = Sam3Provider
Sam3Client = Sam3Provider

__all__ = ["DEFAULT_SAM3_URL", "SAM3Provider", "Sam3Client", "Sam3Provider"]
