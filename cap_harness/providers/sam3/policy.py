"""Text-only SAM3 acceptance policy, shared by service and client.

The pinned processor uses a strict greater-than comparison. Point prompt
scores estimate mask quality and must not be interpreted as text confidence.
"""
import math

DEFAULT_TEXT_CONFIDENCE_THRESHOLD = 0.5
TEXT_SCORE_SEMANTICS = "text_grounding_probability_times_presence"
POINT_SCORE_SEMANTICS = "point_mask_quality_not_text_confidence"


def text_confidence_threshold(value):
    if isinstance(value, bool):
        raise ValueError("SAM3 text confidence threshold must be finite and 0 < t < 1")
    threshold = float(value)
    if not math.isfinite(threshold) or not 0 < threshold < 1:
        raise ValueError("SAM3 text confidence threshold must be finite and 0 < t < 1")
    return threshold


def prompt_diagnostics(encoder, prompt):
    """Count with the actual model tokenizer before its silent truncation."""
    count = len(encoder.tokenizer.encode(prompt)) + 2
    limit = int(encoder.context_length)
    return {"model_query": prompt, "tokens_including_specials": count,
            "context_length": limit, "would_truncate": count > limit,
            "truncated": False}
