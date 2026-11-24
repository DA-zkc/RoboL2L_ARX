"""Explicit, mask-bounded RGB-D selection; no semantic target inference."""
from collections import deque

import numpy as np


class DepthSelectionError(ValueError):
    """Recoverable empty/unsupported selection with its measured evidence."""

    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


def _distribution(values):
    if not values.size:
        return None
    return dict(zip(("min", "p01", "p05", "median", "p95", "p99", "max"),
                    map(float, np.quantile(values, [0, .01, .05, .5, .95, .99, 1]))))


def select_depth_pixels(depth, mask, *, depth_range_m=None, seed_px=None,
                        max_depth_jump_m=None):
    """Return rows/columns plus camera-Z evidence, without changing input arrays.

    With a seed, retain only its 4-connected component whose adjacent depth
    differences meet the requested limit. This is local continuity, not a
    bound on total depth span and not proof that a component is one object.
    """
    bounds = None
    if depth_range_m is not None:
        bounds = np.asarray(depth_range_m)
        if (bounds.shape != (2,) or not np.issubdtype(bounds.dtype, np.number)
                or np.iscomplexobj(bounds) or not np.all(np.isfinite(bounds))
                or not 0 <= bounds[0] < bounds[1]):
            raise ValueError("depth_range_m must be finite [min, max] with 0 <= min < max")
        bounds = bounds.astype(float)
    if (seed_px is None) != (max_depth_jump_m is None):
        raise ValueError("seed_px and max_depth_jump_m must be supplied together")
    seed = None
    if seed_px is not None:
        xy = np.asarray(seed_px)
        if (xy.shape != (2,) or not np.issubdtype(xy.dtype, np.number)
                or np.iscomplexobj(xy) or not np.all(np.isfinite(xy))
                or not np.all(xy == np.floor(xy))):
            raise ValueError("seed_px must be an integer pixel [x, y]")
        x, y = map(int, xy)
        if not (0 <= x < depth.shape[1] and 0 <= y < depth.shape[0]):
            raise ValueError("seed_px is outside the image")
        if (isinstance(max_depth_jump_m, (bool, np.bool_))
                or not isinstance(max_depth_jump_m, (int, float, np.integer, np.floating))):
            raise ValueError("max_depth_jump_m must be a finite positive number")
        jump = float(max_depth_jump_m)
        if not np.isfinite(jump) or jump <= 0:
            raise ValueError("max_depth_jump_m must be a finite positive number")
        seed = (x, y)

    valid = mask & np.isfinite(depth) & (depth > 0)
    count = int(np.count_nonzero(mask))
    valid_count = int(np.count_nonzero(valid))
    keep = valid.copy()
    if bounds is not None:
        keep &= (depth >= bounds[0]) & (depth <= bounds[1])
    in_range_count = int(np.count_nonzero(keep))
    evidence = dict(
        depth_axis="camera_z", units="m", selected_pixels=count,
        valid_positive_pixels=valid_count, invalid_depth_pixels=count-valid_count,
        input_depth_m=_distribution(depth[valid]),
        depth_range_m=None if bounds is None else bounds.tolist(),
        seed_px=None if seed is None else list(seed),
        max_depth_jump_m=None if seed is None else jump,
        connectivity=None if seed is None else 4,
        removed_by_depth_range=valid_count-in_range_count,
        removed_by_connectivity=0, retained_pixels=in_range_count,
        retained_fraction=in_range_count/count if count else 0.,
        output_depth_m=_distribution(depth[keep]),
        target_identity_verified=False, depth_accuracy_verified=False,
    )
    if seed is not None:
        x, y = seed
        if not keep[y, x]:
            evidence.update(retained_pixels=0, retained_fraction=0., output_depth_m=None,
                            rejection_reason="seed_has_no_selected_depth")
            raise DepthSelectionError("seed_px must lie inside the mask with valid in-range depth", evidence)
        connected = np.zeros_like(keep)
        connected[y, x] = True
        queue = deque([(y, x)])
        height, width = depth.shape
        while queue:
            row, col = queue.popleft()
            for nr, nc in ((row-1, col), (row+1, col), (row, col-1), (row, col+1)):
                if (0 <= nr < height and 0 <= nc < width and keep[nr, nc]
                        and not connected[nr, nc]
                        and abs(depth[nr, nc]-depth[row, col]) <= jump):
                    connected[nr, nc] = True
                    queue.append((nr, nc))
        keep = connected
        retained = int(np.count_nonzero(keep))
        evidence.update(removed_by_connectivity=in_range_count-retained,
                        retained_pixels=retained, retained_fraction=retained/count if count else 0.,
                        output_depth_m=_distribution(depth[keep]))
    if not np.any(keep):
        evidence["rejection_reason"] = "no_selected_depth"
        raise DepthSelectionError("segmentation contains no pixels with valid selected depth", evidence)
    return (*np.nonzero(keep), evidence)
