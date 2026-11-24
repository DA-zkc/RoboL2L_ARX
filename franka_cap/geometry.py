from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation


def vector(value, length, name="vector"):
    raw = np.asarray(value)
    if raw.dtype.kind not in "iuf" or raw.shape != (length,):
        raise ValueError(f"{name} must contain {length} finite numbers")
    if any(isinstance(v, (bool, np.bool_)) for v in value):
        raise ValueError(f"{name} must not contain booleans")
    result = raw.astype(float)
    if not np.isfinite(result).all():
        raise ValueError(f"{name} must be finite")
    return result


def matrix(value):
    raw = np.asarray(value)
    if raw.dtype.kind not in "iuf" or raw.shape != (4, 4):
        raise ValueError("Transform must be a finite 4x4 numeric matrix")
    result = raw.astype(float)
    if not np.isfinite(result).all() or not np.allclose(result[3], [0, 0, 0, 1], atol=1e-7, rtol=0):
        raise ValueError("Invalid homogeneous transform")
    r = result[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-5, rtol=0) or abs(np.linalg.det(r)-1) > 1e-5:
        raise ValueError("Invalid rotation")
    return result


def pose_matrix(pose):
    if not isinstance(pose, dict) or set(pose) != {"position", "quaternion_wxyz", "frame"} or pose["frame"] != "base":
        raise ValueError("Pose requires position, quaternion_wxyz and frame='base'; metres and configured EE")
    pos = vector(pose["position"], 3, "position")
    q = vector(pose["quaternion_wxyz"], 4, "quaternion_wxyz")
    if abs(np.linalg.norm(q)-1) > 1e-5:
        raise ValueError("Quaternion must be normalized, wxyz order")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    result[:3, 3] = pos
    return result


def pose_dict(value):
    value = matrix(value)
    q = Rotation.from_matrix(value[:3, :3]).as_quat()
    return {"position": value[:3, 3].tolist(), "quaternion_wxyz": q[[3, 0, 1, 2]].tolist(), "frame": "base"}


def shifted(pose, delta_base_m):
    value = pose_matrix(pose)
    value[:3, 3] += vector(delta_base_m, 3, "delta_base_m")
    return pose_dict(value)
