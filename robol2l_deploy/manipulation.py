"""Device-neutral observed geometry and payload bookkeeping, without task choreography."""
from dataclasses import dataclass
import itertools
import numpy as np
from .contracts import Pose


@dataclass(frozen=True)
class Box:
    name: str
    pose: Pose
    size_m: tuple[float, float, float]
    graspable: bool = False

    def __post_init__(self):
        size = np.asarray(self.size_m, dtype=float)
        if not self.name or self.pose.frame != 'base' or size.shape != (3,) or not np.isfinite(size).all() or np.any(size < 0):
            raise ValueError('invalid_base_frame_box')
        object.__setattr__(self, 'size_m', tuple(map(float, size)))

    def corners(self, pose=None):
        transform = self.pose.as_matrix() if pose is None else pose
        local = np.array(list(itertools.product((-1, 1), repeat=3))) * np.asarray(self.size_m) / 2
        return local @ transform[:3, :3].T + transform[:3, 3]


@dataclass(frozen=True)
class Scene:
    boxes: tuple[Box, ...]
    support_point_m: tuple[float, float, float]
    support_normal: tuple[float, float, float]
    observation_id: str
    calibration_binding: str
    captured_s: float
    reviewed: bool = False

    def __post_init__(self):
        point, normal = np.asarray(self.support_point_m), np.asarray(self.support_normal)
        if point.shape != (3,) or normal.shape != (3,) or not np.isfinite([point, normal]).all() or abs(np.linalg.norm(normal)-1) > 1e-6:
            raise ValueError('invalid_support_plane')
        if not self.observation_id or not self.calibration_binding or not np.isfinite(self.captured_s):
            raise ValueError('scene_provenance_required')
        if len({box.name for box in self.boxes}) != len(self.boxes):
            raise ValueError('duplicate_scene_object')
        object.__setattr__(self, 'boxes', tuple(self.boxes))
        object.__setattr__(self, 'support_point_m', tuple(map(float, point)))
        object.__setattr__(self, 'support_normal', tuple(map(float, normal)))


@dataclass
class Manipulation:
    scene: Scene | None = None
    body: Box | None = None
    ee_to_object: np.ndarray | None = None
    grasp_width_m: float | None = None

    @property
    def holding(self):
        return self.body is not None

    def telemetry_error(self, is_grasped, width_m):
        if bool(is_grasped) != self.holding:
            return 'grasp_lost' if self.holding else 'untracked_held_object'
        return None

    def grasp_candidate(self, ee):
        if self.holding or self.scene is None:
            raise ValueError('already_holding_or_scene_missing')
        candidates = []
        for box in self.scene.boxes:
            if not box.graspable:
                continue
            # Perception registers the intended target. Its visible-point centre
            # need not coincide with the grasp TCP. Collision checks decide
            # clearance; the native gripper receipt decides whether it was held.
            corners = (box.corners()-ee[:3, 3]) @ ee[:3, :3]
            width = float(np.ptp(corners[:, 1]))
            candidates.append((box, width))
        if len(candidates) != 1:
            raise ValueError('grasp_target_missing_or_ambiguous')
        return candidates[0]

    def attach(self, box, ee, width_m):
        self.body = box
        self.ee_to_object = np.linalg.inv(ee) @ box.pose.as_matrix()
        self.grasp_width_m = float(width_m)

    def release(self, ee):
        """Keep the released body in the scene at its last predicted pose."""
        from dataclasses import replace
        from scipy.spatial.transform import Rotation
        if self.body is not None:
            pose = ee @ self.ee_to_object
            q = Rotation.from_matrix(pose[:3, :3]).as_quat()[[3, 0, 1, 2]]
            placed = replace(self.body, pose=Pose(pose[:3, 3], q, 'base'))
            self.scene = replace(self.scene, boxes=tuple(placed if b.name == placed.name else b for b in self.scene.boxes))
        self.body = self.ee_to_object = self.grasp_width_m = None
