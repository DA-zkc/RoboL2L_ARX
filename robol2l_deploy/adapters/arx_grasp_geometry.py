"""Measured or reproducible official-model grasp frames, separate from encoders.

No default finger length is inferred from the robot name or RGB-D estimates.
The native SDK and wrist-camera calibration always retain their link6 frame.
"""
from copy import deepcopy
from pathlib import Path

import numpy as np

from cap_harness.geometry import matrix_to_pose
from franka_cap.common import read_json
from .arx_gripper_model import official_gripper_record


class GraspGeometry:
    def __init__(self, path, ports):
        self.transforms = {}
        self.records = {}
        if path is None:
            return
        data = read_json(Path(path))
        if (not isinstance(data, dict) or data.get('schema') != 1
                or data.get('station') != 'arx-x5-2023-dual' or data.get('ports') != ports
                or not isinstance(data.get('sides'), dict) or set(data['sides']) != set(ports)):
            raise ValueError('grasp_geometry_station_binding_mismatch')
        for side, record in data['sides'].items():
            if record is None:
                continue
            model_record = isinstance(record, dict) and record.get('verification') == 'official_model'
            if model_record:
                expected = official_gripper_record()
                if any(record.get(key) != value for key, value in expected.items()):
                    raise ValueError(f'{side}: official_model_grasp_geometry_mismatch')
            transform = np.asarray(record.get('T_link6_from_grasp_tcp'), dtype=float)
            matrix_to_pose(transform, frame='link6')  # Finite, rigid, proper SE(3).
            transform = transform.copy()
            transform.setflags(write=False)
            self.transforms[side] = transform
            self.records[side] = deepcopy(record)

    def reference(self, side):
        return 'grasp_tcp' if side in self.transforms else 'link6'

    def describe(self, sides):
        return {side: dict(
            configured=side in self.transforms,
            physically_verified=self.records.get(side, {}).get('verified', False),
            pose_reference=self.reference(side),
            geometry=deepcopy(self.records.get(side)),
            missing=[] if side in self.transforms else ['sourced_T_link6_from_grasp_tcp'],
        ) for side in sides}

    def from_flange(self, side, flange):
        """T_base_from_tcp = T_base_from_link6 @ T_link6_from_tcp."""
        if side not in self.transforms:
            return flange
        return matrix_to_pose(flange.as_matrix() @ self.transforms[side], frame=flange.frame)

    def to_flange(self, side, tcp):
        """Convert a desired public TCP pose to the native SDK link6 target."""
        if side not in self.transforms:
            return tcp
        return matrix_to_pose(tcp.as_matrix() @ np.linalg.inv(self.transforms[side]), frame=tcp.frame)
