"""Lossless loading of public frozen RGB-D evidence; no object annotations."""
import json
from pathlib import Path
import numpy as np
from .contracts import Pose, RobotState, Observation, CameraObservation


def load_observation(directory):
    directory=Path(directory)
    metadata=json.loads((directory/'metadata.json').read_text())
    raw=metadata['robot_state'];raw['end_effector_poses']={k:Pose(**v) for k,v in raw['end_effector_poses'].items()}
    state=RobotState(**raw)
    cameras={}
    for index,(name,info) in enumerate(metadata['cameras'].items()):
        with np.load(directory/f'camera-{index}.npz',allow_pickle=False) as data:
            cameras[name]=CameraObservation(data['rgb'],data['depth_m'],data['intrinsics'],info['frame'],Pose(**info['camera_pose']),info['timestamp_s'])
    return Observation(cameras,state,timestamp_s=metadata['timestamp_s'])
