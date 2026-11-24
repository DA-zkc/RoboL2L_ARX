"""Recorded physical observations, with software-only FR3 planning."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from .fr3 import FR3Adapter
from .panda import PandaDriver
from ..observations import load_observation


class RecordedDriver(PandaDriver):
    def __init__(self, raw):
        super().__init__('recorded-no-network')
        self.raw=copy.deepcopy(raw)

    def read(self):return copy.deepcopy(self.raw)
    def connect(self):raise RuntimeError('recorded_adapter_has_no_hardware_connection')


class ReplayAdapter(FR3Adapter):
    synthetic=False
    read_only=True
    def __init__(self, observation_directory, raw_state, station_config, output_directory):
        self.recorded=load_observation(observation_directory)
        state=self.recorded.robot_state
        if not np.allclose(state.joint_positions['primary'],raw_state['q'],atol=1e-8,rtol=0):
            raise ValueError('raw_state_does_not_match_observation')
        if abs(state.timestamp_s-raw_state['timestamp_s'])>1e-4:
            raise ValueError('raw_state_timestamp_does_not_match_observation')
        sdk=SimpleNamespace(cfg=station_config,run_dir=Path(output_directory),close=lambda:None)
        sdk.run_dir.mkdir(parents=True,exist_ok=True)
        super().__init__(sdk,cameras={name:None for name in self.recorded.cameras},driver=RecordedDriver(raw_state))
        metadata=json.loads((Path(observation_directory)/'metadata.json').read_text())
        if metadata.get('device_binding') and metadata['device_binding']!=self.device_binding():
            raise ValueError('recorded_observation_configuration_mismatch')
        if metadata.get('device_binding'):
            self.calibration_provenance={'kind':'archived_binding','binding':metadata['device_binding']}
        else:
            self.calibration_provenance=match_raw_calibrations(self.recorded,raw_state,station_config)

    def get_observation(self,camera_names=None):return self.recorded

    def describe(self):
        return dict(super().describe(),evidence='recorded_physical_rgbd',read_only=True,physical_motion_enabled=False,
                    calibration_provenance=self.calibration_provenance)

    def close(self):self.driver.close()


def matching_raw_state(observation, event_root):
    for path in Path(event_root).glob('*/events.json'):
        try:events=json.loads(path.read_text())
        except (OSError,ValueError):continue
        for event in reversed(events):
            value=event.get('result',{})
            if isinstance(value,dict) and abs(value.get('timestamp_s',-1)-observation.robot_state.timestamp_s)<1e-5 and 'q_d' in value:
                return value
    raise ValueError('matching measured raw state was not found; do not synthesize commanded state')


def match_raw_calibrations(observation, raw_state, config, event_root=None):
    """Migrate old observations by matching their actual archived sensor data.

    A nearby timestamp is only a search hint. RGB, depth, intrinsics, calibration
    hash and camera transform must all match before accepting provenance.
    """
    from ..cameras import Frame, rectify
    from franka_cap.devices import calibration
    root=Path(event_root) if event_root else Path(__file__).resolve().parents[2]/'runs/cap'
    matches={}
    for name,camera in observation.cameras.items():
        _,transform,sha=calibration(config,name,raw_state)
        # Quaternion reconstruction orthogonalizes float32 robot rotations.
        if not np.allclose(transform[:3,:3],camera.camera_pose.as_matrix()[:3,:3],atol=1e-6,rtol=0) or not np.allclose(transform[:3,3],camera.camera_pose.position,atol=1e-9,rtol=0):
            raise ValueError('recorded_camera_transform_mismatch')
        for path in root.glob(f'*/raw-{name}-*/metadata.json'):
            try:stamp=int(path.parent.name.rsplit('-',1)[1])/1e9
            except ValueError:continue
            if abs(stamp-camera.timestamp_s)>.05:continue
            metadata=json.loads(path.read_text())
            if metadata.get('calibration_sha256')!=sha:continue
            with np.load(path.parent/'frame.npz',allow_pickle=False) as raw:
                rgb,depth,intrinsics=rectify(Frame(raw['rgb'],raw['depth_m'],metadata,0.))
            if np.array_equal(rgb,camera.rgb) and np.array_equal(depth,camera.depth_m) and np.allclose(intrinsics,camera.intrinsics,atol=1e-9,rtol=0):
                matches[name]={'raw_metadata':str(path.resolve()),'calibration_sha256':sha}
                break
        if name not in matches:raise ValueError('missing_or_mismatched_recorded_calibration: '+name)
    return {'kind':'matched_archived_raw_frames','cameras':matches}
