"""Bounded, opt-in evidence capture inside the existing camera reader.

No device discovery, stream changes, robot calls, or arbitrary code/paths from
requests. The writer owns copied arrays only, never native SDK frame handles.
"""
import json
from pathlib import Path
import queue
import re
import threading

import numpy as np


def video_metadata(frame):
    profile = frame.get_stream_profile().as_video_stream_profile()
    k, d = profile.get_intrinsic(), profile.get_distortion()
    return dict(width=frame.get_width(), height=frame.get_height(),
                frame_id=frame.get_index(), timestamp_us=frame.get_timestamp_us(),
                system_timestamp_us=frame.get_system_timestamp_us(), format=str(frame.get_format()),
                intrinsics={name: getattr(k, name) for name in ('width','height','fx','fy','cx','cy')},
                distortion=dict(model=str(d.model), **{name: getattr(d, name)
                    for name in ('k1','k2','p1','p2','k3','k4','k5','k6')}))


class DepthCapture:
    """Request file: {id: 32 lowercase hex digits, serial: str, frames: 1..8}."""
    def __init__(self, directory, serial):
        self.directory = Path(directory) / serial
        self.serial = serial
        self.request = None
        self.queued = 0
        self.queue = queue.Queue(maxsize=2)
        self.thread = None

    def wanted(self):
        try:
            path = self.directory/'request.json'
            if not path.is_file():
                return False
            if path.stat().st_size > 1024:
                return False
            value = json.loads(path.read_text())
            if (set(value) != {'id','serial','frames'} or value['serial'] != self.serial
                    or not isinstance(value['id'], str) or not re.fullmatch('[0-9a-f]{32}', value['id'])
                    or type(value['frames']) is not int or not 1 <= value['frames'] <= 8):
                return False
            if self.request is None or self.request['id'] != value['id']:
                self.request, self.queued = value, 0
            return self.queued < self.request['frames'] and not self.queue.full()
        except (OSError, ValueError, TypeError):
            return False

    def record(self, pair, aligned, frame, align):
        """Called only after wanted(); exceptions cannot interrupt acquisition."""
        try:
            from .cameras import rectify
            import pyorbbecsdk as ob
            native_depth, native_color = pair.get_depth_frame(), pair.get_color_frame()
            target_depth = aligned.get_depth_frame()
            native_profile = native_depth.get_stream_profile()
            extrinsic = native_profile.get_extrinsic_to(native_color.get_stream_profile())
            rgb, depth, k = rectify(frame)
            arrays = dict(
                native_depth_u16=np.frombuffer(native_depth.get_data(), np.uint16).reshape(
                    native_depth.get_height(), native_depth.get_width()).copy(),
                native_color_encoded=np.frombuffer(native_color.get_data(), np.uint8).copy(),
                aligned_depth_u16=np.frombuffer(target_depth.get_data(), np.uint16).reshape(
                    target_depth.get_height(), target_depth.get_width()).copy(),
                rgb_before_rectification=frame.rgb.copy(), aligned_depth_m=frame.depth_m.copy(),
                rgb_rectified=rgb, depth_rectified_m=depth, intrinsics=k,
            )
            metadata = dict(schema=1, serial=self.serial, request_id=self.request['id'], index=self.queued,
                sdk_version=ob.get_version(), binding_version=ob.__version__,
                source='same SDK FrameSet before/after AlignFilter and application rectification',
                native_depth=video_metadata(native_depth), native_color=video_metadata(native_color),
                aligned_depth=video_metadata(target_depth), application=frame.metadata,
                native_depth_scale_m=native_depth.get_depth_scale()/1000.,
                aligned_depth_scale_m=target_depth.get_depth_scale()/1000.,
                depth_to_color_extrinsic=dict(rotation=np.asarray(extrinsic.rot).tolist(),
                                             translation_mm=np.asarray(extrinsic.transform).tolist()),
                alignment={name: align.get_config_value(name)
                           for name in ('AlignType','TargetDistortion','GapFillCopy','MatchTargetRes')},
                limits='Native SDK depth is after sensor/firmware processing; no raw disparity or IR streams.')
            self.enqueue(arrays, metadata)
        except Exception as exc:
            self._error(exc)
            # Do not repeatedly copy a failing request at camera frame rate.
            if self.request:
                self.queued = self.request['frames']

    def enqueue(self, arrays, metadata):
        if self.thread is None:
            self.thread = threading.Thread(target=self._write, daemon=True, name='depth-evidence-writer')
            self.thread.start()
        self.queue.put_nowait((arrays, metadata))
        self.queued += 1

    def _error(self, error):
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            (self.directory/'error.json').write_text(json.dumps(
                dict(request=self.request, error=str(error)), ensure_ascii=False))
        except OSError:
            pass

    def _write(self):
        while True:
            arrays, metadata = self.queue.get()
            try:
                root = self.directory/metadata['request_id']/str(metadata['index'])
                root.mkdir(parents=True, exist_ok=False)
                np.savez_compressed(root/'stages.npz', **arrays)
                # Written last: consumers accept only completed records.
                (root/'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
            except Exception as exc:
                self._error(exc)
            finally:
                self.queue.task_done()
