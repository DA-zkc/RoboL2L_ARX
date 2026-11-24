"""Factory-aligned Orbbec RGB-D streams for the dual X5."""
import threading
import time

import cv2
import numpy as np

from ..cameras import Frame


class OrbbecBus:
    def __init__(self):
        self.context = None
        self.lock = threading.Lock()

    def pipeline(self, serial):
        import pyorbbecsdk as ob
        with self.lock:
            if self.context is None:
                self.context = ob.Context()
                self.context.set_uvc_backend_type(ob.OBUvcBackendType.V4L2)
            device = self.context.query_devices().get_device_by_serial_number(serial)
            return ob.Pipeline(device)


class OrbbecSource:
    def __init__(self, serial, bus, *, trace_directory=None):
        self.serial, self.bus = serial, bus
        self.pipe = None
        self.stop_requested = threading.Event()
        from ..depth_capture import DepthCapture
        self.depth_capture = None if trace_directory is None else DepthCapture(trace_directory, serial)

    def open(self):
        import pyorbbecsdk as ob
        self.stop_requested.clear()
        pipe = self.bus.pipeline(self.serial)
        colors = pipe.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
        depths = pipe.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
        config = ob.Config()
        for fps in (10, 15, 5, 6, 30):
            for dw, dh in ((320, 240), (640, 480), (320, 200), (640, 400)):
                try:
                    color = colors.get_video_stream_profile(640, 480, ob.OBFormat.MJPG, fps)
                    depth = depths.get_video_stream_profile(dw, dh, ob.OBFormat.Y16, fps)
                    break
                except ob.OBError:
                    continue
            else:
                continue
            break
        else:
            raise RuntimeError(f'{self.serial}: no compatible RGB-D profiles')
        self.k = color.get_intrinsic()
        self.distortion = color.get_distortion()
        config.enable_stream(color)
        config.enable_stream(depth)
        config.set_frame_aggregate_output_mode(ob.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
        self.align = ob.AlignFilter(ob.OBStreamType.COLOR_STREAM)
        self.align.set_match_target_resolution(True)
        # Align into the ideal color camera. rectify() must then undistort RGB
        # only: this aligned depth already has zero lens distortion.
        self.align.set_config_value('TargetDistortion', 0)
        if self.align.get_config_value('TargetDistortion') != 0:
            raise ValueError('unexpected_alignment_distortion')
        pipe.enable_frame_sync()
        pipe.start(config)
        self.pipe = pipe

    def read(self):
        while not self.stop_requested.is_set():
            pair = self.pipe.wait_for_frames(500)
            if pair is None:
                continue
            aligned = self.align.process(pair)
            if aligned is None:
                continue
            frames = aligned.as_frame_set()
            color, depth = frames.get_color_frame(), frames.get_depth_frame()
            if color is None or depth is None:
                continue
            ct, dt = color.get_timestamp_us()/1000, depth.get_timestamp_us()/1000
            image = cv2.imdecode(np.frombuffer(color.get_data(), np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                continue
            raw = np.frombuffer(depth.get_data(), np.uint16).reshape(depth.get_height(), depth.get_width())
            k, d = self.k, self.distortion
            if raw.shape != image.shape[:2] or raw.shape != (k.height, k.width):
                raise ValueError('unaligned_rgbd_dimensions')
            scale = depth.get_depth_scale() / 1000.
            if not np.isfinite(scale) or scale <= 0:
                raise ValueError('invalid_depth_scale')
            coeffs = [d.k1, d.k2, d.p1, d.p2, d.k3, d.k4, d.k5, d.k6]
            profile = depth.get_stream_profile().as_video_stream_profile()
            dk, dd = profile.get_intrinsic(), profile.get_distortion()
            if not np.allclose([dk.fx, dk.fy, dk.cx, dk.cy], [k.fx, k.fy, k.cx, k.cy], atol=1e-4, rtol=0):
                raise ValueError('aligned_depth_intrinsics_mismatch')
            if any(getattr(dd, field) != 0 for field in ('k1','k2','p1','p2','k3','k4','k5','k6')):
                raise ValueError('aligned_depth_not_undistorted')
            meta = dict(serial=self.serial, width=k.width, height=k.height, fx=k.fx, fy=k.fy,
                        ppx=k.cx, ppy=k.cy, coeffs=coeffs, model='distortion.brown_conrady'
                        if d.model.name in ('BROWN_CONRADY', 'BROWN_CONRADY_K6', 'MODIFIED_BROWN_CONRADY') else d.model.name, alignment='depth_to_color',
                        frame_id=color.get_index(), color_timestamp_ms=ct, depth_timestamp_ms=dt,
                        depth_scale_m=scale, captured_s=color.get_system_timestamp_us()/1e6)
            meta.update(depth_pixel_geometry='undistorted_color', alignment_target_distortion=False,
                        depth_frame_id=depth.get_index(), sdk_depth_intrinsics=dict(
                            fx=dk.fx, fy=dk.fy, cx=dk.cx, cy=dk.cy, width=dk.width, height=dk.height))
            meta['depth_capture'] = dict(supported=self.depth_capture is not None,
                request_file=None if self.depth_capture is None else str(self.depth_capture.directory/'request.json'))
            frame = Frame(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), raw.astype(np.float32)*scale, meta, time.monotonic())
            if self.depth_capture is not None and self.depth_capture.wanted():
                self.depth_capture.record(pair, frames, frame, self.align)
            return frame
        raise RuntimeError(f'{self.serial}: camera_closed')

    def request_stop(self):
        self.stop_requested.set()

    def close(self):
        if self.pipe is not None:
            self.pipe.stop()
            self.pipe = None
