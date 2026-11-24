"""Vendor SDK implementations behind the persistent camera owner."""
import json
import subprocess
import time
import select
import os
import tempfile
import numpy as np
import cv2
from ..cameras import Frame

class RealSenseSource:
    def __init__(self,serial): self.serial=serial;self.pipe=None
    def open(self):
        from franka_cap.devices import _realsense
        self.rs=rs=_realsense();self.pipe=rs.pipeline();cfg=rs.config();cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.color,640,480,rs.format.rgb8,30);cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
        self.profile=self.pipe.start(cfg);self.align=rs.align(rs.stream.color)
    def read(self):
        # The first RSUSB pair can straddle clock synchronization. Discard bad
        # pairs, never expose them or relax the 50 ms observation requirement.
        for _ in range(15):
            frames=self.align.process(self.pipe.wait_for_frames(1000))
            color=frames.get_color_frame();depth=frames.get_depth_frame()
            if color and depth and color.get_frame_timestamp_domain()==depth.get_frame_timestamp_domain() and abs(color.timestamp-depth.timestamp)<=50:
                break
        else:raise RuntimeError('rgbd_timestamp_skew_or_missing_frames')
        k=color.profile.as_video_stream_profile().intrinsics
        scale=self.profile.get_device().first_depth_sensor().get_depth_scale()
        meta={key:getattr(k,key) for key in ('width','height','fx','fy','ppx','ppy')}
        meta.update(serial=self.profile.get_device().get_info(self.rs.camera_info.serial_number),coeffs=list(k.coeffs),model=str(k.model),
                    alignment='depth_to_color',color_timestamp_ms=color.timestamp,depth_timestamp_ms=depth.timestamp,depth_scale_m=scale,frame_id=color.frame_number,depth_frame_id=depth.frame_number,timestamp_domain=str(color.get_frame_timestamp_domain()))
        return Frame(np.asanyarray(color.get_data()).copy(),np.asanyarray(depth.get_data()).astype(float)*scale,meta,time.monotonic())
    def close(self):
        if self.pipe:self.pipe.stop();self.pipe=None

class GeminiSource:
    def __init__(self,serial):self.serial=serial;self.process=None;self.startup=True;self.errors=None
    def open(self):
        from franka_cap.common import ROOT
        self.errors=tempfile.TemporaryFile()
        self.process=subprocess.Popen([str(ROOT/'native/build/franka_cap_global'),self.serial,'--stream'],stdout=subprocess.PIPE,stderr=self.errors,bufsize=0)
    def _read(self,count):
        data=bytearray();deadline=time.monotonic()+(15 if self.startup else 5)
        while len(data)<count:
            remaining=deadline-time.monotonic()
            if remaining<=0 or not select.select([self.process.stdout],[],[],remaining)[0]:raise TimeoutError('gemini_stream_timeout')
            part=os.read(self.process.stdout.fileno(),count-len(data))
            if not part:
                detail=os.pread(self.errors.fileno(),2000,max(0,os.fstat(self.errors.fileno()).st_size-2000)).decode(errors='replace')
                raise RuntimeError('gemini_stream_closed: '+detail[-1000:])
            data.extend(part)
        return bytes(data)
    def read(self):
        header=bytearray()
        while not header.endswith(b'\n'):
            if len(header)>100:raise ValueError('invalid_stream_header')
            header.extend(self._read(1))
        lengths=list(map(int,header.split()))
        if len(lengths)!=3 or not all(0<n<=16_000_000 for n in lengths):raise ValueError('invalid_stream_lengths')
        self.startup=False
        meta=json.loads(self._read(lengths[0]));encoded=self._read(lengths[1]);raw=self._read(lengths[2])
        bgr=cv2.imdecode(np.frombuffer(encoded,np.uint8),cv2.IMREAD_COLOR)
        if bgr is None:raise ValueError('invalid_jpeg')
        depth=np.frombuffer(raw,np.uint16).reshape(meta['height'],meta['width']).astype(float)*meta['depth_scale_m']
        return Frame(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB),depth,meta,time.monotonic())
    def close(self):
        if self.process:
            self.process.terminate()
            try:self.process.wait(2)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
            self.process.stdout.close();self.process=None
        if self.errors:self.errors.close();self.errors=None
