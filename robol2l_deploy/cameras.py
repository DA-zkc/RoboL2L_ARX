"""Persistent RGB-D owners, bounded latest-frame cache and explicit rectification."""
from dataclasses import dataclass
import threading
import time
import cv2
import numpy as np

@dataclass(frozen=True)
class Frame:
    rgb: np.ndarray
    depth_m: np.ndarray
    metadata: dict
    monotonic_s: float


def rectify(frame):
    m=frame.metadata
    depth_geometry=m.get('depth_pixel_geometry','distorted_color')
    if depth_geometry not in ('distorted_color','undistorted_color'):
        raise ValueError('unsupported_depth_pixel_geometry')
    if frame.depth_m.shape != frame.rgb.shape[:2]:
        raise ValueError('unaligned_rgbd_dimensions')
    k=np.array([[m['fx'],0,m['ppx']],[0,m['fy'],m['ppy']],[0,0,1.]])
    coeff=np.asarray(m.get('coeffs', [0]*5))
    if not np.all(np.isfinite(coeff)): raise ValueError('invalid_distortion')
    if np.any(coeff):
        if m['model'] not in ('distortion.brown_conrady','distortion.modified_brown_conrady'):
            raise ValueError('unsupported_distortion_model')
        size=(frame.rgb.shape[1],frame.rgb.shape[0])
        x,y=cv2.initUndistortRectifyMap(k,coeff,None,k,size,cv2.CV_32FC1)
        rgb=cv2.remap(frame.rgb,x,y,cv2.INTER_LINEAR)
        # Orbbec software D2C with TargetDistortion=0 already produces depth
        # on this ideal K grid. Remapping it again shifts depth relative to RGB.
        depth=(frame.depth_m.copy() if depth_geometry=='undistorted_color'
               else cv2.remap(frame.depth_m,x,y,cv2.INTER_NEAREST))
    else: rgb=frame.rgb.copy();depth=frame.depth_m.copy()
    return rgb,depth,k

class CameraOwner:
    def __init__(self, source):
        self.source=source; self.condition=threading.Condition();self.frame=None;self.error=None
        self.stop=threading.Event();self.thread=None;self.retry_after=0.
    def start(self):
        with self.condition:
            if self.stop.is_set() or (self.thread and self.thread.is_alive()) or time.monotonic() < self.retry_after: return
            self.error = self.frame = None
            self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()
    def _run(self):
        try:
            self.source.open()
            while not self.stop.is_set():
                frame=self.source.read()
                with self.condition:
                    self.frame=frame;self.error=None;self.condition.notify_all()
        except Exception as error:
            with self.condition:
                self.frame=None;self.error=str(error);self.retry_after=time.monotonic()+2.;self.condition.notify_all()
        finally: self.source.close()
    def latest(self, *, after=0, timeout=None):
        self.start(); deadline=None if timeout is None else time.monotonic()+timeout
        with self.condition:
            while True:
                if self.error: raise RuntimeError(self.error)
                if self.stop.is_set():raise RuntimeError('camera_closed')
                if self.frame and self.frame.monotonic_s>after: return self.frame
                remaining=None if deadline is None else deadline-time.monotonic()
                if remaining is not None and remaining<=0: raise TimeoutError('camera_frame_unavailable')
                self.condition.wait(remaining)
    def close(self):
        self.stop.set()
        with self.condition:self.condition.notify_all()
        request_stop = getattr(self.source, 'request_stop', None)
        if request_stop is not None: request_stop()
        if self.thread: self.thread.join()
