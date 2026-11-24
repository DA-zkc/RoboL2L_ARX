"""Optional RGB recording of an attempt.

Frames are sampled from the camera owners the station already runs, so
recording never opens a device, never changes capture, and never gates motion.
If a camera stalls or the encoder fails, the attempt continues and the failure
is recorded against the video instead.
"""
import threading
import time
from pathlib import Path

DEFAULT_FPS = 10
MAX_SECONDS = 20 * 60


class CameraRecorder:
    """Samples one camera into an mp4 for as long as it is running."""

    def __init__(self, owner, path, fps=DEFAULT_FPS):
        self.owner, self.path, self.fps = owner, Path(path), fps
        self.stop = threading.Event()
        self.thread = None
        self.frames = 0
        self.error = None
        self.started_s = None

    def start(self):
        self.started_s = time.time()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return self

    def _run(self):
        import cv2
        writer, interval, deadline = None, 1.0 / self.fps, time.monotonic() + MAX_SECONDS
        try:
            while not self.stop.is_set() and time.monotonic() < deadline:
                tick = time.monotonic()
                frame = self.owner.latest(timeout=2)
                if frame is None:
                    continue
                rgb = frame.rgb
                if writer is None:
                    height, width = rgb.shape[:2]
                    writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*'mp4v'),
                                             self.fps, (width, height))
                    if not writer.isOpened():
                        raise RuntimeError('video_writer_unavailable')
                writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                self.frames += 1
                remaining = interval - (time.monotonic() - tick)
                if remaining > 0:
                    self.stop.wait(remaining)
        except Exception as error:
            # Recording is evidence, not control: a failure here never stops a run.
            # Keep the message: it is our own diagnostic, not provider output.
            self.error = '%s: %s' % (type(error).__name__, error)
        finally:
            if writer is not None:
                writer.release()

    def finish(self):
        self.stop.set()
        if self.thread:
            self.thread.join(10)
        return {'file': self.path.name, 'frames': self.frames, 'fps': self.fps,
                'seconds': round(time.time() - (self.started_s or time.time()), 2),
                'error': self.error,
                'written': self.path.is_file() and self.path.stat().st_size > 0}


class AttemptRecording:
    """Starts one recorder per requested camera and collects their receipts.

    Videos sit in the experiment folder next to the before/after stills, not in
    the version folder: the executor creates that one itself and refuses to
    reuse an existing directory.
    """

    def __init__(self, runtime, folder, cameras, version, fps=DEFAULT_FPS):
        self.runtime, self.folder = runtime, Path(folder)
        self.cameras = [c for c in (cameras or []) if c in runtime.adapter.cameras]
        self.version, self.fps = version, fps
        self.recorders = {}

    def __enter__(self):
        for name in self.cameras:
            try:
                owner = self.runtime.adapter.cameras[name]
                path = self.folder / ('camera-' + self.version + '-' + name + '.mp4')
                self.recorders[name] = CameraRecorder(owner, path, self.fps).start()
            except Exception:
                continue
        return self

    def __exit__(self, *error):
        self.results = {name: recorder.finish() for name, recorder in self.recorders.items()}
        return False

    def receipts(self):
        return self.results.copy()
