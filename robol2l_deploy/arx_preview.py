"""Local, uncalibrated colour previews for the two-arm operator panel."""
from collections import Counter
import fcntl
import json
from pathlib import Path
import re
import subprocess
import threading
import time

import cv2

from .cap.storage import write_json

ROLES = ('global', 'left_wrist', 'right_wrist')
COLOR_FORMAT = re.compile(r"'(?:MJPG|JPEG|YUYV|UYVY|NV12|RGB3|BGR3)'")


def video_nodes():
    nodes = []
    for entry in sorted(Path('/sys/class/video4linux').glob('video*')):
        device = Path('/dev') / entry.name
        try:
            if not device.exists() or (entry / 'index').read_text().strip() != '0':
                continue
            interface = (entry / 'device').resolve()
            usb = interface.parent
            serial = (usb / 'serial').read_text().strip() if (usb / 'serial').exists() else ''
            number = (interface / 'bInterfaceNumber').read_text().strip()
            label = (usb / 'product').read_text().strip()
            port = usb.name
            nodes.append((str(device), serial, number, port, label))
        except OSError:
            continue
    return tuple(nodes)


def color_devices(nodes):
    devices = {}
    counts = Counter((serial, interface) for _, serial, interface, _, _ in nodes)
    for path, serial, interface, port, label in nodes:
        try:
            formats = subprocess.run(['v4l2-ctl', '-d', path, '--list-formats'], timeout=2,
                                     text=True, capture_output=True, check=True).stdout
        except (OSError, subprocess.SubprocessError):
            continue
        if COLOR_FORMAT.search(formats):
            identity = 'serial/' + serial if serial and counts[serial, interface] == 1 else 'usb/' + port
            key = identity + '/' + interface
            devices[key] = dict(path=path, label=f'{label} · {serial or port} · {interface}')
    return devices


class ColorFeed:
    """One capture loop per device, shared by viewers and released after idling."""
    def __init__(self, path):
        self.path = path
        self.condition = threading.Condition()
        self.worker = None
        self.last_request = 0.
        self.frame = None
        self.stamp = 0.
        self.error = None
        self.retry_at = 0.
        self.closed = False

    def jpeg(self):
        with self.condition:
            now = self.last_request = time.monotonic()
            if self.closed or (self.error and now < self.retry_at):
                raise RuntimeError(self.error or 'preview_closed')
            if self.worker is None:
                self.error = self.frame = None
                self.worker = threading.Thread(target=self._capture, daemon=True)
                self.worker.start()
            ready = self.condition.wait_for(lambda: self.closed or self.error or
                    (self.frame is not None and time.monotonic() - self.stamp < 1.), timeout=3.)
            if not ready or self.closed or self.error:
                raise RuntimeError(self.error or 'preview_frame_unavailable')
            return self.frame

    def _capture(self):
        camera = None
        error = None
        try:
            camera = cv2.VideoCapture(self.path, cv2.CAP_V4L2)
            if not camera.isOpened():
                raise RuntimeError('preview_device_unavailable')
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            while not self.closed and time.monotonic() - self.last_request < 3.:
                ok, image = camera.read()
                if not ok:
                    raise RuntimeError('preview_capture_failed')
                if time.monotonic() - self.stamp < .09:
                    continue
                ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 82])
                if not ok:
                    raise RuntimeError('preview_encode_failed')
                with self.condition:
                    self.frame, self.stamp = encoded.tobytes(), time.monotonic()
                    self.condition.notify_all()
        except (OSError, RuntimeError, cv2.error) as exc:
            error = str(exc)
        finally:
            if camera is not None:
                camera.release()
            with self.condition:
                self.frame = self.worker = None
                self.error, self.retry_at = error, time.monotonic() + 2.
                self.condition.notify_all()

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()


class CameraPreviews:
    def __init__(self, defaults, settings):
        self.defaults = {role: defaults.get(role) for role in ROLES}
        self.settings = Path(settings)
        self.lock = threading.RLock()
        self.devices, self.feeds = {}, {}
        self.nodes = None
        self.next_scan = 0.

    def _bindings(self):
        try:
            return {**self.defaults, **json.loads(self.settings.read_text())}
        except FileNotFoundError:
            return self.defaults.copy()

    def _scan(self):
        if time.monotonic() < self.next_scan:
            return
        nodes = video_nodes()
        if nodes != self.nodes:
            devices = color_devices(nodes)
            for key in list(self.feeds):
                if devices.get(key) != self.devices.get(key):
                    self.feeds.pop(key).close()
            self.nodes, self.devices = nodes, devices
        self.next_scan = time.monotonic() + 2.

    def describe(self):
        with self.lock:
            self._scan()
            return dict(roles=self._bindings(), devices={k: v['label'] for k, v in self.devices.items()})

    def bind(self, role, source):
        if role not in ROLES:
            raise ValueError('unknown_preview_role')
        with self.lock:
            self._scan()
            if source is not None and (not isinstance(source, str) or source not in self.devices):
                raise ValueError('preview_device_unavailable')
            self.settings.parent.mkdir(parents=True, exist_ok=True)
            with self.settings.with_suffix('.lock').open('a') as file:
                fcntl.flock(file, fcntl.LOCK_EX)
                roles = self._bindings()
                if source and any(k != role and v == source for k, v in roles.items()):
                    raise ValueError('preview_device_already_assigned')
                roles[role] = source
                write_json(self.settings, roles)
            return self.describe()

    def jpeg(self, role, source):
        with self.lock:
            self._scan()
            if role not in ROLES or not source or self._bindings()[role] != source or source not in self.devices:
                raise ValueError('preview_device_unavailable')
            if source not in self.feeds:
                self.feeds[source] = ColorFeed(self.devices[source]['path'])
            feed = self.feeds[source]
        return feed.jpeg()

    def close(self):
        with self.lock:
            for feed in self.feeds.values():
                feed.close()
            self.feeds.clear()


class RGBDPreviews(CameraPreviews):
    """Share the serial-bound RGB-D owners with observations and recorders."""
    def __init__(self, serials, settings):
        from .adapters.orbbec_pair import OrbbecBus, OrbbecSource
        from .cameras import CameraOwner
        if set(serials) != set(ROLES) or len(set(serials.values())) != len(ROLES):
            raise ValueError('three_unique_arx_camera_serials_required')
        defaults = {role: 'serial/' + serial + '/04' for role, serial in serials.items()}
        super().__init__(defaults, settings)
        bus = OrbbecBus()
        self.owners = {defaults[role]: CameraOwner(OrbbecSource(serial, bus,
                          trace_directory=Path(settings).parent/'rgbd-depth-traces'))
                       for role, serial in serials.items()}

    def _scan(self):
        self.devices = {key: dict(label=owner.source.serial) for key, owner in self.owners.items()}

    def cameras(self):
        with self.lock:
            return {role: self.owners[key] for role, key in self._bindings().items() if key in self.owners}

    def jpeg(self, role, source):
        with self.lock:
            if source not in self.owners or self._bindings().get(role) != source:
                raise ValueError('preview_device_unavailable')
            owner = self.owners[source]
        frame = owner.latest(after=time.monotonic()-1, timeout=12.)
        ok, data = cv2.imencode('.jpg', cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR))
        if not ok:
            raise RuntimeError('preview_encode_failed')
        return data.tobytes()

    def close(self):
        for owner in self.owners.values():
            owner.close()
