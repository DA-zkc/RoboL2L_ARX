from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import TestCase
from unittest.mock import patch
import time

import cv2
import numpy as np

from robol2l_deploy import arx_preview as preview


class FakeCapture:
    def __init__(self, path, backend):
        self.value = int(path[-1]) * 50

    def isOpened(self):
        return True

    def set(self, prop, value):
        return True

    def read(self):
        time.sleep(.01)
        return True, np.full((16, 16, 3), self.value, dtype=np.uint8)

    def release(self):
        pass


class PreviewTests(TestCase):
    def test_three_distinct_live_sources_persist_and_reconnect(self):
        devices = {key: dict(path='/dev/video' + str(i), label=key)
                   for i, key in enumerate(['global-usb', 'left-usb', 'right-usb'], 1)}
        with TemporaryDirectory() as tmp, patch.object(preview, 'video_nodes') as nodes, \
                patch.object(preview, 'color_devices', side_effect=lambda keys: {k: devices[k] for k in keys}), \
                patch.object(cv2, 'VideoCapture', side_effect=FakeCapture):
            nodes.return_value = tuple(devices)
            path = Path(tmp) / 'sources.json'
            manager = preview.CameraPreviews({'global': 'global-usb'}, path)
            self.addCleanup(manager.close)
            manager.bind('right_wrist', 'right-usb')
            bindings = manager.bind('left_wrist', 'left-usb')['roles']
            self.assertEqual(preview.CameraPreviews({}, path).describe()['roles'], bindings)
            with self.assertRaisesRegex(ValueError, 'already_assigned'):
                manager.bind('left_wrist', 'global-usb')
            with ThreadPoolExecutor(3) as workers:
                frames = list(workers.map(manager.jpeg, preview.ROLES, map(bindings.get, preview.ROLES)))
            for expected, frame in zip([50, 100, 150], frames):
                decoded = cv2.imdecode(np.frombuffer(frame, np.uint8), cv2.IMREAD_COLOR)
                self.assertEqual(decoded[0, 0].tolist(), [expected] * 3)
            nodes.return_value = ('global-usb', 'right-usb')
            manager.next_scan = 0
            self.assertNotIn('left-usb', manager.describe()['devices'])
            with self.assertRaisesRegex(ValueError, 'unavailable'):
                manager.jpeg('left_wrist', 'left-usb')
            self.assertTrue(manager.jpeg('right_wrist', 'right-usb'))
            nodes.return_value = tuple(devices)
            manager.next_scan = 0
            self.assertTrue(manager.jpeg('left_wrist', 'left-usb'))
