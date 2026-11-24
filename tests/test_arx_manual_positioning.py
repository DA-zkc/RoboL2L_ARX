"""Manual recovery tests with software arms only; never open SDK/CAN devices."""
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from robol2l_deploy.arx_teach import TeachKey
from robol2l_deploy.arx_owner import OwnerState
from robol2l_deploy.runtime import StationRuntime
from test_arx_dual import fixture_station


class ManualTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.station = fixture_station()
        self.addCleanup(self.station.close)
        self.rt = StationRuntime(self.station, tmp.name)
        self.key = TeachKey(self.rt)

    def wait_phase(self, phase):
        end = time.monotonic()+2
        while self.key.status()['phase'] != phase and time.monotonic() < end:
            time.sleep(.01)
        self.assertEqual(self.key.status()['phase'], phase)

    def finish(self):
        self.key.hold()
        with self.key.lock:
            self.assertTrue(self.key.lock.wait_for(lambda: self.key.token is None, 2))

    def authorize(self, rt, *args):
        rt.adapter.allow_motion = rt.allow_motion = True

    def test_ten_second_countdown_unlimited_drag_and_selected_arm_hold(self):
        now = [100.]
        dragged = Mock()
        with (patch('robol2l_deploy.arx_teach.time', SimpleNamespace(monotonic=lambda:now[0])),
              patch('robol2l_deploy.arx_teach.authorize', self.authorize),
              patch.object(self.station, 'teach_drag', dragged)):
            self.key.prepare('left', delayed=True)
            try:
                self.wait_phase('countdown')
                self.assertEqual(self.key.status()['countdown_s'], 10.)
                now[0] = 109.99
                time.sleep(.1)
                dragged.assert_not_called()
                self.assertFalse(self.rt.allow_motion)
                now[0] = 110.
                self.wait_phase('drag')
                # No browser heartbeat and well beyond the former 30 s cap.
                now[0] = 510.
                time.sleep(.15)
                self.assertEqual(self.key.phase, 'drag')
                with self.assertRaisesRegex(ValueError, 'busy'):
                    self.key.prepare('right', delayed=True)
                self.finish()
            finally:
                self.finish()
        dragged.assert_called_once_with('left')
        self.assertEqual(self.key.phase, 'held')
        self.assertFalse(self.station.arms['right'].driver.arm.commands)
        self.assertFalse(self.rt.allow_motion or self.station.allow_motion)
        self.assertEqual(self.key.exit_reason, 'operator_hold')

    def test_cancel_pause_fault_and_close_during_countdown_never_dispatch(self):
        for stop in ('cancel', 'paused', 'faulted', 'closed'):
            with self.subTest(stop=stop), patch('robol2l_deploy.arx_teach.authorize') as authorize:
                self.rt.paused = self.rt.faulted = self.rt.closed = False
                self.key.prepare('right', delayed=True)
                self.wait_phase('countdown')
                if stop == 'cancel': self.key.hold()
                else: setattr(self.rt, stop, True)
                self.finish()
                self.assertEqual(self.key.phase, 'cancelled')
                authorize.assert_not_called()
        self.rt.closed = False
        self.assertFalse(self.station.arms['right'].driver.arm.commands)

    def test_faulted_start_and_old_owner_reject_without_queued_work(self):
        self.rt.faulted = True
        with self.assertRaisesRegex(ValueError, 'faulted'):
            self.key.prepare('left', delayed=True)
        self.rt.faulted = False
        self.station.owner_client = SimpleNamespace(call=lambda *a, **k: {})
        try:
            with self.assertRaisesRegex(ValueError, 'maintenance_update'):
                self.key.prepare('left', delayed=True)
            self.assertIsNone(self.key.token)
        finally:
            self.station.owner_client = None

    def test_feedback_fault_recheck_authorizes_resident_owner_and_revokes(self):
        owner = SimpleNamespace(token='test', lost=False, call=Mock())
        self.station.owner_client = owner
        self.addCleanup(lambda: setattr(self.station, 'owner_client', None))
        self.station.allow_motion = False
        self.rt.faulted = True
        self.rt.teach_fault = dict(kind='feedback', side='left', error='hold_feedback_not_verified')
        left = self.station.arms['left'].driver.arm
        original = left.set_joint_positions
        def dispatch(**kwargs):
            self.assertEqual(owner.call.call_args.kwargs, {'enabled': True})
            return original(**kwargs)
        with patch.object(self.station, 'realtime_ready', return_value=True), patch.object(left, 'set_joint_positions', dispatch):
            self.assertEqual(self.key.acknowledge_parked('left', recheck_hold=True)['phase'], 'held')
        self.assertFalse(self.rt.faulted or self.rt.allow_motion or self.station.allow_motion)
        self.assertEqual(owner.call.call_args.kwargs, {'enabled': False})
        self.assertFalse(self.station.arms['right'].driver.arm.commands)

    def test_hold_reports_velocity_without_project_fault_or_verified_claim(self):
        left = self.station.arms['left'].driver.arm
        original = left.get_status
        with patch.object(left, 'get_status', lambda: dict(original(), v=np.full(6, .11868))):
            receipt = self.station.teach_hold('left')
        self.assertAlmostEqual(receipt['max_speed_rad_s'], .11868)
        self.assertFalse(receipt['hold_verified'] or left.protected)
        self.assertNotIn('thresholds', receipt)

    def test_owner_loss_exits_only_known_gravity_arm_without_damping(self):
        now = [0.]
        owner = OwnerState(self.station, clock=lambda: now[0])
        token = owner.handle(dict(op='attach', identity=owner.identity))['token']
        owner.handle(dict(op='authorize', token=token, enabled=True))
        left = self.station.arms['left'].driver.arm
        original = left.get_status
        mode = ['GRAVITY']
        def status(): return dict(original(), mode=mode[0])
        original_set = left.set_joint_positions
        def hold(**kwargs):
            mode[0] = 'JOINT'
            return original_set(**kwargs)
        for unit in self.station.arms.values(): unit.driver._arm = unit.driver.arm
        with (patch.object(left, 'gravity_compensation', return_value=True, create=True),
              patch.object(left, 'get_status', status), patch.object(left, 'set_joint_positions', hold)):
            owner.handle(dict(op='sdk', token=token, side='left', method='gravity_compensation'))
            owner.lose_client('fixture_process_exited')
        self.assertIsNone(owner.token)
        self.assertFalse(owner.last_hold['left']['hold_verified'])
        self.assertEqual(mode[0], 'JOINT')
        self.assertFalse(left.protected)

    def test_hold_does_not_change_native_protect_or_fault(self):
        left = self.station.arms['left'].driver.arm
        left.protected = True
        with self.assertRaisesRegex(RuntimeError, 'healthy_joint_or_gravity'):
            self.station.teach_hold('left')
        left.protected = False
        left.fault = 'native fault'
        with self.assertRaisesRegex(RuntimeError, 'fault_or_offline'):
            self.station.teach_hold('left')
        self.assertFalse(left.commands)

    def test_unchanged_sdk_cycle_is_not_a_project_fault_or_physical_verification(self):
        left = self.station.arms['left'].driver.arm
        original = left.get_status
        with patch.object(left, 'get_status', lambda: dict(original(), cycles=42)):
            receipt = self.station.teach_hold('left')
        self.assertFalse(receipt['hold_verified'])
        self.assertEqual(receipt['samples'][0]['cycles'], 42)


class ManualHttpTests(unittest.TestCase):
    def test_routes_without_calibration_batch_and_waiting_experiment(self):
        import requests
        from franka_player.core import Console
        from franka_player.server import Server, handler
        from robol2l_deploy.web import hardware_handler
        from robol2l_deploy.cap.experiment import ExperimentController
        from robol2l_deploy.cap.session import CapController
        with tempfile.TemporaryDirectory() as directory:
            station = fixture_station()
            rt = StationRuntime(station, Path(directory)/'hardware')
            console = Console(Path(directory)/'console')
            cap = CapController(rt)
            server = Server(('127.0.0.1', 0), hardware_handler(handler(console), rt, cap_controller=cap))
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            client = requests.Session(); client.trust_env = False
            url = f'http://127.0.0.1:{server.server_port}'
            def post(action, body):
                return client.post(url+'/api/hardware/manual-positioning/'+action, json=body,
                                   headers={'X-Console-Request':'1'}, timeout=5)
            def status(): return client.get(url+'/api/hardware/manual-positioning/status', timeout=5).json()
            try:
                html = client.get(url+'/hardware', timeout=5).text
                self.assertEqual(html.count('data-manual-positioning'), 2)
                self.assertEqual(client.get(url+'/manual-positioning.js', timeout=5).status_code, 200)
                self.assertEqual(status()['teach']['phase'], 'idle')
                with patch.object(ExperimentController, 'busy', return_value=True):
                    self.assertEqual(post('start', {'side':'left'}).status_code, 409)
                # Active but waiting experiment may position manually without ending it.
                with patch.object(ExperimentController, 'active', return_value=True):
                    self.assertEqual(post('start', {'side':'right'}).status_code, 200)
                    self.assertEqual(post('start', {'side':'left'}).status_code, 409)
                    self.assertEqual(post('hold', {}).status_code, 200)
                end = time.monotonic()+2
                while status()['teach']['active'] and time.monotonic()<end: time.sleep(.01)
                self.assertEqual(status()['teach']['phase'], 'cancelled')
                self.assertFalse(station.arms['right'].driver.arm.commands)
                self.assertFalse(list(rt.directory.glob('calibration/*')))
                rt.faulted = True
                self.assertEqual(post('start', {'side':'left'}).status_code, 409)
                self.assertEqual(post('recheck-hold', {'side':'left','confirmed':True}).status_code, 409)
                self.assertTrue(rt.faulted)
                cap.thread = SimpleNamespace(is_alive=lambda:True)
                self.assertEqual(post('start', {'side':'left'}).status_code, 409)
            finally:
                cap.thread = None
                server.shutdown(); server.server_close(); thread.join(2)
                rt.close(); console.shutdown(); client.close()


if __name__ == '__main__': unittest.main()
