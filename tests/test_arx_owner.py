"""Resident-owner lifecycle tests. All SDK arms are software fixtures."""
from contextlib import nullcontext
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from robol2l_deploy.arx_owner import OwnerState, OwnerClient, OwnerRejected, rpc, serve, owner_lock, ensure_owner
from robol2l_deploy.adapters.arx import ARXDriver
from robol2l_deploy.launcher import HarnessLauncher
from test_arx_dual import fixture_station


class OwnerTests(unittest.TestCase):
    def setUp(self):
        self.station = fixture_station()
        self.addCleanup(self.station.close)
        self.arms = {}
        for side, unit in self.station.arms.items():
            arm = unit.driver.arm
            unit.driver._arm = arm
            arm.close = Mock()
            arm.damp = Mock()
            self.arms[side] = arm
        self.station.close = Mock()
        self.now = 10.
        self.events = []
        self.state = OwnerState(self.station, clock=lambda: self.now, journal=self.events.append)

    def attach(self):
        return self.state.handle(dict(op='attach', identity=self.state.identity))['token']

    def request(self, token, op, **kwargs):
        return self.state.handle(dict(op=op, token=token, **kwargs))

    def test_graceful_detach_and_reattach_reuse_sdk_and_disable_dispatch(self):
        token = self.attach()
        self.request(token, 'authorize', enabled=True)
        self.request(token, 'sdk', side='left', method='set_joint_positions',
                     kwargs=dict(positions=[.1]*6, joint_names=[f'joint{i}' for i in range(1,7)], duration=1.))
        self.request(token, 'detach')
        self.assertFalse(self.state.authorized)
        for arm in self.arms.values():
            arm.close.assert_not_called()
            arm.damp.assert_not_called()
            self.assertFalse(arm.protected)
        second = self.attach()
        self.assertNotEqual(token, second)
        with self.assertRaisesRegex(ValueError, 'motion_disabled'):
            self.request(second, 'sdk', side='left', method='set_gripper_pos', args=[0.])
        self.assertIs(self.station.arms['left'].driver.arm, self.arms['left'])

    def test_elapsed_time_does_not_revoke_client_but_disconnect_preserves_support(self):
        token = self.attach()
        self.request(token, 'authorize', enabled=True)
        self.arms['left'].gripper = 2.5
        self.now += 10000
        self.state.expire()
        self.assertEqual(self.state.token, token)
        self.assertTrue(self.state.authorized)
        self.state.lose_client('fixture_process_exited')
        self.assertIsNone(self.state.token)
        self.assertEqual(self.arms['left'].gripper, 2.5)
        self.assertEqual(self.arms['left'].commands[-1][-1], 0.)
        for arm in self.arms.values():
            arm.close.assert_not_called()
            arm.damp.assert_not_called()
        with self.assertRaisesRegex(ValueError, 'lease_lost'):
            self.request(token, 'sdk', side='left', method='set_gripper_pos', args=[0.])

    def test_faulted_arm_is_not_forced_out_of_native_protection(self):
        token = self.attach()
        self.arms['left'].fault = 'native_fault'
        self.request(token, 'detach')
        self.assertFalse(self.arms['left'].commands)
        self.assertEqual(self.state.last_hold['left']['strategy'], 'native_safety_retained')
        self.assertTrue(self.arms['right'].commands)

    def test_owner_has_one_lease_and_rejects_model_change(self):
        token = self.attach()
        with self.assertRaisesRegex(ValueError, 'already_attached'):
            self.attach()
        self.request(token, 'detach')
        with self.assertRaisesRegex(ValueError, 'model_changed'):
            self.state.handle(dict(op='attach', identity='changed-model'))

    def test_maintenance_requires_support_confirmation_and_is_not_an_sdk_rpc(self):
        token = self.attach()
        for method in ('close','damp','protect_mode','__getattribute__'):
            with self.assertRaisesRegex(ValueError, 'not_allowed'):
                self.request(token,'sdk',side='left',method=method)
        with self.assertRaisesRegex(ValueError, 'support_confirmation'):
            self.request(token,'shutdown')
        self.station.close.assert_not_called()
        self.request(token,'shutdown',hardware_supported=True)
        self.station.close.assert_called_once_with(hardware_supported=True)
        self.assertTrue(self.state.stopped)

    def test_unattached_model_mismatch_can_be_maintained_without_loading_new_model(self):
        with self.assertRaisesRegex(ValueError, 'confirmation'):
            self.state.handle(dict(op='maintenance_shutdown',instance=self.state.instance))
        self.state.handle(dict(op='maintenance_shutdown',instance=self.state.instance,hardware_supported=True))
        self.station.close.assert_called_once_with(hardware_supported=True)

    def test_direct_driver_cannot_close_until_supported(self):
        driver=ARXDriver('can1', acquire_lock=nullcontext)
        driver._arm=self.arms['left']
        with self.assertRaisesRegex(RuntimeError,'support_confirmation'):
            driver.close()
        self.assertIs(driver._arm,self.arms['left'])
        driver.close(hardware_supported=True)
        self.arms['left'].close.assert_called_once()
        self.arms['left'].damp.assert_not_called()

    def test_failed_shutdown_retains_handle_and_blocks_new_attachments(self):
        driver=ARXDriver('can1',acquire_lock=nullcontext)
        driver._arm=self.arms['left']
        self.arms['left'].close.side_effect=RuntimeError('close outcome unknown')
        with self.assertRaisesRegex(RuntimeError,'unknown'):
            driver.close(hardware_supported=True)
        self.assertIs(driver._arm,self.arms['left'])
        token=self.attach()
        self.station.close.side_effect=RuntimeError('close outcome unknown')
        with self.assertRaisesRegex(RuntimeError,'unknown'):
            self.request(token,'shutdown',hardware_supported=True)
        self.assertFalse(self.state.stopped)
        self.assertTrue(self.state.closing)
        with self.assertRaisesRegex(ValueError,'shutdown_unconfirmed'):
            self.attach()

    def test_client_lost_reply_is_never_retried_or_reattached(self):
        client=OwnerClient('/unused','model')
        client.token='existing'
        with patch('robol2l_deploy.arx_owner.rpc',side_effect=TimeoutError) as call:
            with self.assertRaisesRegex(RuntimeError,'unknown_no_retry'):
                client.call('sdk',side='left',method='set_gripper_pos',args=[0.])
            with self.assertRaisesRegex(RuntimeError,'new_session_required'):
                client.call('sdk',side='left',method='set_gripper_pos',args=[0.])
            self.assertEqual(call.call_count,1)

    def test_unresponsive_locked_owner_never_spawns_replacement(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'owner.sock'
            with owner_lock(path), patch('robol2l_deploy.arx_owner.socket_path',return_value=path), patch('subprocess.Popen') as start:
                with self.assertRaisesRegex(RuntimeError,'do_not_start_another_controller'):
                    ensure_owner('/unused',folder)
                start.assert_not_called()

    def test_real_local_rpc_console_process_loss_keeps_owner_alive(self):
        # Only this fixture station is served; no installed SDK is constructed.
        self.state.clock=time.monotonic
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'owner.sock'
            thread=threading.Thread(target=serve,args=(self.state,path),daemon=True)
            thread.start()
            deadline=time.monotonic()+3
            while not path.exists() and time.monotonic()<deadline:time.sleep(.01)
            first=OwnerClient(path,self.state.identity)
            first.arm('left').get_status()
            instance=first.instance
            first.close()
            second=OwnerClient(path,self.state.identity)
            second.arm('right').get_status()
            self.assertEqual(instance,second.instance)
            second.call('authorize',enabled=True)
            second.arm('right').set_gripper_pos(2.,duration=1.)
            self.assertEqual(self.arms['right'].gripper,2.)
            second.close()
            code=('import sys,time; from robol2l_deploy.arx_owner import OwnerClient; '
                  'c=OwnerClient(sys.argv[1],sys.argv[2]); c.attach(); '
                  'print("attached",flush=True); time.sleep(60)')
            child=subprocess.Popen([sys.executable,'-c',code,str(path),self.state.identity],stdout=subprocess.PIPE,text=True)
            try:
                self.assertEqual(child.stdout.readline().strip(),'attached')
                child.kill();child.wait(timeout=3)
                deadline=time.monotonic()+5
                while self.state.token and time.monotonic()<deadline:time.sleep(.05)
                self.assertIsNone(self.state.token)
                self.assertTrue(thread.is_alive())
                for arm in self.arms.values():
                    arm.close.assert_not_called();arm.damp.assert_not_called()
                final=OwnerClient(path,self.state.identity)
                final.attach()
                self.assertEqual(final.instance,instance)
                final.close(hardware_supported=True)
                thread.join(3)
                self.assertFalse(thread.is_alive())
            finally:
                if child.poll() is None:child.kill();child.wait()
                child.stdout.close()
                self.state.stopped=True

    def test_changed_model_client_can_only_shutdown_via_confirmed_maintenance(self):
        client=OwnerClient('/unused','changed')
        def local(path, body, **kwargs):
            try:return self.state.handle(body)
            except ValueError as error:raise OwnerRejected(str(error)) from error
        with patch('robol2l_deploy.arx_owner.rpc',side_effect=local):
            with self.assertRaisesRegex(OwnerRejected,'model_changed'):
                client.attach()
            self.station.close.assert_not_called()
            client.close(hardware_supported=True)
        self.station.close.assert_called_once_with(hardware_supported=True)

    def test_revoke_local_motion_succeeds_even_if_owner_has_disappeared(self):
        client=OwnerClient('/unused','model')
        client.token='existing'
        self.station.owner_client=client
        for unit in self.station.arms.values():unit.driver.allow_motion=True
        with patch.object(client,'call',side_effect=OSError('offline')):
            self.station.allow_motion=False
        self.assertFalse(self.station.allow_motion)
        self.assertTrue(client.lost)


class LauncherLifecycleTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.manager=HarnessLauncher(temp.name,'/unused')
        self.manager.record=dict(active=True,backend='arx_x5_2023_dual',session_id='old',url='http://unused',directory=temp.name)

    def test_legacy_reload_stop_switch_block_before_any_disconnect(self):
        self.manager.probe=lambda:dict(session_id='old',busy=False,cap_running=False)
        self.manager.client.post=Mock()
        for operation in (self.manager.stop,self.manager.restart,lambda:self.manager.connect('mock')):
            with self.assertRaisesRegex(ValueError,'support_confirmation'):
                operation()
        self.manager.client.post.assert_not_called()

    def test_resident_reload_preserves_owner_without_support_ack(self):
        state=dict(session_id='old',busy=False,cap_running=False,hardware_lifecycle={'mode':'resident_arx_owner_v1'})
        self.manager.probe=lambda:state
        with patch.object(self.manager,'stop') as stop,patch.object(self.manager,'start',return_value={}) as start:
            self.manager.restart()
        stop.assert_called_once_with(hardware_supported=False)
        start.assert_called_once_with('arx_x5_2023_dual',history_directory=self.manager.record['directory'])

    def test_busy_experiment_blocks_even_confirmed_maintenance(self):
        self.manager.probe=lambda:dict(session_id='old',busy=False,cap_running=True)
        with patch.object(self.manager,'stop') as stop:
            with self.assertRaisesRegex(ValueError,'station_busy'):
                self.manager.maintenance(reload=True)
        stop.assert_not_called()

    def test_ending_console_is_not_permission_to_switch_hardware_backend(self):
        self.manager.probe=lambda:None
        with patch.object(self.manager,'start') as start:
            with self.assertRaisesRegex(ValueError,'support_confirmation'):
                self.manager.connect('franky')
        start.assert_not_called()


class DisconnectHttpTests(unittest.TestCase):
    def test_legacy_and_resident_routes_enforce_lifecycle_before_closing(self):
        import requests
        from franka_player.core import Console
        from franka_player.server import Server, handler
        from robol2l_deploy.adapters.mock import MockAdapter
        from robol2l_deploy.runtime import StationRuntime
        from robol2l_deploy.web import hardware_handler
        for resident in (False, True):
            with self.subTest(resident=resident), tempfile.TemporaryDirectory() as folder:
                adapter=MockAdapter()
                adapter.hardware_lifecycle=lambda:dict(mode='resident_arx_owner_v1' if resident else 'in_process',
                    disconnect_requires_support_confirmation=not resident)
                adapter.prepare_disconnect=Mock()
                runtime=StationRuntime(adapter,Path(folder)/'hardware')
                console=Console(Path(folder)/'console')
                server=Server(('127.0.0.1',0),hardware_handler(handler(console),runtime))
                thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
                session=requests.Session();session.trust_env=False
                try:
                    url=f'http://127.0.0.1:{server.server_address[1]}/api/hardware/session/end'
                    response=session.post(url,json={'confirmed':True},headers={'X-Console-Request':'1'},timeout=5)
                    if not resident:
                        self.assertEqual(response.status_code,409)
                        self.assertFalse(runtime.closed)
                        adapter.prepare_disconnect.assert_not_called()
                        response=session.post(url,json={'confirmed':True,'hardware_supported':True},
                                              headers={'X-Console-Request':'1'},timeout=5)
                    self.assertEqual(response.status_code,200,response.text)
                    adapter.prepare_disconnect.assert_called_once_with(hardware_supported=not resident)
                finally:
                    server.shutdown();thread.join(3);server.server_close()
                    session.close();runtime.close();console.shutdown()


if __name__=='__main__':unittest.main()
