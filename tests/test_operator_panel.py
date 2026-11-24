"""Human panel and launcher tests. All hardware and perception data are mock."""
import json
from pathlib import Path
import socket
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch
import requests
import numpy as np
from franka_player.core import Console
from franka_player.server import Server, handler
from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.cap.session import CapController
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cap.tasks import MockReachTask
from robol2l_deploy.contracts import Segmentation, SegmentationSet
from robol2l_deploy.operator import OperatorConsole
from robol2l_deploy.runtime import StationRuntime
from robol2l_deploy.web import hardware_handler
from robol2l_deploy.launcher import HarnessLauncher

GOAL={'$pose':dict(position=[0,0,.32],quaternion_wxyz=[1,0,0,0],frame='base')}

class OperatorTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.runtime=StationRuntime(MockAdapter(),self.temp.name)
        self.cap=CapController(self.runtime)
        self.operator=OperatorConsole(self.runtime,self.cap,providers={})
        self.counter=0
    def tearDown(self):
        self.runtime.close();self.temp.cleanup()
    def call(self,name,args=None,key=None):
        self.counter+=1;key=key or str(self.counter)
        self.operator.submit(key,name,args or {})
        deadline=time.monotonic()+4
        while self.runtime.jobs[key]['status'] in ('accepted','running') and time.monotonic()<deadline:time.sleep(.01)
        self.assertNotIn(self.runtime.jobs[key]['status'],('accepted','running'))
        return self.runtime.jobs[key]
    def test_authorization_plan_reference_execute_dedup_and_manual_evidence(self):
        self.operator.authorize('Mock validation only')
        plan=self.call('plan_motion',{'goal':GOAL})['result']
        self.assertTrue(plan['value']['ok']);self.assertEqual(len(plan['value']['diagnostics']['preview_xyz']),11)
        arguments={'trajectory':{'$ref':plan['handle'],'path':['trajectory']}}
        result=self.call('execute_trajectory',arguments,key='execute')
        self.assertTrue(result['result']['value']['ok'])
        self.call('execute_trajectory',arguments,key='execute')
        self.assertEqual(self.runtime.adapter.executions,1)
        events=[json.loads(line) for line in (Path(self.temp.name)/'events.jsonl').read_text().splitlines()]
        manual=[e for e in events if e['kind']=='operator_call']
        self.assertTrue(manual);self.assertFalse(any(e['autonomy_evidence'] for e in manual))
        self.assertTrue((Path(self.temp.name)/'operator'/plan['handle']/'arrays.npz').exists())
    def test_disabled_motion_and_unknown_operation_never_dispatch(self):
        result=self.call('set_gripper',{'position':.5})
        self.assertFalse(result['result']['value']['ok'])
        for op in ('eval','__import__','go_home'):
            with self.assertRaises(ValueError):self.operator.submit('bad',op,{})
        self.assertEqual(self.runtime.adapter.executions,0)
    def test_connection_reports_measurements_and_failures_without_enabling_motion(self):
        from robol2l_deploy.errors import ApiError, ErrorCode
        from dataclasses import FrozenInstanceError
        self.assertEqual(self.runtime.snapshot()['connection']['status'],'not_connected')
        self.call('get_observation')
        connection=self.runtime.snapshot()['connection']
        self.assertEqual(connection['status'],'observation_read')
        self.assertEqual(set(connection['cameras']),{'wrist','global'})
        self.assertEqual(connection['cameras']['global']['depth_shape'],[120,160])
        error=ApiError(ErrorCode.SAFETY_INTERLOCK,'FCI refused')
        with self.assertRaises(FrozenInstanceError):error.message='changed'
        with self.assertRaises(FrozenInstanceError):del error.message
        with patch.object(self.runtime.adapter,'get_robot_state',side_effect=error):
            self.call('get_robot_state')
        self.assertEqual(self.runtime.snapshot()['connection']['status'],'error')
        self.assertEqual(self.runtime.snapshot()['connection']['error'],'safety_interlock: FCI refused')
        self.assertEqual(self.runtime.jobs[str(self.counter)]['error'],'safety_interlock: FCI refused')
        self.assertFalse(self.runtime.allow_motion)
        self.assertEqual(self.runtime.adapter.executions,0)
    def test_references_are_typed_and_perception_epoch_is_not_shared_with_cap(self):
        obs=self.call('get_observation')['result'];epoch=obs['observation_id']
        mask=Segmentation(np.ones((120,160),bool),'software fixture',1.,'global','global',observation_id=epoch)
        self.operator.references['fixture']={'value':SegmentationSet(True,(mask,)), 'observation_id':epoch,'operation':'test-only'}
        cloud=self.call('mask_to_point_cloud',{'mask':{'$ref':'fixture','path':['segmentations',0]},'camera_name':'global','target_frame':'base'})['result']
        self.assertEqual(cloud['value']['frame'],'base')
        geometry=self.call('estimate_geometry',{'point_cloud':{'$ref':cloud['handle']}})
        self.assertEqual(geometry['status'],'failed')
        self.assertIn('degenerate',geometry['error'])  # The mock depth image is a plane.
        autonomous=PublicAPI(self.runtime,MockReachTask.context,providers={})
        self.assertIsNone(autonomous.environment.observation)
        with self.assertRaises(ValueError):self.operator.decode({'$ref':obs['handle'],'path':['__class__']})
        self.call('get_observation')
        stale=self.call('estimate_geometry',{'point_cloud':{'$ref':cloud['handle']}})
        self.assertEqual(stale['status'],'failed');self.assertIn('degenerate',stale['error'])
    def test_cap_busy_blocks_human_commands(self):
        self.cap.thread=Mock();self.cap.thread.is_alive.return_value=True
        with self.assertRaisesRegex(ValueError,'cap_session_busy'):self.operator.submit('x','get_robot_state',{})
        with self.assertRaisesRegex(ValueError,'cap_session_busy'):self.operator.authorize('test')
    def test_physical_authorization_uses_owner_lease_and_never_bypasses_runtime(self):
        def authorize(runtime,*args,**kwargs):runtime.get_robot_state()
        with patch.object(self.runtime.adapter,'synthetic',False),patch('robol2l_deploy.physical.authorize',side_effect=authorize) as permission:
            self.operator.authorize('Software stub, no physical experiment')
            self.assertEqual(permission.call_args.args[1],'manual-workbench')
            result=self.call('open_gripper')
            self.assertEqual(result['status'],'failed')
            self.assertIn('manual_motion_authorization_required',result['error'])
        self.assertEqual(self.runtime.adapter.executions,0)

class PanelHttpTests(unittest.TestCase):
    def test_routes_origin_artifacts_and_session_end(self):
        with tempfile.TemporaryDirectory() as directory:
            console=Console(Path(directory)/'console');runtime=StationRuntime(MockAdapter(),Path(directory)/'hw')
            server=Server(('127.0.0.1',0),hardware_handler(handler(console),runtime))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            client=requests.Session();client.trust_env=False
            base=f'http://127.0.0.1:{server.server_port}'
            def post(route,body,**kwargs):return client.post(base+'/api/hardware/'+route,json=body,headers={'X-Console-Request':'1',**kwargs},timeout=5)
            try:
                self.assertEqual(client.get(base+'/api/hardware/operator').status_code,200)
                self.assertEqual(post('cap/model',{'mode':'codex'}).status_code,200)
                self.assertEqual(json.loads((runtime.directory/'cap-model.json').read_text()),{'mode':'codex'})
                self.assertEqual(post('cap/model',{'mode':'unknown'}).status_code,409)
                self.assertEqual(post('operator/authorize',dict(reason='Mock only',probe_distance_m=.02,confirmed=True)).status_code,200)
                self.assertEqual(post('operator/call',dict(request_id='x',operation='get_observation',arguments={})).status_code,202)
                deadline=time.monotonic()+4
                while runtime.jobs['x']['status'] in ('accepted','running') and time.monotonic()<deadline:time.sleep(.01)
                evidence=client.get(base+'/api/hardware/evidence').json()
                self.assertTrue(evidence['artifacts'])
                artifact=evidence['artifacts'][0]
                self.assertEqual(client.get(base+'/api/hardware/artifact',params={'path':artifact}).status_code,200)
                self.assertEqual(client.get(base+'/api/hardware/artifact',params={'path':'../../config/station.json'}).status_code,409)
                self.assertEqual(post('operator/revoke',{},Origin='http://evil').status_code,403)
                self.assertEqual(post('cap/start',dict(model_mode='fixture',task='probe',authorization='mock',probe_distance_m=.02,confirmed=True)).status_code,409)
                self.assertEqual(post('session/end',{'confirmed':True}).status_code,200)
                thread.join(3);self.assertFalse(thread.is_alive());self.assertTrue(runtime.closed)
            finally:server.shutdown();server.server_close();runtime.close();console.shutdown()

class LauncherTests(unittest.TestCase):
    def test_live_listener_is_never_replaced(self):
        with tempfile.TemporaryDirectory() as folder, socket.socket() as listener:
            listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            listener.bind(('127.0.0.1',0));listener.listen()
            manager=HarnessLauncher(folder,'config/station.json',runtime_port=listener.getsockname()[1])
            with patch('subprocess.Popen') as start:
                with self.assertRaises(OSError):manager.start('mock')
                start.assert_not_called()
    def test_backend_selection_builds_only_the_selected_fixed_process_command(self):
        for backend in ('panda_py','franky'):
            with tempfile.TemporaryDirectory() as folder:
                manager=HarnessLauncher(folder,'config/station.json',runtime_port=0)
                manager.probe=lambda: {'session_id':manager.record['session_id']} if manager.record.get('active') else None
                process=Mock(pid=12345);process.poll.return_value=None
                manager.client.post=Mock(return_value=Mock())
                with patch('subprocess.Popen',return_value=process) as start, patch('robol2l_deploy.providers.services.ensure_local_services'):
                    manager.start(backend)
                    command=start.call_args.args[0]
                    self.assertEqual(command[command.index('--robot-backend')+1],backend)
                    self.assertNotIn('--hardware-motion',command)
                    self.assertNotIn('shell',start.call_args.kwargs)
                    self.assertEqual(manager.client.post.call_args.kwargs['json']['operation'],'get_observation')
                    with self.assertRaises(ValueError):manager.start('mock')
                with self.assertRaises(ValueError):manager.start('arbitrary-command')
    def test_real_mock_process_starts_ends_and_restart_does_not_launch(self):
        with tempfile.TemporaryDirectory() as folder:
            with socket.socket() as port:port.bind(('127.0.0.1',0));number=port.getsockname()[1]
            manager=HarnessLauncher(folder,'config/station.json',runtime_port=number)
            try:
                result=manager.start('mock');self.assertTrue(result['running'])
                self.assertEqual(result['session']['backend'],'mock')
                observer=HarnessLauncher(folder,'config/station.json',runtime_port=number)
                self.assertIsNone(observer.process)
                self.assertTrue(observer.snapshot()['running'])
                result=manager.stop();self.assertFalse(result['running'])
                # Immediate reuse must work even with previous HTTP TIME_WAIT sockets.
                self.assertTrue(manager.start('mock')['running'])
                self.assertFalse(manager.stop()['running'])
                restored=HarnessLauncher(folder,'config/station.json',runtime_port=number)
                self.assertIsNone(restored.process);self.assertFalse(restored.snapshot()['running'])
            finally:
                if manager.process and manager.process.poll() is None:manager.stop()

if __name__=='__main__':unittest.main()
