import dataclasses
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
import numpy as np
from robol2l_deploy.runtime import StationRuntime,OutcomeUnknown
from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.contracts import Pose,MotionStrategy
from robol2l_deploy.errors import ApiError,ErrorCode
from robol2l_deploy.cameras import Frame,rectify

GOAL=Pose([0,0,.3],[1,0,0,0],'base')
class DeploymentTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.a=MockAdapter();self.r=StationRuntime(self.a,self.tmp.name,allow_motion=True)
    def tearDown(self):self.r.close();self.tmp.cleanup()
    def wait_job(self,key):
        deadline=time.monotonic()+3
        while (self.r.jobs[key]['status'] in ('accepted','running') or self.r._gate.locked()) and time.monotonic()<deadline:time.sleep(.005)
        self.assertNotIn(self.r.jobs[key]['status'],('accepted','running'));return self.r.jobs[key]
    def test_normalized_gripper_and_default_disabled(self):
        self.r.allow_motion=False;self.assertFalse(self.r.set_gripper(.5).ok);self.assertEqual(self.a.executions,0)
        self.r.allow_motion=True;self.assertTrue(self.r.set_gripper(.5).ok);self.assertEqual(self.r.get_robot_state().gripper_positions['primary'],.5)
        for bad in [float('nan'),-1,2,True]:
            with self.assertRaises(ValueError):self.r.set_gripper(bad)
    def test_unsupported_never_actuates(self):
        for name in ['step','go_home','move_to_pose','move_to_joints','set_grippers']:
            self.assertEqual(getattr(self.r,name)(None).error.code.value,'unsupported')
        self.assertFalse(self.r.plan_motion(GOAL,strategy=MotionStrategy()).ok);self.assertEqual(self.a.executions,0)
    def test_plan_start_binding_mutation_and_single_use(self):
        p=self.r.plan_motion(GOAL);self.a.q[0]=.1
        self.assertEqual(self.r.execute_trajectory(p.trajectory).status.value,'stale_plan');self.assertEqual(self.a.executions,0)
        self.a.q[0]=0;self.a.generation+=1;self.assertFalse(self.r.execute_trajectory(p.trajectory).ok)
        p=self.r.plan_motion(GOAL)
        changed=dataclasses.replace(p.trajectory,joint_positions=p.trajectory.joint_positions+.01)
        self.assertFalse(self.r.execute_trajectory(changed).ok)
        self.assertTrue(self.r.execute_trajectory(p.trajectory).ok);self.assertFalse(self.r.execute_trajectory(p.trajectory).ok)
    def test_pause_does_not_interrupt_dispatched_call(self):
        entered=threading.Event();release=threading.Event();original=self.a.execute
        def execute(*args):entered.set();release.wait(2);return original(*args)
        self.a.execute=execute;p=self.r.plan_motion(GOAL)
        self.r.submit('a','execute_trajectory',{'plan_id':p.diagnostics['plan_id']});self.assertTrue(entered.wait(1))
        self.r.pause();self.assertEqual(self.a.executions,0)
        with self.assertRaises(ApiError):self.r.submit('b','get_robot_state',{})
        release.set();self.assertTrue(self.wait_job('a')['result']['ok']);self.assertFalse(self.r.set_gripper(.2).ok)
    def test_deduplication_and_conflict(self):
        self.r.submit('a','set_gripper',{'position':.4});self.wait_job('a')
        self.r.submit('a','set_gripper',{'position':.4});self.assertEqual(self.a.executions,1)
        with self.assertRaises(ValueError):self.r.submit('a','set_gripper',{'position':.5})
    def test_unknown_latches_and_survives_restart(self):
        def broken(position):raise TimeoutError()
        self.a.set_gripper=broken;self.r.submit('a','set_gripper',{'position':.4})
        self.assertEqual(self.wait_job('a')['status'],'outcome_unknown');self.assertTrue(self.r.faulted)
        other=StationRuntime(MockAdapter(),self.tmp.name,allow_motion=True)
        self.assertTrue(other.faulted);self.assertFalse(other.set_gripper(.2).ok);other.close()
    def test_operator_unlock_recovers_the_controller_without_clearing_the_latch(self):
        self.r.faulted=True
        receipt=self.r.unlock()
        self.assertFalse(receipt['has_errors']);self.assertFalse(receipt['physical_evidence'])
        # Recovery talks to the controller; the latch belongs to the human gates.
        self.assertTrue(self.r.faulted);self.assertFalse(self.r.set_gripper(.2).ok)
        events=[json.loads(line) for line in (self.r.directory/'events.jsonl').read_text().splitlines()]
        self.assertEqual(events[-1]['kind'],'human_unlock');self.assertTrue(events[-1]['synthetic'])
        self.assertEqual(self.a.executions,0)
    def test_fault_acknowledgement_needs_an_idle_device_and_survives_restart(self):
        original=self.a.set_gripper
        def broken(position):raise TimeoutError()
        self.a.set_gripper=broken;self.r.submit('a','set_gripper',{'position':.4})
        self.assertEqual(self.wait_job('a')['status'],'outcome_unknown');self.assertTrue(self.r.faulted)
        self.a.set_gripper=original;reading=self.a.get_robot_state
        def not_idle():raise ApiError(ErrorCode.SAFETY_INTERLOCK,'robot_not_idle')
        self.a.get_robot_state=not_idle
        with self.assertRaises(ApiError):self.r.acknowledge_fault('human cleared the controller')
        self.assertTrue(self.r.faulted);self.assertFalse(self.r.set_gripper(.2).ok)
        self.a.get_robot_state=reading
        self.r.acknowledge_fault('human cleared the controller')
        self.assertFalse(self.r.faulted);self.assertTrue(self.r.set_gripper(.2).ok)
        other=StationRuntime(MockAdapter(),self.tmp.name,allow_motion=True)
        try:self.assertFalse(other.faulted)
        finally:other.close()
        for bad in ('','  ',None):
            with self.assertRaises(ValueError):self.r.acknowledge_fault(bad)
    def test_unknown_device_diagnostics_are_recorded(self):
        def broken(position):
            raise OutcomeUnknown('missing_receipt',diagnostics={'cause_type':'RuntimeError','cause_message':'holder mismatch'})
        self.a.set_gripper=broken
        with self.assertRaises(OutcomeUnknown):self.r.set_gripper(.4)
        events=[json.loads(line) for line in (self.r.directory/'events.jsonl').read_text().splitlines()]
        self.assertEqual(events[-1]['diagnostics']['cause_message'],'holder mismatch')
        self.assertTrue(self.r.faulted)
        other=StationRuntime(MockAdapter(),self.tmp.name,allow_motion=True)
        try:
            self.assertTrue(other.faulted)
            self.assertFalse(other.set_gripper(.4).ok)
        finally:other.close()
    def test_interrupted_direct_cap_dispatch_latches_on_restart(self):
        self.r.event('dispatch',operation='execute_trajectory',job_id=None,dispatch='sent')
        other=StationRuntime(MockAdapter(),self.tmp.name,allow_motion=True)
        try:
            self.assertTrue(other.faulted)
            self.assertFalse(other.set_gripper(.4).ok)
        finally:other.close()
    def test_restart_never_replays(self):
        self.r.jobs={'a':dict(status='running',dispatch='sent'),'b':dict(status='accepted',dispatch='not_sent')};self.r._persist()
        a=MockAdapter();other=StationRuntime(a,self.tmp.name,allow_motion=True)
        self.assertEqual(other.jobs['a']['status'],'outcome_unknown');self.assertEqual(other.jobs['b']['status'],'interrupted');self.assertEqual(a.executions,0);other.close()
    def test_six_dof_and_arbitrary_camera_names(self):
        a=MockAdapter(6,('overhead','side','tool'));other=StationRuntime(a,Path(self.tmp.name)/'six',allow_motion=True)
        try:
            obs=other.get_observation();self.assertEqual(set(obs.cameras),{'overhead','side','tool'})
            self.assertEqual(len(obs.robot_state.joint_positions['primary']),6)
            p=other.plan_motion(GOAL);self.assertTrue(other.execute_trajectory(p.trajectory).ok)
            key=next(iter(other.observations));point=other.localize_pixel(key,'side',[80,60])
            np.testing.assert_allclose(point['point'],[0,0,.5]);self.assertEqual(other.measure_region(key,'tool',[70,50,20,20])['point_count'],400)
            a.generation+=1
            with self.assertRaises(ValueError):other.localize_pixel(key,'side',[80,60])
        finally:other.close()
    def test_rectification_preserves_depth_edges(self):
        depth=np.ones((12,16));depth[:,8:]=2
        meta=dict(fx=10,fy=10,ppx=8,ppy=6,coeffs=[.1,0,0,0,0],model='distortion.brown_conrady')
        rgb,d,k=rectify(Frame(np.zeros((12,16,3),np.uint8),depth,meta,time.monotonic()))
        self.assertTrue(set(np.unique(d))<={0,1,2});self.assertEqual(k.shape,(3,3))
    def test_read_only_contract_arrays(self):
        s=self.r.get_robot_state()
        with self.assertRaises(ValueError):s.joint_positions['primary'][0]=1
    def test_fixed_upstream_contract_changes(self):
        from robol2l_deploy import contracts, errors
        from cap_harness import contracts as upstream, errors as upstream_errors
        self.assertIs(contracts.Pose, upstream.Pose)
        self.assertIs(contracts.RobotState, upstream.RobotState)
        self.assertIs(errors.ApiError, upstream_errors.ApiError)
        manifest=json.loads(Path('cap_harness/UPSTREAM.json').read_text())
        self.assertEqual(manifest['commit'],'fa00f0ef2bc1d91d594b9aa502c22133c9f538f7')
        self.assertEqual(upstream.JOINT_DIMENSIONS['franka_fr3'],7)


class NativeParityTests(unittest.TestCase):
    def test_native_time_curve_matches_python(self):
        import subprocess
        from scipy.interpolate import CubicSpline
        from franka_cap.planning import Kinematics
        binary=Path('native/build/franka_cap_motion')
        if not binary.exists():self.skipTest('native helper not built')
        q=np.array([0,-.3,0,-1.8,0,1.5,.7]);knots=np.linspace(q,q+np.array([.001,0,0,0,0,0,0]),101)
        start=Kinematics().forward(q)
        with tempfile.TemporaryDirectory() as directory:
            file=Path(directory)/'plan.txt'
            header=[*start.reshape(-1,order='F'),0,0,0,20,knots[-1,2],.08,101]
            file.write_text('FRANKA_CAP_JOINT_V1\n'+' '.join(map(str,header))+'\n'+'\n'.join(' '.join(map(str,row)) for row in knots))
            output=subprocess.check_output([str(binary),'--sample-time',str(file)],text=True)
        actual=np.array([json.loads(line) for line in output.splitlines()]);u=np.linspace(0,1,1001)
        expected=CubicSpline(np.linspace(0,1,101),knots,bc_type='natural')(u**4*(35+u*(-84+u*(70-20*u))))
        np.testing.assert_allclose(actual,expected,atol=1e-12,rtol=0)

class HardwareHttpTests(unittest.TestCase):
    def test_operator_unlock_route_returns_the_receipt_and_keeps_the_latch(self):
        from urllib.request import Request,urlopen
        from franka_player.server import Server,handler
        from franka_player.core import Console
        from robol2l_deploy.web import hardware_handler
        with tempfile.TemporaryDirectory() as directory:
            runtime=StationRuntime(MockAdapter(),Path(directory)/'hw')
            server=Server(('127.0.0.1',0),hardware_handler(handler(Console(Path(directory)/'console')),runtime))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            try:
                runtime.faulted=True
                with urlopen(Request('http://127.0.0.1:%d/api/hardware/unlock'%server.server_port,data=b'{}',
                        headers={'X-Console-Request':'1','Content-Type':'application/json'}),timeout=5) as response:
                    receipt=json.loads(response.read())
                self.assertFalse(receipt['has_errors']);self.assertFalse(receipt['physical_evidence'])
                self.assertTrue(runtime.faulted)
            finally:server.shutdown();server.server_close();runtime.close();thread.join(2)
    def test_workbench_freeze_plan_dispatch_and_origin(self):
        from urllib.request import Request,urlopen
        from urllib.error import HTTPError
        from franka_player.server import Server,handler
        from franka_player.core import Console
        from robol2l_deploy.web import hardware_handler
        with tempfile.TemporaryDirectory() as directory:
            runtime=StationRuntime(MockAdapter(),Path(directory)/'hw',allow_motion=True)
            server=Server(('127.0.0.1',0),hardware_handler(handler(Console(Path(directory)/'console')),runtime))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            root='http://127.0.0.1:'+str(server.server_port)
            def request(path,body=None,headers=None):
                data=None if body is None else json.dumps(body).encode()
                with urlopen(Request(root+path,data=data,headers=headers or {'X-Console-Request':'1'}),timeout=5) as response:return response.read()
            def job(key,op,args):
                request('/api/hardware/jobs',dict(request_id=key,operation=op,arguments=args))
                deadline=time.monotonic()+4
                while time.monotonic()<deadline:
                    state=json.loads(request('/api/hardware/state'));value=state['jobs'][key]
                    if value['status'] not in ('accepted','running') and not state['capabilities']['busy']:return value
                    time.sleep(.01)
                self.fail('job timeout')
            try:
                self.assertIn(b'Hardware Workbench',request('/hardware'))
                with self.assertRaises(HTTPError) as error:request('/api/hardware/jobs',dict(request_id='x',operation='get_robot_state',arguments={}),{'Origin':'http://evil','X-Console-Request':'1'})
                self.assertEqual(error.exception.code,403)
                obs=job('obs','get_observation',{});self.assertEqual(obs['status'],'completed')
                image=request('/api/hardware/image?camera=wrist&observation='+obs['observation_id']);self.assertTrue(image.startswith(b'\xff\xd8'))
                point=json.loads(request('/api/hardware/localize',dict(observation_id=obs['observation_id'],camera_name='wrist',pixel=[80,60])))
                np.testing.assert_allclose(point['point'],[0,0,.5])
                plan=job('plan','plan_motion',{'goal':dict(position=[0,0,.3],quaternion_wxyz=[1,0,0,0],frame='base')})
                run=job('exec','execute_trajectory',{'plan_id':plan['result']['diagnostics']['plan_id']});self.assertTrue(run['result']['ok'])
                again=job('exec','execute_trajectory',{'plan_id':plan['result']['diagnostics']['plan_id']});self.assertEqual(runtime.adapter.executions,1)
                self.assertEqual(again['status'],'completed')
                from robol2l_deploy.manipulation import Box,Scene
                runtime.install_scene(Scene((Box('test_object',runtime.adapter.pose,(.04,.04,.04),True),),
                    (0.,0.,0.),(0.,0.,1.),'synthetic-http',runtime.adapter.device_binding(),time.time(),True))
                self.assertTrue(job('grasp','close_gripper',{})['result']['ok'])
                self.assertTrue(job('release','open_gripper',{})['result']['ok'])
                self.assertEqual(runtime.adapter.executions,3)
                with urlopen(root+'/api/hardware/stream?camera=global&depth=1',timeout=5) as response:self.assertIn(b'--frame',response.read(20))
            finally:server.shutdown();server.server_close();runtime.close();thread.join(2)

class CameraStartupTests(unittest.TestCase):
    def source(self, skews):
        from unittest.mock import MagicMock
        from types import SimpleNamespace
        from robol2l_deploy.adapters.rgbd import RealSenseSource
        source=RealSenseSource('test');source.pipe=MagicMock();source.align=MagicMock();source.profile=MagicMock();source.rs=MagicMock()
        source.profile.get_device.return_value.first_depth_sensor.return_value.get_depth_scale.return_value=.001
        source.profile.get_device.return_value.get_info.return_value='test'
        frames=[]
        for skew in skews:
            rgb=MagicMock();depth=MagicMock();frame=MagicMock()
            rgb.timestamp=1000.;depth.timestamp=1000.+skew;rgb.frame_number=1;depth.frame_number=1
            rgb.get_frame_timestamp_domain.return_value=depth.get_frame_timestamp_domain.return_value='global'
            rgb.get_data.return_value=np.zeros((2,3,3),np.uint8);depth.get_data.return_value=np.ones((2,3),np.uint16)
            rgb.profile.as_video_stream_profile.return_value.intrinsics=SimpleNamespace(width=3,height=2,fx=2,fy=2,ppx=1,ppy=1,coeffs=[0]*5,model='distortion.none')
            frame.get_color_frame.return_value=rgb;frame.get_depth_frame.return_value=depth;frames.append(frame)
        source.align.process.side_effect=frames
        return source
    def test_initial_bad_pair_is_discarded(self):
        source=self.source([133,.06]);frame=source.read()
        self.assertEqual(source.pipe.wait_for_frames.call_count,2)
        self.assertAlmostEqual(abs(frame.metadata['color_timestamp_ms']-frame.metadata['depth_timestamp_ms']),.06)
    def test_persistently_skewed_frames_fail(self):
        with self.assertRaisesRegex(RuntimeError,'rgbd_timestamp_skew'):self.source([133]*15).read()
