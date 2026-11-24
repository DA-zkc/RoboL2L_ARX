"""Software-only checks: real FR3 adapter, fake driver; never connect hardware."""
import base64
import copy
import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np

from franka_cap.common import config
from franka_cap.planning import Kinematics
from robol2l_deploy.adapters.fr3 import FR3Adapter
from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cap.model import messages
from robol2l_deploy.cap.tasks import MockReachTask
from robol2l_deploy.contracts import Pose
from robol2l_deploy.operator import OperatorConsole
from robol2l_deploy.physical import authorize
from robol2l_deploy.runtime import OutcomeUnknown, StationRuntime


class FakeDriver:
    backend='franky';planner='software_fixture';speed_factor=.05;max_speed_factor=1.
    def __init__(self):
        q=np.array([0.,-.3,0.,-1.8,0.,1.5,.7])
        self.raw=dict(q=q.tolist(),q_d=q.tolist(),dq=[0.]*7,T_base_ee=Kinematics().forward(q).tolist(),
            has_errors=False,mode_name='idle',timestamp_s=time.time(),gripper=dict(is_grasped=False,width_m=.08))
        self.commands=[];self.next_error=None;self.grasp_confirmed=True
    def read(self):return copy.deepcopy(self.raw)
    def plan(self,waypoints,**kwargs):
        self.path=np.asarray(waypoints)
        return dict(joint_positions=self.path,dt_s=.01,backend=self.planner,token='fake',duration_s=1.)
    def prepare_samples(self,positions,dt_s):
        self.path=np.asarray(positions)
        return dict(joint_positions=self.path,dt_s=dt_s,backend=self.planner,token="sampled",duration_s=max(1,len(self.path)-1)*dt_s)
    def execute(self,token):
        self.commands.append(('trajectory',token))
        if self.next_error:raise self.next_error
        self.raw.update(q=self.path[-1].tolist(),q_d=self.path[-1].tolist(),
            T_base_ee=Kinematics().forward(self.path[-1]).tolist())
        return dict(self.read(),ok=True,elapsed_s=1.,source='physical_FR3_franky',
            controller_report=dict(completed=True))
    def move_gripper(self,width):
        self.commands.append(('gripper',width))
        if self.next_error:raise self.next_error
        self.raw['gripper']['width_m']=width
        return dict(self.raw['gripper'],ok=True)
    def grasp(self,*,force):
        self.commands.append(('grasp',force))
        self.raw['gripper']=dict(width_m=.04 if self.grasp_confirmed else 0.,is_grasped=self.grasp_confirmed)
        return dict(self.raw['gripper'],ok=self.grasp_confirmed)
    def realtime_ready(self):return True
    def close(self):pass


class SoftwareFR3(FR3Adapter):
    synthetic=True
    def describe(self):return dict(super().describe(),synthetic=True,physical_evidence=False)


class ExecutionOnlyHarnessTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.driver=FakeDriver()
        self.sdk=SimpleNamespace(cfg=config(),run_dir=Path(self.temp.name),close=lambda:None)
        self.adapter=SoftwareFR3(self.sdk,cameras={'unavailable_camera':Mock()},driver=self.driver)
        self.runtime=StationRuntime(self.adapter,Path(self.temp.name)/'runtime',allow_motion=True)
        self.provider=Mock()
        self.api=PublicAPI(self.runtime,MockReachTask.context,providers={'segmentation':self.provider})
        self.no_camera=patch.object(self.adapter,'get_observation',side_effect=AssertionError('implicit camera call'))
        self.no_camera.start()
    def tearDown(self):
        self.no_camera.stop();self.runtime.close();self.temp.cleanup()
    def plan(self):
        pose=self.adapter.get_robot_state().end_effector_poses['primary']
        return self.api.tools()['plan_motion'](Pose(pose.position+[0,0,.02],pose.quaternion_wxyz,'base'))

    def test_planning_and_motion_do_not_use_cameras_models_or_held_object_inference(self):
        with patch('franka_cap.planning.CollisionScene.__init__',side_effect=AssertionError('collision model constructed')):
            plan=self.plan()
        self.assertTrue(plan.ok,plan.error)
        self.assertFalse(plan.trajectory.collision_aware)
        self.assertFalse(plan.diagnostics['collision_checked'])
        # This change used to invalidate a public plan and the held-object state.
        self.driver.raw['gripper'].update(is_grasped=True,width_m=.01)
        result=self.api.tools()['execute_trajectory'](plan.trajectory)
        self.assertTrue(result.ok,result.error)
        self.assertTrue(result.diagnostics['native_receipt']['gripper']['is_grasped'])
        self.assertFalse(self.runtime.faulted)
        self.provider.segment_text.assert_not_called()

    def test_fr3_joint_goals_direct_curves_and_timing_reach_the_same_driver(self):
        from dataclasses import replace
        state=self.runtime.get_robot_state();q=state.joint_positions['primary'].copy();q[0]+=.02
        tools=self.api.tools();plan=tools['plan_motion'](q,start_joints=state.joint_positions['primary'])
        self.assertTrue(plan.ok,plan.error)
        changed=replace(plan.trajectory,dt_s=.7,joint_positions=np.stack([state.joint_positions['primary'],q]))
        for trajectory in (plan.trajectory,changed,changed):
            result=tools['execute_trajectory'](trajectory)
            self.assertTrue(result.ok,result.error)
        self.assertEqual(self.driver.commands,[('trajectory','fake'),('trajectory','sampled'),('trajectory','sampled')])
        np.testing.assert_allclose(self.driver.path[-1],q)
        self.assertTrue(tools['go_home']().ok)
        self.assertTrue(tools['set_grippers']({'primary':.25}).ok)

    def test_gripper_dispatches_without_scene_or_target_and_preserves_native_feedback(self):
        self.driver.raw['gripper']['is_grasped']=True
        result=self.api.tools()['set_gripper'](.25)
        self.assertTrue(result.ok,result.error)
        self.assertTrue(result.diagnostics['native_receipt']['is_grasped'])
        result=self.api.tools()['close_gripper']()
        self.assertTrue(result.ok,result.error)
        self.assertEqual(self.driver.commands,[('gripper',.02),('grasp',5.)])
        self.provider.segment_text.assert_not_called()

    def test_unsuccessful_native_grasp_is_recoverable_and_does_not_block_next_command(self):
        self.driver.grasp_confirmed=False
        result=self.api.tools()['close_gripper']()
        self.assertFalse(result.ok);self.assertEqual(result.error.code.value,'grasp_failed')
        self.assertFalse(self.runtime.faulted)
        self.assertTrue(self.api.tools()['open_gripper']().ok)
        self.assertEqual(len(self.driver.commands),2)

    def test_authorization_does_not_require_empty_gripper(self):
        self.driver.raw['gripper']['is_grasped']=True
        with patch.object(self.adapter,'synthetic',False):
            policy=authorize(self.runtime,'task','Software fixture only')
        self.assertTrue(policy.permits('task'))
        self.assertEqual(self.driver.commands,[])

    def test_real_controller_error_stops_following_dispatch(self):
        plan=self.plan()
        self.driver.next_error=OutcomeUnknown('fixture',diagnostics=dict(cause_type='ControlException',cause_message='cartesian_reflex'))
        result=self.api.tools()['execute_trajectory'](plan.trajectory)
        self.assertEqual(result.error.code.value,'controller_failed')
        self.assertTrue(self.runtime.faulted)
        self.assertFalse(self.api.tools()['open_gripper']().ok)
        self.assertEqual(len(self.driver.commands),1)

    def test_unknown_gripper_outcome_remains_distinct_through_public_api(self):
        self.driver.next_error=OutcomeUnknown('fixture',diagnostics=dict(cause_type='NetworkException',cause_message='connection lost'))
        result=self.api.tools()['open_gripper']()
        self.assertEqual(result.error.code.value,'outcome_unknown')
        self.assertEqual(result.error.details['cause_type'],'NetworkException')
        self.assertTrue(self.runtime.faulted)


class OperatorAndImagesTests(unittest.TestCase):
    def test_manual_actions_run_when_cameras_are_unavailable(self):
        with tempfile.TemporaryDirectory() as folder:
            adapter=MockAdapter();runtime=StationRuntime(adapter,folder,allow_motion=True)
            try:
                operator=OperatorConsole(runtime,SimpleNamespace(thread=None),providers={})
                with patch.object(adapter,'get_observation',side_effect=AssertionError('implicit camera call')):
                    for key,action,args in [('move','move',{'delta_m':[0,0,.02]}),('hand','gripper',{'position':.5})]:
                        operator.manual_action(key,action,args)
                        deadline=time.monotonic()+5
                        while runtime.jobs[key]['status'] in ('accepted','running') and time.monotonic()<deadline:time.sleep(.01)
                        self.assertEqual(runtime.jobs[key]['status'],'completed',runtime.jobs[key])
                self.assertEqual(adapter.executions,2)
            finally:runtime.close()

    def test_model_message_contains_actual_camera_pixels_and_tool_feedback(self):
        with tempfile.TemporaryDirectory() as folder:
            runtime=StationRuntime(MockAdapter(),folder)
            try:
                api=PublicAPI(runtime,MockReachTask.context,providers={});obs=api.get_observation()
                request=messages(api,obs,[{'feedback':{'stdout':'checkpoint','calls':[{'tool':'get_robot_state','line':2}]}}],{})
                content=request[1]['content'];images=[p for p in content if p['type']=='image_url']
                self.assertEqual(len(images),len(obs.cameras))
                for part,camera in zip(images,obs.cameras.values()):
                    data=base64.b64decode(part['image_url']['url'].split(',',1)[1])
                    image=cv2.cvtColor(cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR),cv2.COLOR_BGR2RGB)
                    self.assertEqual(image.shape,camera.rgb.shape)
                    self.assertGreater(image[60,80,1],150)
                context=json.loads(content[0]['text'])
                self.assertEqual(context['history'][0]['feedback']['calls'][0]['line'],2)
                self.assertNotIn('secret_oracle',json.dumps(context))
            finally:runtime.close()


if __name__=='__main__':unittest.main()
