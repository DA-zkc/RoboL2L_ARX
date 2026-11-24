"""Software-only public API parity. Fixture perception is not model/hardware evidence."""
import ast
import inspect
import os
import json
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from cap_harness.api import CapApi
from cap_harness.registry import SHARED_PUBLIC_TOOL_NAMES
from robol2l_deploy import contracts as c
from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cap.program import ProgramExecutor
from robol2l_deploy.cap.tasks import MockReachTask
from robol2l_deploy.operator import OperatorConsole
from robol2l_deploy.runtime import StationRuntime
from robol2l_deploy.errors import ApiError, ErrorCode


class FixturePerception:
    def segment_text(self,camera,query):
        mask=np.ones(camera.depth_m.shape,bool)
        return c.SegmentationSet(True,(c.Segmentation(mask,query,.6,'global',camera.frame),
                                       c.Segmentation(mask,'second',.3,'global',camera.frame)))
    def segment_points(self,camera,points):return self.segment_text(camera,'fixture')
    def generate_grasps(self,cloud,**kwargs):
        return c.GraspSet(True,(c.GraspCandidate(c.Pose(np.mean(cloud.points,axis=0),[1,0,0,0],cloud.frame),.5,.04),))


class AspireParityTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.adapter=MockAdapter()
        self.runtime=StationRuntime(self.adapter,self.temp.name,allow_motion=True)
        self.providers={'segmentation':FixturePerception(),'grasps':{'contact-graspnet':FixturePerception()}}
        self.api=PublicAPI(self.runtime,MockReachTask.context,providers=self.providers)
        self.tools=self.api.tools()
    def tearDown(self):self.runtime.close();self.temp.cleanup()

    def test_all_upstream_public_tools_registered_with_additive_perception_keywords(self):
        upstream=Path(os.environ.get('ASPIRE_CHECKOUT', '/home/solution/AgenticRSI/ASPIRE')) / 'src/cap_harness/api.py'
        source=ast.parse(upstream.read_text());local=ast.parse(Path('cap_harness/api.py').read_text())
        def signatures(tree):
            cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CapApi')
            return {n.name:ast.dump(n.args,include_attributes=False) for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in SHARED_PUBLIC_TOOL_NAMES}
        # SAM3 input adaptation and explicit mask reuse add optional keywords;
        # all upstream positional/required arguments retain their contract.
        depth_options={'depth_range_m','seed_px','max_depth_jump_m'}
        extensions={'segment_text':{'model_query'},
                    'localize_object':{'model_query','segmentation'} | depth_options,
                    'mask_to_point_cloud':depth_options}
        for cls in local.body:
            if not isinstance(cls,ast.ClassDef) or cls.name!='CapApi':continue
            for method in cls.body:
                if not isinstance(method,ast.FunctionDef) or method.name not in extensions:continue
                pairs=[(arg,default) for arg,default in zip(method.args.kwonlyargs,method.args.kw_defaults)
                       if arg.arg not in extensions[method.name]]
                method.args.kwonlyargs=[pair[0] for pair in pairs]
                method.args.kw_defaults=[pair[1] for pair in pairs]
        self.assertEqual(signatures(source),signatures(local))
        self.assertEqual(set(self.tools),SHARED_PUBLIC_TOOL_NAMES)
        self.assertEqual(set(self.api.describe()['capabilities']['supported']),SHARED_PUBLIC_TOOL_NAMES)
        for name,fn in self.tools.items():
            expected=inspect.signature(getattr(CapApi,name)).replace(parameters=list(inspect.signature(getattr(CapApi,name)).parameters.values())[1:])
            self.assertEqual(inspect.signature(fn),expected)

    def test_all_tools_have_a_functional_mock_path(self):
        called=set()
        def call(name,*args,**kwargs):
            value=self.tools[name](*args,**kwargs);called.add(name)
            self.assertTrue(getattr(value,'ok',True),getattr(value,'error',None))
            return value
        call('get_task_context');state=call('get_robot_state');obs=call('get_observation')
        camera=obs.cameras['global'];yy,xx=np.indices(camera.depth_m.shape)
        camera=replace(camera,depth_m=.4+.04*np.sin(xx*.1)*np.cos(yy*.13))
        # The production geometry path processes this declared synthetic depth.
        self.api.environment.observation=replace(obs,cameras={**dict(obs.cameras),'global':camera})
        segmentation=call('segment_text','global','test fixture').segmentations[0]
        call('segment_points','global',[[30,40]])
        cloud=call('mask_to_point_cloud',segmentation.mask,'global','base')
        call('crop_point_cloud',cloud,[-2,-2,-2],[2,2,2]);call('estimate_geometry',cloud)
        call('localize_object','test fixture',camera_name='global',target_frame='base')
        grasps=call('generate_grasps','global',segmentation.mask)
        call('select_grasp',grasps)
        pose=state.end_effector_poses['primary'];call('solve_ik',pose)
        plan=call('plan_motion',pose);call('execute_trajectory',plan.trajectory)
        call('plan_synchronized_motion',{'primary':pose})
        call('move_to_pose',pose);call('move_to_joints',state.joint_positions['primary'])
        call('move_synchronized',{'primary':state.joint_positions['primary']})
        call('go_home');call('set_gripper',.4);call('set_grippers',{'primary':.6});call('open_gripper')
        # Native close may complete without an object; that is real feedback,
        # not a missing API. The mock reports GRASP_FAILED after dispatch.
        before=self.adapter.executions;out=self.tools['close_gripper']();called.add('close_gripper')
        self.assertEqual(self.adapter.executions,before+1);self.assertEqual(out.error.code.value,'grasp_failed')
        call('step',c.RobotAction({'primary':c.ArmCommand('joint_position',state.joint_positions['primary'],embodiment=state.embodiment)}))
        self.assertEqual(called,SHARED_PUBLIC_TOOL_NAMES)

    def test_constructed_changed_repeated_and_sync_trajectories_execute(self):
        state=self.runtime.get_robot_state();q=np.stack([state.joint_positions['primary'],np.full(7,.04)])
        trajectory=c.Trajectory(q,.7,state.joint_names['primary'],'agent_authored',False,state,embodiment=state.embodiment)
        self.assertTrue(self.tools['execute_trajectory'](trajectory).ok)
        self.assertTrue(self.tools['execute_trajectory'](trajectory).ok)
        plan=self.tools['plan_motion'](np.full(7,.02));self.adapter.generation+=1
        self.assertTrue(self.tools['execute_trajectory'](plan.trajectory).ok)
        sync=c.SynchronizedTrajectory({'primary':q},.7,state.joint_names,'agent_authored',False,state,embodiment=state.embodiment)
        self.assertTrue(self.tools['execute_trajectory'](sync).ok)
        self.assertEqual(self.adapter.executions,4)

    def test_gripper_columns_are_executed_and_hand_failure_stops_remaining_segments(self):
        state=self.runtime.get_robot_state();q=np.zeros((5,7));q[:,0]=np.arange(5)*.01
        trajectory=c.Trajectory(q,.6,state.joint_names['primary'],'authored',False,state,embodiment=state.embodiment,
            gripper_positions=[1,1,.5,.5,0])
        result=self.tools['execute_trajectory'](trajectory)
        self.assertTrue(result.ok,result.error);np.testing.assert_allclose(self.adapter.q,q[-1]);self.assertEqual(self.adapter.gripper,0)
        self.assertTrue(result.diagnostics['gripper_dwell_added'])
        with patch.object(self.adapter,'set_gripper',return_value=c.ExecutionResult(False,0,error=ApiError(ErrorCode.GRASP_FAILED,'fixture refusal'))):
            before=self.adapter.executions
            result=self.tools['execute_trajectory'](trajectory)
            self.assertFalse(result.ok);self.assertEqual(self.adapter.executions,before)

    def test_start_timing_home_and_native_conveniences_without_sample_count_gate(self):
        first=self.runtime.get_robot_state().joint_positions['primary'].copy()
        strategy=c.MotionStrategy(ik_solver='station',pose_planner='station',interpolation_dt_s=.03,maximum_trajectory_dt_s=.01)
        plan=self.tools['plan_motion'](np.full(7,.03),start_joints=np.full(7,.02),strategy=strategy)
        self.assertTrue(plan.ok,plan.error);np.testing.assert_allclose(plan.trajectory.joint_positions[0],.02)
        self.assertLessEqual(plan.trajectory.dt_s,.01)
        self.assertTrue(self.tools['move_to_joints'](np.full(7,.04),max_steps=1,tolerance=1e-12).ok)
        self.assertTrue(self.tools['go_home']().ok);np.testing.assert_array_equal(self.adapter.q,first)
        home=json.loads(self.runtime.home_path.read_text());self.assertEqual(home['source'],'first_measured_state_in_session')
        self.assertTrue(self.tools['move_to_pose'](c.Pose([0,0,.32],[1,0,0,0],'base'),max_steps=1).ok)
        self.assertTrue(self.tools['move_synchronized']({'primary':first},max_steps=1).ok)
        for method,args in [('plan_motion',(first,)),('plan_synchronized_motion',({'primary':first},))]:
            bad=self.tools[method](*args,strategy=c.MotionStrategy(pose_planner='curobo-integrated',trajectory_planner='curobo'))
            self.assertFalse(bad.ok);self.assertEqual(bad.error.code.value,'unsupported')

    def test_manual_retained_references_and_step_constructor_decode(self):
        operator=OperatorConsole(self.runtime,SimpleNamespace(thread=None),providers={})
        state=self.runtime.get_robot_state()
        operator.references['state']={'value':state,'observation_id':'old','operation':'get_robot_state'}
        decoded=operator.decode({'$type':'RobotAction','arms':{'primary':{'$type':'ArmCommand','mode':'joint_position',
            'target':{'$ref':'state','path':['joint_positions','primary']},'embodiment':state.embodiment}}})
        self.assertIsInstance(decoded,c.RobotAction)
        self.assertTrue(operator.tools['step'](decoded).ok)

    def test_isolated_program_can_use_new_tools_and_preserves_long_stdout(self):
        source='''import math
s=get_robot_state()
def move(q):
    return move_to_joints(q,max_steps=1)
r=move(s.joint_positions["primary"])
print("x"*17000)
result={"moved":r.ok,"home":go_home().ok,"hand":set_grippers({"primary":0.5}).ok}
'''
        result=ProgramExecutor(self.runtime,MockReachTask.context).execute(source,Path(self.temp.name)/'program')
        self.assertTrue(result['ok'],result['error']);self.assertTrue(all(result['result'].values()))
        self.assertEqual(len(result['stdout']),17001)

    def test_other_joint_count_and_camera_names_use_same_upper_layer(self):
        with tempfile.TemporaryDirectory() as folder:
            runtime=StationRuntime(MockAdapter(6,('overhead',)),folder,allow_motion=True)
            try:
                api=PublicAPI(runtime,MockReachTask.context,providers={});tools=api.tools()
                self.assertTrue(tools['move_to_joints'](np.full(6,.01)).ok)
                self.assertTrue(tools['move_synchronized']({'primary':np.zeros(6)}).ok)
                self.assertEqual(set(tools['get_observation']().cameras),{'overhead'})
                self.assertTrue(tools['go_home']().ok)
            finally:runtime.close()


if __name__=='__main__':unittest.main()
