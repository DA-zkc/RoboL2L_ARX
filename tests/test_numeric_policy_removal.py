"""Software-only regressions for removing deployment policy thresholds."""
import tempfile
import time
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.adapters.drivers import resolve_speed_factor
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cap.physical_task import PhysicalPickPlaceTask
from robol2l_deploy.cap.session import CapSession
from robol2l_deploy.cap.program import ProgramExecutor
from robol2l_deploy.cap.tasks import MockReachTask
from robol2l_deploy.contracts import Pose, Segmentation
from robol2l_deploy.manipulation import Box, Manipulation, Scene
from robol2l_deploy.physical import validate_scene
from robol2l_deploy.runtime import StationRuntime
from robol2l_deploy.errors import ApiError,ErrorCode
from franka_cap.planning import CollisionPrediction


class NumericPolicyRemovalTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.adapter=MockAdapter()
        self.runtime=StationRuntime(self.adapter,self.temp.name,allow_motion=True)

    def tearDown(self):
        self.runtime.close();self.temp.cleanup()

    def test_old_plan_changed_start_and_explicit_repeat_execute(self):
        plan=self.runtime.plan_motion(Pose([0,0,.4],[1,0,0,0],'base'))
        self.runtime.plans[plan.diagnostics['plan_id']]['created']-=3600
        self.adapter.q[0]=.2;self.adapter.gripper=.5
        state=self.adapter.get_robot_state()
        state=replace(state,joint_velocities={'primary':np.full(7,.02)})
        with patch.object(self.adapter,'get_robot_state',return_value=state):
            self.assertTrue(self.runtime.execute_trajectory(plan.trajectory).ok)
        self.assertEqual(self.adapter.executions,1)
        self.assertTrue(self.runtime.execute_trajectory(plan.trajectory).ok)
        self.assertEqual(self.adapter.executions,2)

    def test_collision_remains_a_planning_result_and_never_dispatches(self):
        error=CollisionPrediction([dict(links=['arm','object'],distance_m=-.001)],stage='cartesian_path')
        with patch.object(self.adapter,'plan_motion',side_effect=error):
            result=self.runtime.plan_motion(self.adapter.pose)
        self.assertFalse(result.ok);self.assertFalse(self.runtime.faulted)
        self.assertEqual(self.adapter.executions,0)
        self.assertEqual(result.error.details['source'],'geometric_prediction')

    def test_scene_provenance_is_recorded_without_project_gate(self):
        scene=Scene((),(0,0,0),(0,0,1),'observation','binding',time.time()-3600)
        evidence=dict(source='full_rgbd',manual_regions=False,support_inliers=3)
        self.assertTrue(validate_scene(scene,evidence).reviewed)
        self.assertTrue(validate_scene(scene,dict(evidence,manual_regions=True)).reviewed)

    def test_held_width_change_is_feedback_but_lost_grasp_remains(self):
        box=Box('target',self.adapter.pose,(.04,.04,.04),True)
        m=Manipulation();m.attach(box,self.adapter.pose.as_matrix(),.04)
        self.assertIsNone(m.telemetry_error(True,.055))
        self.assertEqual(m.telemetry_error(False,.055),'grasp_lost')

    def test_sparse_far_depth_and_low_score_are_not_projection_gates(self):
        env=PublicAPI(self.runtime,MockReachTask().context,providers={}).environment
        obs=env.get_observation();cam=obs.cameras['global']
        depth=np.full(cam.depth_m.shape,4.)
        camera=replace(cam,depth_m=depth)
        observation=replace(obs,cameras=dict(obs.cameras,global_=camera))
        mask=np.zeros(depth.shape,bool);mask[1,1]=True
        segmentation=Segmentation(mask,'object',.01,'global_',camera.frame)
        with patch.object(self.adapter,'synthetic',False):
            self.assertEqual(env.prepare_projection(segmentation,observation).mask.sum(),1)

    def test_program_source_has_no_project_size_cap(self):
        result=ProgramExecutor(self.runtime,MockReachTask().context).execute('#'+('x'*33000)+'\nresult=42',Path(self.temp.name)/'program')
        self.assertTrue(result['ok'],result)
        self.assertEqual(result['result'],42)

    def test_agent_finish_is_not_numeric_scoring_or_physical_verification(self):
        class Model:
            label='software-fixture'
            def __init__(self):self.turn=0
            def decide(self,messages):
                self.turn+=1
                return dict(kind='program',source='result=True',reason='software only') if self.turn==1 else dict(kind='finish',reason='model report')
        session=CapSession(self.runtime,PhysicalPickPlaceTask(),Model(),Path(self.temp.name)/'cap')
        result=session.run()
        self.assertEqual(result['status'],'completed')
        self.assertFalse(result['physical_task_success'])
        self.assertFalse(result['verification']['required'])

    def test_speed_has_native_domain_without_extra_point_one_cap(self):
        self.assertEqual(resolve_speed_factor(.5),.5)
        self.assertEqual(resolve_speed_factor(None),.05)
        for bad in (0,-1,1.1,float('nan')):
            with self.assertRaises(ApiError):resolve_speed_factor(bad)


if __name__=='__main__':unittest.main()
