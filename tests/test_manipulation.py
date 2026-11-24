import copy
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.contracts import Pose, TaskContext
from robol2l_deploy.manipulation import Box, Scene
from robol2l_deploy.runtime import StationRuntime, OutcomeUnknown
from robol2l_deploy.cap.program import ProgramExecutor


def scene_for(adapter, pose, *, reviewed=True):
    return Scene((Box('object', pose, (.04,.04,.04), True),), (0.,0.,0.), (0.,0.,1.),
                 'software-observation', adapter.device_binding(), time.time(), reviewed)


class ManipulationTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.a=MockAdapter(6, ('overhead',))
        self.r=StationRuntime(self.a, self.tmp.name, allow_motion=True)
        self.r.install_scene(scene_for(self.a,self.a.pose))
    def tearDown(self):
        self.r.close();self.tmp.cleanup()
    def test_cap_composes_grasp_transfer_release(self):
        context=TaskContext('mock',0,'pick-place','Synthetic pick and place.','mock',{})
        program='''
grasp = close_gripper()
assert grasp.ok
target = Pose([0.1, 0.0, 0.4], [1, 0, 0, 0], "base")
plan = plan_motion(target)
assert plan.ok
motion = execute_trajectory(plan.trajectory)
assert motion.ok
released = open_gripper()
assert released.ok
result = {"completed": True}
'''
        report=ProgramExecutor(self.r,context).execute(program,Path(self.tmp.name)/'program')
        self.assertTrue(report['ok'],report)
        self.assertFalse(self.a.manipulation.holding)
        np.testing.assert_allclose(self.a.manipulation.scene.boxes[0].pose.position,[.1,0,.4])
        self.assertEqual(self.a.executions,3)
        self.assertEqual(self.a.gripper,1.)
        self.assertFalse(report['physical_task_success'])
    def test_close_requires_object_and_motion_permission(self):
        self.r.allow_motion=False
        self.assertFalse(self.r.close_gripper().ok);self.assertEqual(self.a.executions,0)
        self.r.allow_motion=True;self.a.manipulation.scene=None
        self.assertFalse(self.r.close_gripper().ok);self.assertEqual(self.a.executions,0)
    def test_registered_target_has_no_tcp_center_distance_gate(self):
        self.a.pose=Pose([.025,0,.325],[1,0,0,0],'base')
        self.assertTrue(self.r.close_gripper().ok)
        self.assertTrue(self.a.manipulation.holding)
    def test_grasp_changes_binding_and_drop_blocks_lift(self):
        before=self.r.plan_motion(self.a.pose)
        self.assertTrue(self.r.close_gripper().ok)
        self.assertFalse(self.r.execute_trajectory(before.trajectory).ok)
        plan=self.r.plan_motion(Pose([0,0,.4],[1,0,0,0],'base'))
        self.a.is_grasped=False
        result=self.r.execute_trajectory(plan.trajectory)
        self.assertEqual(result.error.message,'grasp_lost');self.assertEqual(self.a.executions,1)
    def test_scene_change_invalidates_plan_and_cannot_replace_held_scene(self):
        plan=self.r.plan_motion(self.a.pose)
        self.r.install_scene(scene_for(self.a,self.a.pose))
        self.assertFalse(self.r.execute_trajectory(plan.trajectory).ok)
        self.assertTrue(self.r.close_gripper().ok)
        with self.assertRaisesRegex(ValueError,'while_holding'):
            self.r.install_scene(scene_for(self.a,self.a.pose))
    def test_unknown_attachment_on_restart_refuses_motion(self):
        self.a.is_grasped=True
        plan=self.r.plan_motion(self.a.pose)
        result=self.r.execute_trajectory(plan.trajectory)
        self.assertEqual(result.error.message,'untracked_held_object')
        self.assertEqual(self.a.executions,0)


class FR3ReceiptTests(unittest.TestCase):
    def setUp(self):
        from franka_cap.common import config
        from franka_cap.planning import Kinematics
        from franka_cap.geometry import pose_dict
        from robol2l_deploy.adapters.fr3 import FR3Adapter
        self.tmp=tempfile.TemporaryDirectory();cfg=config()
        q=[0,-.3,0,-1.8,0,1.5,.7];ee=Kinematics().forward(q)
        self.raw=dict(q=q,q_d=q,dq=[0]*7,T_base_ee=ee.tolist(),has_errors=False,
                      mode_name='idle',timestamp_s=time.time(),gripper=dict(width_m=.08,is_grasped=False))
        sdk=SimpleNamespace(cfg=cfg,run_dir=Path(self.tmp.name),get_robot_state=lambda:copy.deepcopy(self.raw),close=lambda:None)
        from unittest.mock import Mock
        driver=SimpleNamespace(read=sdk.get_robot_state,close=lambda:None,grasp=Mock(),move_gripper=Mock(),execute=Mock())
        self.a=FR3Adapter(sdk,cameras={},driver=driver);self.a.get_robot_state()
        p=pose_dict(ee);self.a.install_scene(scene_for(self.a,Pose(p['position'],p['quaternion_wxyz'],'base')))
        self.r=StationRuntime(self.a,Path(self.tmp.name)/'runtime',allow_motion=True)
    def tearDown(self):self.r.close();self.tmp.cleanup()
    def test_library_grasp_and_release_use_same_adapter(self):
        receipts=[dict(ok=True,width_m=.04,is_grasped=True,stopped=False),
                  dict(ok=True,width_m=.08,is_grasped=False,stopped=False)]
        with patch.object(self.a,'check_finger_sweep',return_value=None),patch.object(self.a.driver,'grasp',return_value=receipts[0]) as grasp,patch.object(self.a.driver,'move_gripper',return_value=receipts[1]) as move:
            self.assertTrue(self.r.close_gripper().ok)
            self.assertAlmostEqual(grasp.call_args.args[0],.04)
            self.raw['gripper']=dict(width_m=.04,is_grasped=True)
            self.assertTrue(self.r.open_gripper().ok)
            move.assert_called_once_with(.08)
        self.assertFalse(self.a.manipulation.holding)
    def test_negative_grasp_receipt_is_known_failure(self):
        receipt=dict(ok=False,width_m=.04,is_grasped=False,stopped=False)
        with patch.object(self.a,'check_finger_sweep',return_value=None),patch.object(self.a.driver,'grasp',return_value=receipt):
            result=self.r.close_gripper()
        self.assertFalse(result.ok);self.assertFalse(self.r.faulted)
        self.assertFalse(self.a.manipulation.holding)
        self.assertEqual(result.error.message,'grasp_not_confirmed')
    def test_offset_target_still_passes_through_collision_check(self):
        from dataclasses import replace
        from robol2l_deploy.errors import ApiError,ErrorCode
        scene=self.a.manipulation.scene;box=scene.boxes[0]
        shifted=replace(box,pose=Pose(box.pose.position+[.025,0,.025],box.pose.quaternion_wxyz,'base'))
        self.a.manipulation.scene=replace(scene,boxes=(shifted,))
        with patch.object(self.a,'check_finger_sweep',side_effect=ApiError(ErrorCode.SAFETY_INTERLOCK,'gripper_sweep_collision')) as collision:
            result=self.r.close_gripper()
        self.assertFalse(result.ok);self.assertEqual(result.error.message,'gripper_sweep_collision')
        collision.assert_called_once_with(0.,box.name)
        self.a.driver.grasp.assert_not_called()
        self.assertFalse(self.r.faulted)
    def test_explicit_continuation_accepts_tracked_held_object(self):
        from robol2l_deploy.physical import authorize
        self.a.driver.realtime_ready=lambda:True
        self.a.manipulation.attach(self.a.manipulation.scene.boxes[0],np.array(self.raw['T_base_ee']),.04)
        self.raw['gripper']=dict(width_m=.04,is_grasped=True)
        with self.assertRaisesRegex(ValueError,'empty_start'):
            authorize(self.r,'test','Mock fresh start')
        policy=authorize(self.r,'test','Mock continuation',continuing=True)
        self.assertTrue(policy.active)
        self.a.driver.grasp.assert_not_called();self.a.driver.move_gripper.assert_not_called()
    def test_incomplete_motion_receipt_remains_unknown(self):
        q=np.array(self.raw['q']);target=q.copy();target[0]+=.01
        receipt=dict(ok=False,q=q.tolist(),width_m=.08,is_grasped=False)
        with patch.object(self.a.driver,'execute',return_value=receipt):
            with self.assertRaises(OutcomeUnknown):
                self.a.execute({'token':'test'},SimpleNamespace(joint_positions=np.array([q,target])))
    def test_missing_receipt_stays_unknown(self):
        with patch.object(self.a,'check_finger_sweep',return_value=None),patch.object(self.a.driver,'grasp',side_effect=TimeoutError):
            with self.assertRaises(OutcomeUnknown):self.r.close_gripper()
        self.assertTrue(self.r.faulted)
    def test_native_nonzero_keeps_explicit_failed_grasp(self):
        from franka_cap.devices import native_receipt
        receipt=dict(ok=False,width_m=.04,is_grasped=False,stopped=True)
        with patch('franka_cap.devices.subprocess.run',return_value=SimpleNamespace(returncode=1,stdout=json.dumps(receipt),stderr='')):
            self.assertEqual(native_receipt('franka_cap_gripper','unused','grasp',.04,5),receipt)
    def test_held_motion_uses_holding_mode_and_detects_lost_grasp(self):
        box=self.a.manipulation.scene.boxes[0]
        self.a.manipulation.attach(box,np.array(self.raw['T_base_ee']),.04)
        self.raw['gripper']=dict(width_m=.04,is_grasped=True)
        plan=dict(token='checked-library-trajectory',holding_object=True,start_robot_state=self.raw,target_pose=dict(position=np.asarray(self.raw['T_base_ee'])[:3,3]),
                  duration_s=20,q_samples=[self.raw['q']]*101)
        trajectory=SimpleNamespace(joint_positions=np.array([self.raw['q']]*11))
        receipt=dict(q=self.raw['q'],width_m=.04,is_grasped=False,elapsed_s=20)
        with patch.object(self.a.driver,'execute',return_value=receipt) as native:
            result=self.a.execute(plan,trajectory)
        native.assert_called_once_with('checked-library-trajectory')
        self.assertFalse(result.ok);self.assertEqual(result.error.message,'grasp_lost')
    def test_calibration_change_invalidates_observed_geometry(self):
        with patch.object(self.a,'device_binding',return_value='changed-calibration'):
            self.assertFalse(self.a.scene_reviewed())
            self.assertEqual(self.a.check_close_gripper(),'scene_not_reviewed')


class SceneGeometryTests(unittest.TestCase):
    def synthetic_camera(self):
        # Synthetic metric geometry with a cube top and a shallow receiving dish.
        import cv2
        y,x=np.indices((320,320));points=np.stack((x*.001,y*.001,np.zeros_like(x)),axis=-1).astype(float)
        rgb=np.full((320,320,3),90,np.uint8)
        rgb[80:130,90:140]=[0,200,0];points[80:130,90:140,2]=.05
        radius=np.hypot(x-230,y-190);mask=radius<70
        rgb[mask]=[220,220,220];points[mask,2]=.004;points[(radius>55)&mask,2]=.022
        meta={'timestamp_s':time.time()}
        return meta,rgb,points,np.ones((320,320),bool)
    def test_observed_cube_and_plate_remain_unreviewed_geometry(self):
        from robol2l_deploy.tasks.green_cube import build_scene
        regions=dict(table=[[5,5,60,60]],cube=[80,70,80,80],plate=[159,119,142,142])
        with patch('robol2l_deploy.tasks.green_cube.load_camera',return_value=self.synthetic_camera()):
            scene,metrics,_,_=build_scene('observation-synthetic','overhead',regions,'test')
        self.assertFalse(scene.reviewed);self.assertEqual(scene.observation_id,'synthetic')
        np.testing.assert_allclose(scene.boxes[0].pose.position,[.1145,.1045,.025],atol=.001)
        self.assertAlmostEqual(metrics['cube_height_m'],.05)
        self.assertAlmostEqual(metrics['plate_center_m'][2],.004)
    def test_truncated_cube_is_rejected(self):
        from robol2l_deploy.tasks.green_cube import build_scene
        regions=dict(table=[[5,5,60,60]],cube=[90,80,30,30],plate=[159,119,142,142])
        with patch('robol2l_deploy.tasks.green_cube.load_camera',return_value=self.synthetic_camera()):
            with self.assertRaisesRegex(ValueError,'truncated'):build_scene('observation-synthetic','overhead',regions,'test')
    def test_placement_carries_tcp_offset_to_avoid_plate_penetration(self):
        from robol2l_deploy.manipulation import Manipulation
        from robol2l_deploy.tasks.prepare_pick_place import candidate_operations
        adapter=MockAdapter();scene=scene_for(adapter,Pose([.5,0,.03],[1,0,0,0],'base'))
        operations=candidate_operations(scene,{'plate_center_m':[.5,.2,.006]},adapter.pose)
        poses={o['name']:o['pose'] for o in operations if o['kind']=='motion'}
        payload=Manipulation(scene);payload.attach(scene.boxes[0],poses['descend_to_grasp'].as_matrix(),.04)
        placed=poses['lower_inside_plate'].as_matrix()@payload.ee_to_object
        corners=scene.boxes[0].corners(placed)
        self.assertAlmostEqual(corners[:,2].min(),.006+.004)
        adapter.close()
