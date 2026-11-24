import json
from pathlib import Path
import tempfile
import unittest
import time
from unittest.mock import patch,MagicMock
import numpy as np
from franka_cap.geometry import pose_dict,pose_matrix,shifted,matrix
from franka_cap.program import Program
from franka_cap.planning import Kinematics,CollisionScene,plan_path
from franka_cap.common import ROOT,config,read_json
from franka_cap import devices
from franka_cap.perception import localize_pixel
from franka_cap.sdk import CapSDK

class GeometryTests(unittest.TestCase):
    def test_pose_contract(self):
        pose=pose_dict(np.eye(4));pose=shifted(pose,[.01,0,0]);self.assertAlmostEqual(pose_matrix(pose)[0,3],.01)
        for bad in [dict(pose,frame='world'),dict(pose,quaternion_wxyz=[0,0,0,0]),dict(pose,position=[True,0,0]),dict(pose,position=[float('nan'),0,0])]:
            with self.assertRaises(ValueError):pose_matrix(bad)
    def test_reflection_rejected(self):
        bad=np.eye(4);bad[0,0]=-1
        with self.assertRaises(ValueError):matrix(bad)
    def test_ik_and_model(self):
        model=Kinematics();q=np.array([0,-.3,0,-1.8,0,1.5,.7]);start=model.forward(q)
        goal=start.copy();goal[2,3]+=.005
        solved=model.solve(goal,q)
        self.assertLess(np.linalg.norm(model.forward(solved)[:3,3]-goal[:3,3]),.0001)
        with self.assertRaises(ValueError):model.solve(goal,[99]*7)

class ProgramTests(unittest.TestCase):
    def test_complete_validation_before_first_call(self):
        api=MagicMock()
        for source in ['def run(api):\n api.get_robot_state()\n import os',
                       'def run(api):\n api.get_robot_state()\n api.__dict__',
                       'def run(api):\n while True: pass',
                       'def run(api):\n return open("x")',
                       'def run(api):\n return {**api.describe()}',
                       'def run(api):\n def other(): pass']:
            with self.assertRaises(ValueError):Program(api).run(source)
        api.get_robot_state.assert_not_called()
    def test_program_branch_loop_return(self):
        api=MagicMock();api.get_robot_state.return_value={'q':[1,2,3]}
        result=Program(api).run('def run(api):\n s=api.get_robot_state()\n x=0\n for i in range(3):\n  x=x+1\n if x==3:\n  return s["q"][0]+x\n return 0')
        self.assertEqual(result['returned'],4);self.assertFalse(result['executed_motion'])
    def test_call_budget(self):
        with self.assertRaisesRegex(ValueError,'call budget'):
            Program(MagicMock(),max_calls=1).run('def run(api):\n for i in range(2):\n  api.get_robot_state()')
    def test_zero_statement_budget_bypass(self):
        with self.assertRaises(ValueError):Program(MagicMock()).run('def run(api):\n for i in range(100000000):\n  pass')

class PerceptionTests(unittest.TestCase):
    def test_depth_units_and_transform(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'d.npy';np.save(path,np.full((5,5),.5,np.float32));T=np.eye(4);T[0,3]=1
            cam={'depth_available':True,'width':5,'height':5,'depth_path':str(path),'model':'distortion.none',
                 'fx':100,'fy':100,'ppx':2,'ppy':2,'coeffs':[0]*5,'T_base_camera':T.tolist()}
            self.assertEqual(localize_pixel(cam,[2,2])['point_base_m'],[1,0,.5])
            inverse=dict(cam,model='distortion.inverse_brown_conrady')
            self.assertEqual(localize_pixel(inverse,[2,2])['point_base_m'],[1,0,.5])
            with self.assertRaises(ValueError):localize_pixel(dict(inverse,coeffs=[.1,0,0,0,0]),[2,2])
            with self.assertRaises(ValueError):localize_pixel(dict(cam,depth_available=False),[2,2])
            np.save(path,np.zeros((5,5),np.float32))
            with self.assertRaises(ValueError):localize_pixel(cam,[2,2])
    def test_gemini_brown_k6_deprojection(self):
        import cv2
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'d.npy';np.save(path,np.full((800,1280),.7,np.float32))
            cam={'depth_available':True,'width':1280,'height':800,'depth_path':str(path),
                 'model':'distortion.brown_conrady','fx':612.,'fy':612.,'ppx':638.,'ppy':406.,
                 'coeffs':[-.03,.036,0,0,-.012,0,0,0],'T_base_camera':np.eye(4).tolist()}
            pixel=[900,250];point=localize_pixel(cam,pixel)['point_camera_m']
            k=np.array([[612.,0,638.],[0,612.,406.],[0,0,1.]])
            projected,_=cv2.projectPoints(np.array([point]),np.zeros(3),np.zeros(3),k,np.array(cam['coeffs']))
            np.testing.assert_allclose(projected.reshape(2),pixel,atol=1e-4)


class DeviceTests(unittest.TestCase):
    def test_global_depth_scale_and_invalid_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp)
            meta={'width':2,'height':2,'depth_available':True,'alignment':'depth_to_color','depth_scale_m':.0001}
            (directory/'camera.json').write_text(json.dumps(meta))
            raw=np.array([[0,10000],[5000,20000]],np.uint16)
            raw.tofile(directory/'depth_u16.raw')
            with patch('franka_cap.devices.native'):
                devices.global_capture({'global_serial':'test'},directory)
                np.testing.assert_allclose(np.load(directory/'depth_m.npy'),[[0,1],[.5,2]])
                raw[:]=0;raw.tofile(directory/'depth_u16.raw')
                with self.assertRaisesRegex(ValueError,'no valid distances'):
                    devices.global_capture({'global_serial':'test'},directory)
                raw[:1].tofile(directory/'depth_u16.raw')
                with self.assertRaisesRegex(ValueError,'incorrect size'):
                    devices.global_capture({'global_serial':'test'},directory)
                meta['depth_scale_m']=float('nan')
                (directory/'camera.json').write_text(json.dumps(meta))
                with self.assertRaisesRegex(ValueError,'scale'):
                    devices.global_capture({'global_serial':'test'},directory)

    def test_calibration_serial_and_ee_fail_closed(self):
        cfg=config();cal=read_json(cfg['wrist_calibration']);state={'F_T_EE':cal['F_T_EE'],'T_base_ee':np.eye(4).tolist()}
        _,T,_=devices.calibration(cfg,'wrist',state);self.assertTrue(np.isfinite(T).all())
        with self.assertRaises(ValueError):devices.calibration(dict(cfg,wrist_serial='wrong'),'wrist',state)
        state['F_T_EE']=np.eye(4).tolist()
        with self.assertRaises(ValueError):devices.calibration(cfg,'wrist',state)
    def test_no_motion_even_with_valid_commands(self):
        with tempfile.TemporaryDirectory() as tmp:
            sdk=CapSDK(run_dir=tmp)
            with patch('franka_cap.devices.native') as native:
                with self.assertRaisesRegex(RuntimeError,'motion_not_commissioned'):sdk.move_to_pose(pose_dict(np.eye(4)))
                with self.assertRaisesRegex(RuntimeError,'motion_not_commissioned'):sdk.set_gripper(1)
                native.assert_not_called()
    def test_stale_observation_and_calibration_change(self):
        with tempfile.TemporaryDirectory() as tmp:
            sdk=CapSDK(run_dir=tmp);sdk.observations['old']={'host_monotonic_s':-100,'cameras':{}}
            with self.assertRaisesRegex(ValueError,'stale'):sdk._camera('old','wrist')
            sdk.observations['lag']={'host_monotonic_s':time.monotonic(),'cameras':{'wrist':{'host_monotonic_s':time.monotonic()-60}}}
            with self.assertRaisesRegex(ValueError,'frame stale'):sdk._camera('lag','wrist')
    def test_model_collision_detects_obstacle(self):
        cfg=config();cal=read_json(cfg['wrist_calibration'])
        scene=CollisionScene(cal,[{'name':'blocked','center_m':[0,0,.4],'half_extents_m':[2,2,2]}])
        try:
            scene.set_state([0,-.3,0,-1.8,0,1.5,.7],.08)
            self.assertTrue(any('blocked' in h['links'] for h in scene.violations()))
        finally:scene.close()
    def test_out_of_bound_target_before_collision(self):
        cfg=config();kin=Kinematics();q=[0,-.3,0,-1.8,0,1.5,.7];T=kin.forward(q)
        state={'q':q,'dq':[0]*7,'has_errors':False,'T_base_ee':T.tolist(),'gripper':{'width_m':.08}}
        goal=shifted(pose_dict(T),[.5,0,0])
        with patch('franka_cap.planning.CollisionScene') as scene:
            with self.assertRaisesRegex(ValueError,'segment'):plan_path(state,goal,cfg,{})
            scene.assert_not_called()

if __name__=='__main__':unittest.main()
