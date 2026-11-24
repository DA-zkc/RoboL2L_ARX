import time
import unittest
from unittest.mock import patch,Mock
from types import SimpleNamespace
import numpy as np
from robol2l_deploy.physical import PhysicalSession
from robol2l_deploy.manipulation import Scene,Box,Manipulation
from robol2l_deploy.contracts import Pose


class PhysicalSessionTests(unittest.TestCase):
    def setUp(self):
        self.policy=PhysicalSession('task','Explicit test authorization; fixtures only')
        self.scene=Scene((),(0,0,0),(0,0,1),'observation','binding',time.time())
        self.adapter=SimpleNamespace(manipulation=Manipulation(self.scene),raw={'T_base_ee':np.eye(4)})
        self.start=np.eye(4);self.start[2,3]=.25

    def plan(self,delta,angle=0):
        return {'start_robot_state':{'T_base_ee':self.start},'target_pose':{'position':self.start[:3,3]+delta},'rotation_rad':angle}

    def test_explicit_task_scoped_nonresumable_authorization(self):
        with self.assertRaises(ValueError):PhysicalSession('task','')
        self.assertTrue(self.policy.permits('task'));self.assertFalse(self.policy.permits('different'))
        self.policy.active=False;self.assertFalse(self.policy.permits('task'))

    def test_native_call_restores_host_scheduling_even_on_error(self):
        from robol2l_deploy.adapters.panda import library_thread_policy
        parameters=object()
        with patch('os.sched_getscheduler',side_effect=[0,1]),patch('os.sched_getparam',return_value=parameters),patch('os.sched_setscheduler') as restore:
            with self.assertRaises(RuntimeError):
                with library_thread_policy():raise RuntimeError('test native error')
            restore.assert_called_once_with(0,0,parameters)

    def test_first_motion_must_be_small_and_away_from_support(self):
        self.assertIsNone(self.policy.check_motion(self.adapter,self.plan(np.array([0,0,.004]))))
        for delta in ([0,0,-.004],[.004,0,0],[0,0,.01]):
            self.assertIsNotNone(self.policy.check_motion(self.adapter,self.plan(np.array(delta))))
        self.assertIsNotNone(self.policy.check_motion(self.adapter,self.plan(np.array([0,0,.004]),.1)))

    def test_explicit_twenty_mm_probe_is_single_dispatch_and_does_not_change_default(self):
        probe=PhysicalSession('probe','User authorized 2 cm test',probe_distance_m=.02,maximum_dispatches=1)
        self.assertIsNone(probe.check_motion(self.adapter,self.plan(np.array([0,0,.02]))))
        self.assertIsNotNone(self.policy.check_motion(self.adapter,self.plan(np.array([0,0,.02]))))
        self.assertIsNotNone(probe.check_motion(self.adapter,self.plan(np.array([0,0,.004]))))
        probe.dispatches=1;self.assertFalse(probe.permits('probe'))

    def test_untrusted_or_stale_scenes_cannot_be_authorized(self):
        evidence={'source':'full_rgbd','manual_regions':False,'support_inliers':1000}
        self.assertTrue(self.policy.validate_scene(self.scene,evidence).reviewed)
        self.assertFalse(self.scene.reviewed)
        with self.assertRaises(ValueError):self.policy.validate_scene(self.scene,dict(evidence,manual_regions=True))
        from dataclasses import replace
        with self.assertRaises(ValueError):self.policy.validate_scene(replace(self.scene,captured_s=time.time()-31),evidence)

    def test_probe_and_height_guard_release(self):
        self.assertIsNotNone(self.policy.check_gripper(self.adapter,1.))
        self.policy.arm_probe=True;self.assertIsNone(self.policy.check_gripper(self.adapter,1.))
        body=Box('object',Pose([0,0,.02],[1,0,0,0],'base'),(.04,.04,.04),True)
        self.adapter.manipulation.attach(body,np.eye(4),.04)
        self.assertIsNone(self.policy.check_gripper(self.adapter,1.))
        self.adapter.raw['T_base_ee'][2,3]=.20
        self.assertIsNotNone(self.policy.check_gripper(self.adapter,1.))

if __name__=='__main__':unittest.main()
