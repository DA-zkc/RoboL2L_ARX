"""AABB pruning must preserve exhaustive Bullet contact decisions."""
import unittest
from unittest.mock import patch
import numpy as np
from franka_cap.planning import Kinematics
from robol2l_deploy.adapters.fr3_scene import PayloadCollisionScene
from robol2l_deploy.contracts import Pose
from robol2l_deploy.manipulation import Box, Manipulation, Scene


class CollisionBroadphaseTests(unittest.TestCase):
    def test_fixed_base_is_static_but_still_collides_with_attached_payload(self):
        q=np.array([0.,-.3,0.,-1.8,0.,1.5,.7])
        box=Box('mount_contact',Pose([0,0,.05],[1,0,0,0],'base'),(.12,.12,.12),True)
        manipulation=Manipulation(Scene((box,),(0,0,-1),(0,0,1),'test','test',0))
        scene=PayloadCollisionScene({'T_EE_camera':np.eye(4).tolist()},(),manipulation)
        try:
            scene.set_state(q,.08)
            self.assertNotIn(('fr3_link0','mount_contact'),[tuple(h['links']) for h in scene.violations()])
            self.assertTrue(any(scene.links[i]=='fr3_link0' for i in scene.fixed_links))
            self.assertFalse(any(i in scene.fixed_links for i in scene.arm))
            manipulation.attach(box,Kinematics().forward(q),.04)
            scene.set_state(q,.08)
            self.assertIn(('fr3_link0','mount_contact'),[tuple(h['links']) for h in scene.violations()])
        finally:scene.close()

    def test_same_contacts_as_exhaustive_queries_with_moving_payload(self):
        q=np.array([0.,-.3,0.,-1.8,0.,1.5,.7])
        tcp=Kinematics().forward(q)
        rng=np.random.default_rng(42)
        boxes=[Box('payload',Pose(tcp[:3,3],[1,0,0,0],'base'),(.04,.04,.04),True)]
        for index in range(100):
            center=rng.uniform([-.1,-.3,-.05],[.7,.3,.8])
            boxes.append(Box(str(index),Pose(center,[1,0,0,0],'base'),(.04,.04,.04)))
        manipulation=Manipulation(Scene(tuple(boxes),(0,0,-.01),(0,0,1),'test','test',0))
        scene=PayloadCollisionScene({'T_EE_camera':np.eye(4).tolist()},(),manipulation,'payload')
        def normalized(hits):
            return sorted((tuple(h['links']),round(h['distance_m'],12)) for h in hits)
        try:
            for holding in (False,True):
                if holding:manipulation.attach(boxes[0],tcp,.04)
                for offset in (0.,.1,-.1):
                    state=q.copy();state[0]+=offset
                    scene.set_state(state,.08)
                    actual=normalized(scene.violations())
                    with patch.object(scene,'nearby',return_value=range(len(boxes))):
                        expected=normalized(scene.violations())
                    self.assertEqual(actual,expected)
                    self.assertTrue(expected)  # Test includes actual collisions.
        finally:scene.close()
