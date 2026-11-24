"""Offline IK regressions, including the 2026-11-19 v05 incident target."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from franka_cap.common import config
from franka_cap.geometry import pose_dict, pose_matrix
from franka_cap.planning import Kinematics, plan_path


INCIDENT_START=np.array([-.502774178981781,.5136218667030334,.5701788067817688,
    -1.6285202503204346,2.858973741531372,4.259713172912598,2.5835390090942383])
INCIDENT_TARGET=pose_matrix(dict(
    position=[.4530511700831267,-.06707297650304209,.29828941962946776],
    quaternion_wxyz=[-.015974912396467256,.7367789582836696,.6759125801599908,-.006614588862504311],
    frame='base'))


class IKJointLimitTests(unittest.TestCase):
    def setUp(self):self.kin=Kinematics()

    def assert_pose(self,q,target):
        actual=self.kin.forward(q)
        self.assertLess(np.linalg.norm(actual[:3,3]-target[:3,3]),1e-9)
        self.assertLess(Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).magnitude(),1e-9)
        self.assertTrue(np.all(q>self.kin.lower) and np.all(q<self.kin.upper))

    def test_incident_target_uses_redundancy_without_pose_tradeoff(self):
        q=self.kin.solve(INCIDENT_TARGET,INCIDENT_START)
        self.assert_pose(q,INCIDENT_TARGET)
        # Regression expectation for this recorded case, not an execution gate.
        self.assertGreater(self.kin.upper[4]-q[4],.01)
        self.assertLess(np.max(np.abs(q-INCIDENT_START)),.3)

    def test_incident_cartesian_path_is_continuous_and_within_actual_limits(self):
        plan=plan_path(dict(q=INCIDENT_START,has_errors=False,
            T_base_ee=self.kin.forward(INCIDENT_START)),pose_dict(INCIDENT_TARGET),
            config(),None,scene_factory=None)
        q=np.asarray(plan['q_samples'])
        self.assert_pose(q[-1],INCIDENT_TARGET)
        self.assertTrue(np.all(q>self.kin.lower) and np.all(q<self.kin.upper))
        self.assertLess(np.max(np.abs(np.diff(q,axis=0))),.01)
        self.assertFalse(plan['collision_checked'])

    def test_seed_pose_is_unchanged_even_near_limits(self):
        q=self.kin.solve(self.kin.forward(INCIDENT_START),INCIDENT_START)
        np.testing.assert_allclose(q,INCIDENT_START,atol=1e-12,rtol=0)

    def test_both_sides_of_joint_range_and_exact_bound_seeds(self):
        for sign in (-1,1):
            seed=np.array([-.5*sign,.5,.5*sign,-1.7,
                self.kin.upper[4] if sign>0 else self.kin.lower[4],4.2,2.5*sign])
            goal=self.kin.forward(seed);goal[2,3]+=.02
            with self.subTest(sign=sign):self.assert_pose(self.kin.solve(goal,seed),goal)

    def test_unreachable_pose_is_not_accepted_as_approximate_ik(self):
        goal=INCIDENT_TARGET.copy();goal[0,3]=10.
        with self.assertRaisesRegex(ValueError,'IK pose-constrained solver'):
            self.kin.solve(goal,INCIDENT_START)


if __name__=='__main__':unittest.main()
