"""Recorded incident -> IK -> native trajectory -> filtered software replay.

Never constructs Robot/Gripper. Output is software evidence, not a physical run.
"""
import json
import xml.etree.ElementTree as ET
import numpy as np
import franky
from scipy.spatial.transform import Rotation
from franka_cap.common import config
from franka_cap.geometry import pose_dict
from franka_cap.planning import Kinematics, MODEL, plan_path
from robol2l_deploy.adapters.franky import FrankyDriver
from tests.test_ik_joint_limits import INCIDENT_START, INCIDENT_TARGET


kin=Kinematics()
goal=kin.solve(INCIDENT_TARGET,INCIDENT_START)
actual=kin.forward(goal)
# libfranka's parser expects the seven arm joints, not hand/finger joints.
arm_urdf=ET.parse(MODEL/'fr3_hand.urdf').getroot()
for joint in list(arm_urdf.findall('joint')):
    if joint.get('name') not in [f'fr3_joint{i}' for i in range(1,8)]:arm_urdf.remove(joint)
arm_urdf=ET.tostring(arm_urdf,encoding='unicode')
cartesian=plan_path(dict(q=INCIDENT_START,has_errors=False,
    T_base_ee=kin.forward(INCIDENT_START)),pose_dict(INCIDENT_TARGET),config(),None,scene_factory=None)
results={}
for name,points in [('joint_target',[INCIDENT_START,goal]),('cartesian_path',cartesian['q_samples'])]:
    driver=FrankyDriver('offline-only')
    plan=driver.plan(points,speed_factor=.05)
    curve=driver.trajectories[plan['token']]
    sampled=np.asarray([curve.sample(float(t)) for t in
        np.linspace(0,curve.duration,int(np.ceil(curve.duration/.001))+1)])
    lower,upper=np.asarray(franky.station_velocity_limits_for_test(arm_urdf,sampled[:,0]))
    assert np.all(sampled[:,1]>=lower) and np.all(sampled[:,1]<=upper)
    assert np.all(sampled[:,0]>kin.lower) and np.all(sampled[:,0]<kin.upper)
    motion=franky.station_replay_for_test(curve,0.,True)
    report=motion.report();trace=motion.trace()
    assert report['ok'] and report['completion']=='franky.JointMotion'
    assert .010<=report['control_duration_s']-curve.duration<.013
    for t,q in zip(trace['reference_time_s'],trace['reference_q']):
        np.testing.assert_allclose(q,curve.sample(t)[0],rtol=0,atol=1e-12)
    assert driver.robot is None and driver.gripper is None
    end=np.searchsorted(trace['reference_time_s'],curve.duration)
    results[name]=dict(report,
        min_position_velocity_headroom_rad_s=float(min(np.min(upper-sampled[:,1]),np.min(sampled[:,1]-lower))),
        filtered_speed_at_curve_end_rad_s=float(np.max(np.abs(trace['dq'][end]))),
        filtered_speed_at_control_end_rad_s=float(np.max(np.abs(trace['dq'][-1]))))

print(json.dumps(dict(synthetic=True,physical_evidence=False,robot_connected=False,
    source='CaP 546951662acb4600bc4b2e563feb1139 v05',
    ik=dict(joint_positions=goal.tolist(),joint_limit_clearance_rad=np.minimum(goal-kin.lower,kin.upper-goal).tolist(),
        position_residual_m=float(np.linalg.norm(actual[:3,3]-INCIDENT_TARGET[:3,3])),
        rotation_residual_rad=float(Rotation.from_matrix(INCIDENT_TARGET[:3,:3].T@actual[:3,:3]).magnitude())),
    paths=results),indent=2))
