import sys,json
import numpy as np
from scipy.interpolate import CubicSpline
from robol2l_deploy.adapters.curves import coefficients
points=np.array([np.linspace(0,.1,7),np.linspace(.02,.13,7),np.linspace(.01,.2,7)])
dt=.31;c,p=coefficients(points,dt);spline=CubicSpline(np.arange(3)*dt,points,bc_type='clamped')
if sys.argv[1]=='franky':
 import franky
 curve=franky.StationJointTrajectory.from_coefficients(c,dt)
 sample=lambda t:curve.sample(t)
 duration=curve.duration
else:
 import panda_py.motion
 curve=panda_py.motion.JointTrajectory.from_coefficients(c,dt)
 sample=lambda t:[curve.get_joint_positions(t),curve.get_joint_velocities(t),curve.get_joint_accelerations(t)]
 duration=curve.get_duration()
errors=[]
for t in np.linspace(0,2*dt,41):
 actual=np.array(sample(float(t)));expected=np.array([spline(t,n) for n in range(3)])
 errors.append(float(np.max(np.abs(actual-expected))))
assert abs(duration-2*dt)<1e-12
assert max(errors)<1e-10,max(errors)
if sys.argv[1]=='franky':
 motion=franky.station_replay_for_test(curve,0.,True)
 report=motion.report();trace=motion.trace()
 assert report['completed'] and report['completion']=='franky.JointMotion'
 assert .010<=report['control_duration_s']-duration<.013
 for t,q in zip(trace['reference_time_s'],trace['reference_q']):
  np.testing.assert_allclose(q,curve.sample(t)[0],rtol=0,atol=1e-12)
print(json.dumps(dict(backend=sys.argv[1],duration_s=duration,max_curve_error=max(errors),hardware_connected=False)))
