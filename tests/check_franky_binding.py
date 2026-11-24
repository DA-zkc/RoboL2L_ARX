"""Native Ruckig + franky motion replay. Never constructs Robot or Gripper."""
import json
import numpy as np
import franky
from robol2l_deploy.adapters.franky import FrankyDriver

driver = FrankyDriver('offline-only')
start = np.array([0., -.3, 0., -1.8, 0., 1.5, .7])
middle = start + [.01, .005, -.002, .003, .004, -.003, .002]
goal = start + [.02, -.002, .004, .005, .001, -.002, .003]
plan = driver.plan([start.tolist(), middle.tolist(), goal.tolist()], speed_factor=.01)
path = driver.trajectories[plan['token']]
assert driver.robot is None and driver.gripper is None
assert np.allclose(plan['joint_positions'][0], start)
assert np.allclose(plan['joint_positions'][-1], goal)
assert plan['dynamics']['velocity_rad_s'] <= .15
assert plan['dynamics']['acceleration_rad_s2'] <= .3

# Execute the same compiled native callback against synthetic feedback. Check
# both exact feedback and a persistent bias: report error without a policy gate.
results = []
for bias in (0., .004):
    motion = franky.station_replay_for_test(path, bias)
    assert isinstance(motion, franky.BaseJointPositionMotion)
    report = motion.report()
    assert report['completed'] and report['ok']
    assert np.isclose(report['final_max_joint_error_rad'],bias)
    trace = motion.trace()
    for t, command in zip(trace['reference_time_s'], trace['reference_q']):
        assert np.allclose(command, path.sample(t)[0], rtol=0, atol=1e-12)
    assert report['completion'] == 'franky.JointMotion'
    # The curve ending is not the control session ending: upstream holds its
    # terminal command for filter cooldown before emitting MotionFinished.
    assert .010 <= report['control_duration_s']-path.duration < .013
    results.append(report)

filtered = franky.station_replay_for_test(path, 0., True)
trace = filtered.trace()
end = np.searchsorted(trace['reference_time_s'], path.duration)
tail = np.asarray(trace['reference_q'])[end:]
np.testing.assert_allclose(tail, np.broadcast_to(goal, tail.shape), atol=1e-12, rtol=0)
assert np.linalg.norm(trace['dq'][-1]) < np.linalg.norm(trace['dq'][end])
assert filtered.report()['completed']
results.append(filtered.report())
print(json.dumps(dict(synthetic=True, physical_evidence=False,
    native_replays=results, backend=plan['backend'], robot_connected=False), indent=2))
