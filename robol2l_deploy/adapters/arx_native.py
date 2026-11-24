"""Read the installed SDK planner/reference state; never infer physical arrival.

The resident owner executes these fixed read operations on its existing SDK.
No controller is constructed and no gains or native limits are changed.
"""
import numpy as np


QUERIES = {'plan_duration', 'reference_status', 'joint_path'}


def query(arm, name, **arguments):
    if name not in QUERIES:
        raise ValueError('unknown_arx_native_query')
    remote = getattr(arm, 'native_query', None)
    if remote is not None:
        return remote(name, **arguments)
    controller = arm._ctrl
    with controller._cmd_lock:
        if name == 'joint_path':
            otg = controller.otg
            duration = otg.predict(np.asarray(arguments['start'], dtype=float),
                                   np.asarray(arguments['target'], dtype=float),
                                   min_duration=float(arguments['minimum']))
            return [otg._ptraj.at_time(float(t))[0]
                    for t in np.r_[np.arange(0., duration, otg.dt), duration]]
        if name == 'plan_duration':
            return float(controller.plan_duration(
                np.asarray(arguments['start'], dtype=float),
                np.asarray(arguments['target'], dtype=float), float(arguments['minimum'])))
        return dict(joints_finished=bool(controller.otg.at_target(controller._q_target)),
                    gripper_finished=controller._grip_traj is None,
                    source='installed_arx_sdk_reference', physical_arrival_verified=False)
