"""Read the installed SDK planner/reference state; never infer physical arrival.

The resident owner executes these fixed read operations on its existing SDK.
No controller is constructed and no gains or native limits are changed.
"""
import time
import numpy as np


QUERIES = {'plan_duration', 'reference_status', 'joint_path'}


class ReferenceUpdateMonitor:
    """Observe native errors that OnlineTrajectory.step catches internally.

    Delegate unchanged input, output, result and exception to the installed
    Ruckig object. No retry, time limit, tolerance or trajectory modification.
    """
    def __init__(self, engine):
        self.engine = engine
        self.error = None

    def __getattr__(self, name):
        return getattr(self.engine, name)

    def _record(self, input, **error):
        self.error = dict(source='installed_ruckig_update', observed_s=time.time(),
                          target_position=list(input.target_position),
                          native_input=str(input), **error)

    def update(self, input, output):
        try:
            result = self.engine.update(input, output)
        except Exception as exc:
            self._record(input, type=type(exc).__name__, message=str(exc))
            raise
        if int(result) < 0:
            self._record(input, type='RuckigResult', code=int(result), message=str(result))
        else:
            self.error = None
        return result


def monitor_reference(controller):
    engine = controller.otg._otg
    if not isinstance(engine, ReferenceUpdateMonitor):
        engine = ReferenceUpdateMonitor(engine)
        controller.otg._otg = engine
    return engine


def query(arm, name, **arguments):
    if name not in QUERIES:
        raise ValueError('unknown_arx_native_query')
    remote = getattr(arm, 'native_query', None)
    if remote is not None:
        return remote(name, **arguments)
    controller = arm._ctrl
    with controller._cmd_lock:
        monitor = monitor_reference(controller)
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
        error = monitor.error
        if error and not np.array_equal(error['target_position'], controller._q_target):
            error = None  # A receipt from a previous/replaced target is historical.
        return dict(joints_finished=bool(controller.otg.at_target(controller._q_target)),
                    gripper_finished=controller._grip_traj is None,
                    reference_error=error,
                    reference_error_monitor_version=1,
                    source='installed_arx_sdk_reference', physical_arrival_verified=False)
