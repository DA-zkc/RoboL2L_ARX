"""franky backend. Lazy connection; all actuation stays behind StationRuntime."""
from pathlib import Path
from uuid import uuid4
import numpy as np
from .franka_common import library_thread_policy, robot_state
from ..runtime import OutcomeUnknown


class FrankyDriver:
    backend = 'franky'
    planner = 'franky.RuckigJointTrajectory'

    def __init__(self, address, *, allow_motion=False, library=None):
        if library is None:
            import franky as library
        if not hasattr(library, 'StationJointTrajectory'):
            raise RuntimeError('franky_station_binding_missing: run scripts/build-franky.sh')
        self.library = library
        self.address = address
        self.allow_motion = allow_motion
        self.robot = self.gripper = self._ownership = None
        self.trajectories = {}
        self.trace_directory = None
        self.speed_factor = .05
        self.max_speed_factor = 1.
        self.control_profile = dict(controller='libfranka.internal_joint_impedance',
            completion='franky.JointMotion', gains='robot_settings_unchanged',
            path='Ruckig stop at path waypoints', limit_rate=False, cutoff_frequency_hz=100.)

    def realtime_ready(self):
        with library_thread_policy():
            return self.library.station_realtime_ready()

    def connect(self):
        if self.robot is not None:
            return
        from franka_cap.common import hardware_lock
        self._ownership = hardware_lock()
        self._ownership.__enter__()
        try:
            with library_thread_policy():
                self.robot = self.library.Robot(self.address,
                    controller_mode=self.library.ControllerMode.JointImpedance,
                    realtime_config=self.library.RealtimeConfig.Enforce,
                    configure_collision_behavior=False)
                self.gripper = self.library.Gripper(self.address)
        except Exception:
            self.close()
            raise

    def read(self):
        self.connect()
        with library_thread_policy():
            state = self.robot.state
            # franky.Errors has no __bool__; even its empty value is truthy.
            # Its documented binding serializes the actual libfranka error bits.
            flags = state.current_errors.__getstate__()
            if not isinstance(flags, tuple) or not flags or any(type(flag) is not bool for flag in flags):
                raise ValueError('invalid_franky_error_flags')
            return robot_state(state, self.gripper.state, self.backend, has_errors=any(flags))

    def recover(self):
        """Operator-pressed error recovery. The execution path never calls this."""
        self.connect()
        with library_thread_policy():
            self.robot.recover_from_errors()
        return self.read()

    def prepare_samples(self, positions, dt_s):
        from .curves import coefficients
        c,points=coefficients(positions,dt_s)
        curve=self.library.StationJointTrajectory.from_coefficients(c,dt_s)
        token=uuid4().hex;self.trajectories[token]=curve
        return dict(token=token,duration_s=curve.duration,dt_s=dt_s,joint_positions=points,
            backend='franky.sampled_joint_trajectory',timing='caller_dt_s',collision_checked=False)

    def plan(self, waypoints, *, speed_factor=None, interpolation_dt_s=None, maximum_trajectory_dt_s=None):
        factor = self.speed_factor if speed_factor is None else speed_factor
        from .drivers import resolve_speed_factor
        factor = resolve_speed_factor(factor,self.speed_factor,self.max_speed_factor)
        trajectory = self.library.StationJointTrajectory(waypoints,
            2.175*factor, 7.5*factor, 3750*factor)
        duration = float(trajectory.duration)
        from .curves import sample_interval
        count = max(2, int(np.ceil(duration/sample_interval(interpolation_dt_s,maximum_trajectory_dt_s)))+1)
        samples = np.asarray([trajectory.sample(float(t)) for t in np.linspace(0, duration, count)])
        token = uuid4().hex
        self.trajectories[token] = trajectory
        return dict(token=token, duration_s=duration, dt_s=duration/(count-1),
            joint_positions=samples[:, 0], backend=self.planner, library_version='2.0.0+station',
            speed_factor=factor, max_speed_factor=self.max_speed_factor,
            dynamics=dict(velocity_rad_s=float(np.max(np.abs(samples[:, 1]))),
                acceleration_rad_s2=float(np.max(np.abs(samples[:, 2]))), source='actual_ruckig_time_curve'))

    def execute(self, token):
        self._require_motion()
        trajectory = self.trajectories[token]
        self.connect()
        motion = self.library.StationJointMotion(trajectory)
        try:
            with library_thread_policy():
                self.robot.move(motion, asynchronous=False,
                    limit_rate=self.control_profile['limit_rate'],
                    cutoff_frequency=self.control_profile['cutoff_frequency_hz'])
            report = motion.report()
            return dict(self.read(), ok=bool(report['completed'] and report['ok']),
                elapsed_s=report['control_duration_s'], controller_report=report,
                control_profile=self.control_profile, control_trace=self.save_trace(token, motion))
        except Exception as error:
            failure = self._unknown('franky_terminal_receipt_unavailable', error)
            try:
                failure.diagnostics['control_trace'] = self.save_trace(token, motion)
            except Exception as trace_error:
                failure.diagnostics['trace_error_type'] = type(trace_error).__name__
            raise failure from error

    def save_trace(self, token, motion):
        if self.trace_directory is None:
            return None
        arrays = {name: np.asarray(value) for name, value in motion.trace().items()}
        directory = Path(self.trace_directory)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory/(token+'-'+uuid4().hex+'.npz')
        np.savez_compressed(path, **arrays)
        return dict(path=str(path.resolve()), samples=len(arrays['q']),
            alignment='native franky control-loop relative time; installed native reference curve')

    def _require_motion(self):
        if not self.allow_motion:
            raise ValueError('franky_motion_not_commissioned')

    @staticmethod
    def _unknown(message, error):
        return OutcomeUnknown(message, diagnostics=dict(driver='franky',
            cause_type=type(error).__name__, cause_message=str(error)[:2000]))

    def _hand_command(self, operation, *arguments):
        self._require_motion()
        self.connect()
        try:
            with library_thread_policy():
                ok = getattr(self.gripper, operation)(*arguments)
                state = self.gripper.state
            return dict(ok=bool(ok), is_grasped=bool(state.is_grasped), width_m=float(state.width))
        except Exception as error:
            raise self._unknown('franky_gripper_receipt_unavailable', error) from error

    def move_gripper(self, width):
        return self._hand_command('move', float(width), .02)

    def grasp(self, width=None, force=5.):
        # No exact width is supplied by public close_gripper. Use libfranka's
        # contact-close operation over the calibrated stroke, not a visual
        # bounding box plus an arbitrary millimetre acceptance window.
        self.connect()
        maximum = float(self.gripper.state.max_width)
        receipt = self._hand_command('grasp', 0., .02, float(force), 0., maximum)
        return dict(receipt, command='force_contact_close',
                    command_width_m=0., force_n=float(force))

    def close(self):
        self.trajectories.clear()
        self.gripper = self.robot = None
        if self._ownership is not None:
            self._ownership.__exit__(None, None, None)
            self._ownership = None
