"""panda_py binding under the station owner; construction never connects."""
from pathlib import Path
from uuid import uuid4
import numpy as np
from ..runtime import OutcomeUnknown
from .franka_common import library_thread_policy, robot_state


def terminal_receipt_error(message, error):
    # Local device/binding errors only; model transport errors never enter here.
    return OutcomeUnknown(message, diagnostics={'driver':'panda_py',
        'cause_type':type(error).__name__, 'cause_message':str(error)[:2000]})


class PandaDriver:
    backend = "panda_py"
    minimum_speed_factor = .001  # Installed library constant kMinSpeedFactor; no extra gate.
    planner = "panda_py.JointTrajectory"

    def realtime_ready(self):
        with library_thread_policy():
            return self.library.libfranka.has_realtime_kernel() and self.library.realtime_priority_available()[0]

    def __init__(self, address, *, allow_motion=False, library=None):
        if library is None:
            import panda_py as library
            import panda_py.motion
        self.library=library;self.address=address;self.allow_motion=allow_motion
        self.robot=None;self.gripper=None;self.trajectories={};self._ownership=None;self.speed_factor=.05
        self.max_speed_factor=1.
        self.trace_directory=None
        self.control_profile=dict(stiffness=[600,600,600,600,250,150,50],damping=[50,50,50,20,20,20,10],
            completion='native_curve_finished',compensate_coriolis=True)

    def connect(self):
        if self.robot is None:
            from franka_cap.common import hardware_lock
            self._ownership=hardware_lock()
            self._ownership.__enter__()
            try:
                with library_thread_policy():
                    self.robot=self.library.Panda(self.address, realtime_config=self.library.libfranka.RealtimeConfig.kEnforce)
                    self.robot.set_automatic_recovery(False)
                    self.gripper=self.library.libfranka.Gripper(self.address)
            except Exception:
                self.close()
                raise

    def read(self):
        self.connect()
        with library_thread_policy():
            state=self.robot.get_state();hand=self.gripper.read_once()
        return robot_state(state,hand,self.backend)

    def recover(self):
        """Operator-pressed error recovery; automatic recovery stays off for control."""
        self.connect()
        with library_thread_policy():
            self.robot.recover()
        return self.read()

    def prepare_samples(self, positions, dt_s):
        from .curves import coefficients
        c,points=coefficients(positions,dt_s)
        curve=self.library.motion.JointTrajectory.from_coefficients(c,dt_s)
        token=uuid4().hex;self.trajectories[token]=curve
        return dict(token=token,duration_s=curve.get_duration(),dt_s=dt_s,joint_positions=points,
            backend='panda_py.sampled_joint_trajectory',timing='caller_dt_s',collision_checked=False)

    def plan(self, waypoints, *, speed_factor=None, interpolation_dt_s=None, maximum_trajectory_dt_s=None):
        from .drivers import resolve_speed_factor
        speed_factor=resolve_speed_factor(speed_factor,self.speed_factor,self.max_speed_factor)
        trajectory=self.library.motion.JointTrajectory([np.array(q) for q in waypoints],speed_factor=speed_factor,max_deviation=0.)
        duration=float(trajectory.get_duration())
        if not np.isfinite(duration) or duration<=0:raise ValueError('invalid_library_trajectory_duration')
        from .curves import sample_interval
        count=max(2,int(np.ceil(duration/sample_interval(interpolation_dt_s,maximum_trajectory_dt_s)))+1)
        samples=np.array([trajectory.get_joint_positions(float(t)) for t in np.linspace(0,duration,count)])
        velocities=np.array([trajectory.get_joint_velocities(float(t)) for t in np.linspace(0,duration,count)])
        accelerations=np.array([trajectory.get_joint_accelerations(float(t)) for t in np.linspace(0,duration,count)])
        if not all(np.isfinite(v).all() for v in (samples,velocities,accelerations)):
            raise ValueError('invalid_library_trajectory')
        token=uuid4().hex
        self.trajectories[token]=trajectory
        return dict(token=token,duration_s=duration,dt_s=duration/(count-1),joint_positions=samples,
                    speed_factor=speed_factor,max_speed_factor=self.max_speed_factor,
                    dynamics={'velocity_rad_s':float(np.max(np.abs(velocities))),'acceleration_rad_s2':float(np.max(np.abs(accelerations))),'source':'actual_panda_py_time_curve'},
                    backend='panda_py.JointTrajectory',library_version=getattr(self.library,'__version__','1.1.1'))

    def execute(self, token):
        if not self.allow_motion:raise ValueError('panda_py_motion_not_commissioned')
        trajectory=self.trajectories[token]
        self.connect()
        if self.trace_directory is not None:self.robot.enable_logging(20000)
        try:
            with library_thread_policy():
                ok=self.robot.execute_joint_trajectory(trajectory,success_threshold=0.,
                    settle_timeout=0.,compensate_coriolis=self.control_profile['compensate_coriolis'])
            report=self.robot.get_trajectory_report()
            trace=self.save_trace(token,trajectory)
            return dict(self.read(),ok=bool(ok),elapsed_s=report['control_duration_s'],
                controller_report=report,control_profile=self.control_profile,control_trace=trace)
        except Exception as error:
            failure=terminal_receipt_error('panda_py_terminal_receipt_unavailable',error)
            try:failure.diagnostics['control_trace']=self.save_trace(token,trajectory)
            except Exception as trace_error:failure.diagnostics['trace_error_type']=type(trace_error).__name__
            raise failure from error
        finally:
            if self.trace_directory is not None:self.robot.disable_logging()

    def save_trace(self,token,trajectory):
        if self.trace_directory is None:return None
        self.robot.disable_logging()
        arrays={key:np.asarray(value) for key,value in self.robot.get_log().items()}
        if not len(arrays.get('q',[])):raise ValueError('control_trace_empty')
        active=np.flatnonzero(arrays['robot_mode'].reshape(-1)==2)
        if not len(active):raise ValueError('control_trace_has_no_move_state')
        # Device timestamps, aligned to the first move sample (1 kHz resolution).
        elapsed=(arrays['time'].reshape(-1)-arrays['time'].reshape(-1)[active[0]])/1000.
        arrays['reference_time_s']=elapsed
        arrays['reference_q']=np.array([trajectory.get_joint_positions(float(max(0,t))) for t in elapsed])
        arrays['reference_dq']=np.array([trajectory.get_joint_velocities(float(max(0,t))) for t in elapsed])
        directory=Path(self.trace_directory);directory.mkdir(parents=True,exist_ok=True)
        path=directory/(token+'-'+uuid4().hex+'.npz');np.savez_compressed(path,**arrays)
        return dict(path=str(path.resolve()),samples=len(elapsed),capacity=20000,
            alignment='device time relative to first move sample; 1 ms resolution')

    def move_gripper(self, width):
        if not self.allow_motion:raise ValueError('panda_py_motion_not_commissioned')
        self.connect()
        try:
            with library_thread_policy():
                ok=self.gripper.move(float(width),.02);state=self.gripper.read_once()
            return dict(ok=bool(ok),is_grasped=bool(state.is_grasped),width_m=float(state.width))
        except Exception as error:raise terminal_receipt_error('panda_py_gripper_receipt_unavailable',error) from error

    def grasp(self, width=None, force=5.):
        if not self.allow_motion:raise ValueError('panda_py_motion_not_commissioned')
        self.connect()
        try:
            with library_thread_policy():
                # Public close_gripper has no exact object-width argument. Close
                # under force control until contact within the physical stroke;
                # the visual envelope is not a calibrated width measurement.
                maximum=float(self.gripper.read_once().max_width)
                ok=self.gripper.grasp(0.,.02,float(force),0.,maximum);state=self.gripper.read_once()
            return dict(ok=bool(ok),is_grasped=bool(state.is_grasped),width_m=float(state.width),
                        command='force_contact_close',command_width_m=0.,force_n=float(force))
        except Exception as error:raise terminal_receipt_error('panda_py_grasp_receipt_unavailable',error) from error

    def close(self):
        self.trajectories.clear()
        self.gripper=None;self.robot=None
        if self._ownership is not None:
            self._ownership.__exit__(None,None,None);self._ownership=None
