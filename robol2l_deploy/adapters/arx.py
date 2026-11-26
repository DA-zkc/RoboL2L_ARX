"""Single-arm X5 (2023) station backed by the ARX5_beta Python SDK."""

from contextlib import ExitStack
from functools import cached_property
from math import ceil
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from cap_harness.geometry import matrix_to_pose
from franka_cap.common import digest, hardware_lock, read_json, scalar
from ..contracts import CameraObservation, ExecutionResult, IKResult, Observation, Pose, RobotState, Trajectory
from ..errors import ApiError, ErrorCode
from ..execution_feedback import endpoint_accuracy
from ..runtime import failure, fingerprint, public
from .arx_native import query as native_query

def resolve_speed_factor(requested, default=1., maximum=None):
    value = default if requested is None else requested
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
        raise ValueError("positive_finite_speed_factor_required")
    return float(value)

from .arx_payload import file_sha256


EMBODIMENT = 'arx_x5_2023'
JOINT_NAMES = [f'joint{i}' for i in range(1, 7)]
TIME_QUANTUM_S = 0.01
GRIPPER_RANGE = (0., 5.)
EMPTY_CLOSED_MAX_RAD = .1


class ARXDriver:
    backend = EMBODIMENT

    def __init__(self, can_port, *, allow_connection=False, allow_motion=False, acquire_lock=hardware_lock,
                 urdf_path=None, urdf_sha256=None):
        self.acquire_lock = acquire_lock
        self.can_port = can_port
        self.allow_connection = allow_connection
        self.allow_motion = allow_motion
        self._devices = ExitStack()
        self._arm = None
        self.owner = None
        self.owner_side = None
        self.urdf_path = str(Path(urdf_path).resolve()) if urdf_path else None
        self.urdf_sha256 = urdf_sha256
        self.model_config()  # Reject an invalid override before acquiring any device.

    def model_config(self):
        config = {'type': 0}
        if self.urdf_path:
            if not self.urdf_sha256 or file_sha256(self.urdf_path) != self.urdf_sha256:
                raise ValueError('arx_payload_urdf_changed')
            config['urdf_path'] = self.urdf_path
        return config

    def realtime_ready(self):
        flags = Path('/sys/class/net') / self.can_port / 'flags'
        return self.allow_connection and flags.is_file() and bool(int(flags.read_text(), 16) & 1)

    @property
    def arm(self):
        if self._arm is None:
            if not self.allow_connection:
                raise ApiError(ErrorCode.SAFETY_INTERLOCK, 'arx_connection_requires_explicit_enable')
            if self.owner is not None:
                self._arm = self.owner.arm(self.owner_side)
                return self._arm
            if not self.realtime_ready():
                raise ApiError(ErrorCode.SAFETY_INTERLOCK, 'arx_can_interface_down',
                               details={'can_port': self.can_port})
            from bimanual import SingleArm

            with ExitStack() as connecting:
                connecting.enter_context(self.acquire_lock())
                self._arm = SingleArm(dict(self.model_config(), can_port=self.can_port))
                self._devices = connecting.pop_all()
        return self._arm

    def close(self, *, hardware_supported=False):
        if self.owner is not None:
            self._arm = None  # The pair releases its lease; never destroy the resident SDK.
            return
        if self._arm is not None and not hardware_supported:
            raise RuntimeError('hardware_support_confirmation_required')
        if self._arm is not None:
            self._arm.close()
            self._arm = None
        self._devices.close()


class ARXStationAdapter:
    synthetic = False
    model_name = 'X5-2023'
    end_effector_link = 'link6'

    @classmethod
    def from_file(cls, path, **options):
        path = Path(path).resolve()
        cfg = read_json(path)
        if cfg.get('schema') != 1 or cfg.get('station') != 'arx-x5-2023' or cfg.get('motion_enabled') is not False:
            raise ValueError('invalid_arx_station_configuration')
        return cls(cfg, directory=path.parent, **options)

    def __init__(self, cfg, *, directory, allow_connection=False, allow_motion=False, driver=None):
        self.can_port = cfg['can_port']
        if self.can_port not in ('can1', 'can3'):
            raise ValueError('arx_can_port_must_be_can1_or_can3')
        self.home_binding = f'{EMBODIMENT}:{self.can_port}'
        self.driver = driver if driver is not None else ARXDriver(
            self.can_port, allow_connection=allow_connection, allow_motion=allow_motion)
        motion = cfg.get('motion', {})
        self.max_speed_factor = None
        self.speed_factor = resolve_speed_factor(motion.get('default_speed_factor', 1.))

        self.gripper = None
        if cfg.get('gripper') is not None:
            closed = scalar(cfg['gripper']['closed_rad'], 'closed_rad', *GRIPPER_RANGE)
            opened = scalar(cfg['gripper']['open_rad'], 'open_rad', *GRIPPER_RANGE)
            if opened == closed:
                raise ValueError('distinct_gripper_endpoints_required')
            self.gripper = closed, opened
        self.unsupported_operations = {'step', 'close_gripper'}
        if self.gripper is None:
            self.unsupported_operations.update(('set_gripper', 'set_grippers', 'open_gripper'))

        self.cameras, self._calibration = {}, {}
        if cfg.get('cameras'):
            from franka_cap.devices import calibration
            from ..cameras import CameraOwner
            from .rgbd import GeminiSource, RealSenseSource

            sources = {'gemini': GeminiSource, 'realsense': RealSenseSource}
            for name, spec in cfg['cameras'].items():
                if name not in ('global', 'wrist') or spec['driver'] not in sources:
                    raise ValueError('invalid_arx_camera_configuration')
                self._calibration[name + '_serial'] = spec['serial']
                self._calibration[name + '_calibration'] = str(directory / spec['calibration'])
                calibration(self._calibration, name, {'T_base_ee': np.eye(4)})
                self.cameras[name] = CameraOwner(sources[spec['driver']](spec['serial']))
        self._cfg = cfg
        self.raw = None
        if not self.cameras:
            self.unsupported_operations.add('get_observation')

    @cached_property
    def kin(self):
        from bimanual.core.kinematics import Kinematics
        from bimanual.core.model import get_arm_model

        model = get_arm_model(self.driver.model_config() if isinstance(self.driver, ARXDriver) else {'type': 0})
        return Kinematics(model.urdf_path, model.ee_link, model.urdf_joint_names,
                          verbose=False, doc_limits_deg=model.doc_limits_deg)

    def configuration(self):
        return dict(station=self._cfg, robot_backend=EMBODIMENT)

    def describe(self):
        gripper_description = ('Calibrated encoder angle; position control' if self.gripper is not None
                               else 'Uncalibrated encoder angle; gripper control unavailable')
        return dict(synthetic=False, embodiment=EMBODIMENT, arms=['primary'], cameras=list(self.cameras),
                    can_port=self.can_port, robot_backend=EMBODIMENT, planning_backend='arx_point_to_point',
                    control_profile='ARX SDK generated point-to-point trajectory',
                    contact_grasp=False, gripper_calibrated=self.gripper is not None,
                    gripper_description=gripper_description,
                    motion_speed=dict(default_speed_factor=self.speed_factor, max_speed_factor=self.max_speed_factor,
                                      max_joint_speed_rad_s=None, timing_source="installed_sdk_planner"))

    def device_binding(self):
        return fingerprint(dict(configuration=self._cfg,
            calibrations={name: digest(self._calibration[name + '_calibration']) for name in self.cameras}))

    def binding(self):
        return self.device_binding()

    def _pose(self, joints):
        # ARX FK is relative to the zero-position end-effector frame. The
        # station exposes poses in the physical URDF base frame.
        transform = self.kin.T_home.homogeneous @ self.kin.fk(joints).homogeneous
        return matrix_to_pose(transform, frame='base')

    def _joints(self, joints, *, measured=False):
        joints = np.asarray(joints, dtype=float)
        if joints.shape != (6,) or not np.isfinite(joints).all():
            raise ValueError('arx_requires_six_finite_joints')
        if measured:
            return joints
        lower, upper = self.kin.q_min, self.kin.q_max
        outside = np.flatnonzero((joints < lower) | (joints > upper))
        if outside.size:
            limits = ', '.join(f'{JOINT_NAMES[i]}={joints[i]:.6g} rad [{lower[i]:.6g}, {upper[i]:.6g}]'
                               for i in outside)
            raise ValueError('arx_joint_target_outside_limits: ' + limits)
        return joints

    def get_robot_state(self):
        arm = self.driver.arm
        fault, offline = arm.fault, list(arm.offline_joints or ())
        if fault or offline:
            raise ApiError(ErrorCode.SAFETY_INTERLOCK, 'arx_fault_or_offline',
                           details={'fault': str(fault) if fault else None, 'offline_joints': offline})
        joints = arm.get_joint_positions(joint_names=JOINT_NAMES)
        velocities = arm.get_joint_velocities(joint_names=JOINT_NAMES)
        pose = self._pose(joints)
        gripper_rad = float(arm.get_gripper_pos())
        if not np.isfinite(gripper_rad):
            raise ValueError('invalid_arx_gripper_feedback')
        closed, opened = self.gripper or GRIPPER_RANGE
        position = float(np.clip((gripper_rad - closed) / (opened - closed), 0., 1.))
        self.raw = dict(q=joints, dq=velocities, T_base_ee=pose.as_matrix(), gripper_rad=gripper_rad,
                        gripper_calibrated=self.gripper is not None)
        return RobotState(joints, velocities, pose, position, 'base', joint_names=JOINT_NAMES,
                          timestamp_s=time.time(), embodiment=EMBODIMENT)

    def get_observation(self, camera_names=None):
        from franka_cap.devices import _check_intrinsics, calibration
        from ..cameras import rectify

        if not self.cameras:
            raise ApiError(ErrorCode.UNSUPPORTED, 'arx_calibrated_rgbd_required')
        names = list(self.cameras) if camera_names is None else list(camera_names)
        if not names or len(set(names)) != len(names) or any(name not in self.cameras for name in names):
            raise ValueError('invalid_camera_names')
        for name in names:
            self.cameras[name].latest()
        before = self.get_robot_state()
        stamp = time.monotonic()
        frames = {name: self.cameras[name].latest(after=stamp) for name in names}
        after = self.get_robot_state()
        cameras = {}
        for name, frame in frames.items():
            cal, transform, _ = calibration(self._calibration, name, self.raw)
            _check_intrinsics(frame.metadata, cal)
            if frame.metadata['serial'] != cal['camera_serial']:
                raise ValueError('camera_serial_mismatch')
            rgb, depth, intrinsics = rectify(frame)
            cameras[name] = CameraObservation(rgb, depth, intrinsics, name + '_color_optical',
                                               matrix_to_pose(transform, frame='base'), time.time())
        return Observation(cameras, after, timestamp_s=time.time())

    def solve_ik(self, target, seed=None):
        diagnostics = {}
        try:
            if isinstance(target, dict):
                target = Pose(**target)
            if not isinstance(target, Pose) or target.frame != 'base':
                raise ValueError('base_frame_pose_required')
            measured = self.get_robot_state().joint_positions['primary'] if seed is None else seed
            start = np.clip(self._joints(measured, measured=True),
                            self.kin.q_min, self.kin.q_max)
            relative = self.kin.T_home_inv.homogeneous @ target.as_matrix()
            rpy = Rotation.from_matrix(relative[:3, :3]).as_euler('xyz')
            joints, converged, residual = self.kin.ik(np.r_[relative[:3, 3], rpy], q_init=start)
            diagnostics = dict(sdk_converged=bool(converged), sdk_residual=float(residual),
                               residual_semantics='native_solver_residual_not_position_error_m')
            if not converged:
                raise ValueError('arx_ik_did_not_converge')
            return IKResult(True, joint_positions=self._joints(joints), diagnostics=diagnostics,
                            embodiment=EMBODIMENT)
        except (ValueError, TypeError) as error:
            return IKResult(False, error=ApiError(ErrorCode.IK_FAILED, str(error), details=diagnostics),
                            diagnostics=diagnostics, embodiment=EMBODIMENT)

    def _minimum_duration(self, start, target, speed_factor):
        duration = native_query(self.driver.arm, 'plan_duration', start=start, target=target,
                                minimum=0.)
        # Only an explicit caller multiplier changes the SDK-generated duration.
        return ceil(max(duration, TIME_QUANTUM_S, duration / speed_factor) / TIME_QUANTUM_S) * TIME_QUANTUM_S

    def plan_motion(self, goal, *, start_joints=None, speed_factor=None,
                    interpolation_dt_s=None, maximum_trajectory_dt_s=None):
        state = self.get_robot_state()
        # Encoder feedback can sit just beyond a nominal zero boundary. Keep
        # that measurement as the trajectory start and validate the goal strictly.
        start = self._joints(state.joint_positions['primary'] if start_joints is None else start_joints,
                             measured=True)
        if isinstance(goal, (Pose, dict)):
            ik = self.solve_ik(goal, start)
            if not ik.ok:
                raise ik.error
            goal = ik.joint_positions
        target = self._joints(goal)
        factor = resolve_speed_factor(speed_factor, self.speed_factor, self.max_speed_factor)
        duration = self._minimum_duration(start, target, factor) if interpolation_dt_s is None else interpolation_dt_s
        if maximum_trajectory_dt_s is not None and duration > maximum_trajectory_dt_s:
            raise ValueError('arx_point_to_point_exceeds_requested_sample_interval')
        trajectory = Trajectory(np.stack((start, target)), duration, JOINT_NAMES,
                                'arx_point_to_point', False, state, embodiment=EMBODIMENT)
        return trajectory, self.prepare_trajectory(trajectory, speed_factor=factor)

    def prepare_trajectory(self, trajectory, *, speed_factor=None):
        if (trajectory.embodiment != EMBODIMENT or tuple(trajectory.joint_names) != tuple(JOINT_NAMES)
                or trajectory.arm != 'primary' or len(trajectory.joint_positions) not in (1, 2)):
            raise ApiError(ErrorCode.UNSUPPORTED, 'arx_requires_point_to_point_trajectory')
        return self.prepare_segment(trajectory.joint_positions, trajectory.dt_s, speed_factor)

    def prepare_segment(self, positions, duration, speed_factor=None):
        start = self._joints(positions[0], measured=True)
        target = self._joints(positions[-1])
        factor = self.speed_factor if speed_factor is None else speed_factor
        if not np.array_equal(start, target):
            if not np.isfinite(duration) or duration <= 0:
                raise ValueError('arx_trajectory_duration_outside_limits')
        return dict(start_joints=start, target_joints=target, duration_s=duration,
                    target_pose=public(self._pose(target)), speed_factor=factor, max_speed_factor=self.max_speed_factor,
                    collision_checked=False, collision_scope='not_checked', automatic_scene=False)

    def check_execution(self, plan, *, state=None):
        if state is None:
            state = self.get_robot_state()
        self._joints(state.joint_positions['primary'], measured=True)
        return None

    def _command_with_feedback(self, arm, submit, *, channel='joints'):
        from .arx_support import preserve_support
        self.last_support_receipt = None
        try:
            if not submit():
                return None, None
            started = time.monotonic()
            while True:
                state = self.get_robot_state()
                reference = native_query(arm, 'reference_status')
                if channel == 'joints' and reference.get('reference_error'):
                    raise ApiError(ErrorCode.CONTROLLER_FAILED, 'arx_native_trajectory_error',
                                   recoverable=False, details=reference['reference_error'])
                if reference[channel + '_finished']:
                    return state, time.monotonic() - started
                time.sleep(TIME_QUANTUM_S)
        except Exception:
            self.last_support_receipt = preserve_support(self, cancel_motion=True)
            raise

    def execute(self, plan, trajectory):
        if not self.driver.allow_motion:
            return failure(ErrorCode.SAFETY_INTERLOCK, 'arx_motion_disabled')
        start, target, duration = plan['start_joints'], plan['target_joints'], plan['duration_s']
        if np.array_equal(start, target):
            return ExecutionResult(True, 0, diagnostics={'motion_completed': False, 'outcome': 'no_motion'})
        arm = self.driver.arm

        state, elapsed = self._command_with_feedback(arm,
            lambda: arm.set_joint_positions(positions=target, joint_names=JOINT_NAMES, duration=duration))
        if elapsed is None:
            return failure(ErrorCode.CONTROLLER_FAILED, 'arx_joint_target_rejected')
        if state is None:
            return ExecutionResult(False, 0, error=ApiError(ErrorCode.CONTROLLER_FAILED,
                'arx_joint_target_not_verified', recoverable=False),
                diagnostics=dict(support=self.last_support_receipt))
        source = 'synthetic_ARX_X5_2023' if self.synthetic else 'physical_ARX_X5_2023'
        receipt = dict(source=source, q=state.joint_positions['primary'],
                       T_base_ee=state.end_effector_poses['primary'].as_matrix(), elapsed_s=elapsed)
        accuracy = endpoint_accuracy(target, plan['target_pose'], receipt, joint_count=6)
        accuracy.update(accuracy_verified=None, acceptance_policy='sdk_reference_finished_deviation_reported')
        session = getattr(self, 'physical_session', None)
        if session is not None:
            session.arm_probe = True
        return ExecutionResult(True, len(trajectory.joint_positions),
            final_errors={'primary': accuracy['max_joint_error_rad']},
            diagnostics=dict(native_receipt=receipt, endpoint_accuracy=accuracy,
                             motion_completed=True, steps_unit='public_reference_samples'))

    def check_gripper(self, position):
        if self.gripper is None:
            return 'arx_gripper_calibration_required'
        self.get_robot_state()
        return None

    def set_gripper(self, position):
        if not self.driver.allow_motion or self.gripper is None:
            return failure(ErrorCode.SAFETY_INTERLOCK, 'arx_gripper_motion_unavailable')
        closed, opened = self.gripper
        target = closed + position * (opened - closed)
        arm = self.driver.arm
        state, elapsed = self._command_with_feedback(arm,
            lambda: arm.set_gripper_pos(target, duration=None), channel='gripper')
        if elapsed is None:
            return failure(ErrorCode.CONTROLLER_FAILED, 'arx_gripper_target_rejected')
        if state is None:
            return ExecutionResult(False, 0, error=ApiError(ErrorCode.CONTROLLER_FAILED,
                'arx_gripper_target_not_verified', recoverable=False),
                diagnostics=dict(support=self.last_support_receipt))
        return ExecutionResult(True, 1, diagnostics=dict(target_rad=target,
            measured_rad=self.raw['gripper_rad'], elapsed_s=elapsed, target_reached=None,
            physical_arrival_verified=False, completion='sdk_reference_finished'))

    def close(self):
        with ExitStack() as cleanup:
            cleanup.callback(self.driver.close)
            for owner in self.cameras.values():
                cleanup.callback(owner.close)
