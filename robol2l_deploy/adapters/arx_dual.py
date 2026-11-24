"""Encoder-only bimanual X5 station. One device owner, two native SDK loops."""
from contextlib import ExitStack, contextmanager
from dataclasses import replace
from pathlib import Path
import time

import numpy as np
from cap_harness.geometry import matrix_to_pose

from franka_cap.common import ROOT, hardware_lock, read_json
from .arx_collision import PairCollision
from .arx_grasp_geometry import GraspGeometry
from .arx_support import preserve_support
from .arx_payload import PayloadModels
from ..arx_preview import CameraPreviews, RGBDPreviews
from ..cameras import rectify
from ..contracts import CameraObservation
from ..contracts import (ExecutionResult, IKResult, Observation, Pose, RobotState, SynchronizedTrajectory,
                         Trajectory)
from ..errors import ApiError, ErrorCode
from ..runtime import failure, fingerprint
from .arx import (ARXDriver, ARXStationAdapter, JOINT_NAMES,
                  TIME_QUANTUM_S,
                  EMPTY_CLOSED_MAX_RAD)
from .arx import resolve_speed_factor
from .arx_native import query as native_query

EMBODIMENT = 'arx_x5_2023_dual'
SIDES = ('left', 'right')


class DualARXStation:
    synthetic = False
    model_name = 'X5-2023 pair'
    end_effector_link = 'link6'

    @classmethod
    def from_file(cls, path, **options):
        path = Path(path)
        return cls(read_json(path), directory=path.parent, **options)

    def __init__(self, cfg, *, directory, allow_connection=False, allow_motion=False, owner_socket=None):
        ports = cfg.get('arms', {})
        if (cfg.get('schema') != 1 or cfg.get('station') != 'arx-x5-2023-dual'
                or cfg.get('motion_enabled') is not False or set(ports) != set(SIDES)
                or set(ports.values()) != {'can1', 'can3'} or cfg.get('gripper')):
            raise ValueError('invalid_dual_arx_joint_station')
        hands = cfg.get('grippers', {})
        if not isinstance(hands, dict) or set(hands) - set(SIDES):
            raise ValueError('invalid_dual_grippers')
        self.commissioning_path = Path(directory) / cfg['commissioning'] if cfg.get('commissioning') else None
        self.commissioning = read_json(self.commissioning_path) if self.commissioning_path else {}
        self.station_home = self.commissioning.get('home')
        if self.commissioning_path:
            if self.commissioning['ports'] != ports:
                raise ValueError('commissioning_can_binding_changed')
            hands = {**hands, **self.commissioning['grippers']}
            if set(hands) - set(SIDES):
                raise ValueError('invalid_dual_grippers')
        self.collision = PairCollision(cfg.get('collision', {}), directory)
        geometry_path = Path(directory) / cfg['grasp_geometry'] if cfg.get('grasp_geometry') else None
        self.grasp_geometry = GraspGeometry(geometry_path, ports)
        self.unsupported_operations = {'mask_to_point_cloud', 'segment_text', 'segment_points',
                                       'localize_object', 'generate_grasps'}
        if not any(hands.values()):
            self.unsupported_operations.update(('open_gripper', 'close_gripper', 'set_gripper'))
            self.unsupported_operations.add('set_grippers')
        self._cfg = cfg
        settings = ROOT / '.franka-player/arx-preview-sources.json'
        self.previews = (RGBDPreviews(cfg['cameras'], settings) if cfg.get('cameras')
                         else CameraPreviews(cfg.get('preview_sources', {}), settings))
        self.ports = dict(ports)
        self.extrinsics = {}
        self.calibration_session = None
        if cfg.get('extrinsics'):
            report = read_json(Path(directory) / cfg['extrinsics'])
            if (report['ports'] != self.ports or report['cameras'] != cfg.get('cameras')
                    or report['convention'] != 'T_parent_from_child, translations in metres; optical frames use RGB axes'):
                raise ValueError('arx_extrinsics_binding_or_convention_mismatch')
            for name in ('T_left_base_from_right_base', 'T_left_base_from_global_optical',
                         'T_left_flange_from_left_wrist_optical', 'T_right_flange_from_right_wrist_optical'):
                matrix = np.asarray(report['transforms'][name], dtype=float)
                matrix_to_pose(matrix, frame='left_base')  # Validates SE(3), including handedness.
                self.extrinsics[name] = matrix
            self.calibration_session = report['session']
            self.unsupported_operations -= {'segment_text', 'segment_points', 'mask_to_point_cloud', 'localize_object'}
        payload_path = Path(directory) / cfg['payload'] if cfg.get('payload') else None
        self.payload = PayloadModels(payload_path, ports, cfg.get('cameras', {}), self.extrinsics)
        self.home_binding = fingerprint(dict(embodiment=EMBODIMENT, arms=ports))
        self._lock = ExitStack()
        self._locked = False
        self.arms = {}
        self.owner_client = None
        for side in SIDES:
            port = ports[side]
            driver = ARXDriver(port, allow_connection=allow_connection, allow_motion=allow_motion,
                               acquire_lock=self._station_lock, urdf_path=self.payload.paths.get(side),
                               urdf_sha256=self.payload.hashes.get(side))
            self.arms[side] = ARXStationAdapter(dict(can_port=port, motion=cfg.get('motion', {}), gripper=hands.get(side)),
                                                directory=directory, driver=driver)
        if self.station_home:
            home = self.station_home
            if (home['device_binding'] != self.home_binding or home['embodiment'] != EMBODIMENT
                    or home['joint_names'] != {side: JOINT_NAMES for side in SIDES}):
                raise ValueError('home_robot_identity_changed')
            self.reset_targets(home['joint_positions'])
        if owner_socket:
            from ..arx_owner import OwnerClient
            self.owner_client = OwnerClient(owner_socket, self.device_binding())
            for side, unit in self.arms.items():
                unit.driver.owner = self.owner_client
                unit.driver.owner_side = side

    @contextmanager
    def _station_lock(self):
        # Keep one process-wide lock until both SDK handles have been closed.
        if not self._locked:
            self._lock.enter_context(hardware_lock())
            self._locked = True
        yield

    @property
    def driver(self):
        # PhysicalSession controls the station's shared motion permission.
        return self

    @property
    def allow_motion(self):
        return all(unit.driver.allow_motion for unit in self.arms.values())

    @allow_motion.setter
    def allow_motion(self, value):
        if not value:
            for unit in self.arms.values():
                unit.driver.allow_motion = False
        if self.owner_client is not None:
            # Revocation before first connection must not create hardware just to disable it.
            if value:
                self.owner_client.call('authorize', enabled=True)
            elif self.owner_client.token and not self.owner_client.lost:
                try:
                    self.owner_client.call('authorize', enabled=False)
                except Exception:
                    self.owner_client.lost = True
                    self.owner_client.done.set()  # Owner lease expiry cancels motion independently.
        for unit in self.arms.values():
            unit.driver.allow_motion = value

    def realtime_ready(self):
        return all(unit.driver.realtime_ready() for unit in self.arms.values())

    @property
    def cameras(self):
        return self.previews.cameras() if isinstance(self.previews, RGBDPreviews) else {}

    def configuration(self):
        return dict(station=self._cfg, robot_backend=EMBODIMENT)

    def describe(self):
        return dict(embodiment=EMBODIMENT, robot_backend=EMBODIMENT, synthetic=self.synthetic,
                    arms=list(SIDES), arm_aliases={'primary': 'left', 'secondary': 'right'},
                    can_ports=self.ports, cameras=list(self.cameras), camera_extrinsics_calibrated=bool(self.extrinsics),
                    perception_frame='left_base' if self.extrinsics else None,
                    calibration_session=self.calibration_session, gripper_calibrated=all(unit.gripper for unit in self.arms.values()),
                    calibrated_grippers=[side for side, unit in self.arms.items() if unit.gripper],
                    grasp_geometry=self.grasp_geometry.describe(SIDES),
                    payload_model=self.payload.describe(),
                    hardware_lifecycle=self.hardware_lifecycle(),
                    grasp_geometry_ready=len(self.grasp_geometry.transforms) == len(SIDES),
                    end_effector_references=self.end_effector_references,
                    empty_closed_max_rad=EMPTY_CLOSED_MAX_RAD,
                    gripper_closed_validation=self.commissioning.get('gripper_validation', {}).get('sides', {}),
                    collision_available=not self.collision.missing, collision_missing=self.collision.missing,
                    contact_grasp=False,
                    gripper_description='Normalized opening 0=close attempt, 1=open; native reference completion with measured encoder feedback; physical arrival and object possession are not inferred',
                    observation_mode='calibrated_rgbd' if self.extrinsics else 'joint_feedback',
                    shared_base_calibrated=bool(self.extrinsics) or len(self.collision.base) == 2,
                    motion_speed=self.arms['left'].describe()['motion_speed'])

    def calibration_binding(self):
        config = self.configuration()
        config['station'] = {k: v for k, v in self._cfg.items() if k not in ('preview_sources','extrinsics','commissioning','grasp_geometry','payload')}
        if isinstance(self.previews, RGBDPreviews):
            config['camera_bindings'] = {k: v.source.serial for k, v in self.cameras.items()}
        if not self.collision.missing:
            config['collision_geometry_digest'] = self.collision.geometry_digest
        return fingerprint(config)

    def device_binding(self):
        physical = self.calibration_binding()
        if self.payload.record:
            physical = fingerprint(dict(station=physical, payload=self.payload.record))
        if self.commissioning.get('grippers'):
            physical = fingerprint(dict(station=physical, grippers=self.commissioning['grippers']))
        if self.grasp_geometry.records:
            physical = fingerprint(dict(station=physical, grasp_geometry=self.grasp_geometry.records))
        if not self.extrinsics:
            return physical
        return fingerprint(dict(station=physical, transforms=self.extrinsics, session=self.calibration_session))

    binding = device_binding

    @property
    def end_effector_references(self):
        return {side: self.grasp_geometry.reference(side) for side in SIDES}

    def hardware_lifecycle(self):
        return dict(mode='resident_arx_owner_v1' if self.owner_client else 'in_process',
                    disconnect_requires_support_confirmation=not bool(self.owner_client),
                    maintenance_requires_support_confirmation=True,
                    owner_instance=self.owner_client.instance if self.owner_client else None)

    def prepare_disconnect(self, *, hardware_supported=False):
        if self.owner_client:
            self.owner_client.close(hardware_supported=hardware_supported)
        elif any(isinstance(unit.driver, ARXDriver) and unit.driver._arm is not None
                 for unit in self.arms.values()) and not hardware_supported:
            raise ValueError('hardware_support_confirmation_required')
        self._disconnect_supported = hardware_supported

    def close(self, *, hardware_supported=False):
        hardware_supported = hardware_supported or getattr(self, '_disconnect_supported', False)
        if self.owner_client and not self.owner_client.done.is_set():
            self.owner_client.close(hardware_supported=hardware_supported)
        elif not self.owner_client and not hardware_supported and any(isinstance(unit.driver, ARXDriver) and unit.driver._arm is not None
                                            for unit in self.arms.values()):
            raise RuntimeError('hardware_support_confirmation_required')
        try:
            self._close_arms(hardware_supported=hardware_supported)
        finally:
            self.previews.close()

    def _close_arms(self, *, hardware_supported=False):
        error = None
        for unit in self.arms.values():
            try:
                if isinstance(unit.driver, ARXDriver):
                    unit.driver.close(hardware_supported=hardware_supported)
                else:
                    unit.close()
            except Exception as failure:
                error = error or failure
        if error:
            raise error  # Keep the station lock when closing a native handle is unconfirmed.
        self._lock.close()
        self._locked = False

    def get_robot_state(self):
        states = {}
        for side, unit in self.arms.items():
            try:
                states[side] = unit.get_robot_state()
            except Exception as error:
                # A peer connection failure must not remove support from a healthy arm.
                details = dict(error.details) if isinstance(error, ApiError) else {}
                raise ApiError(ErrorCode.SAFETY_INTERLOCK, f'{side}: {error}',
                               details=dict(details, arm=side, port=self.ports[side])) from error
        def values(field):
            return {side: getattr(state, field)['primary'] for side, state in states.items()}
        poses = {side: self.grasp_geometry.from_flange(side, replace(pose, frame=side + '_base'))
                 for side, pose in values('end_effector_poses').items()}
        if self.extrinsics:
            poses['right'] = self.shared_flange('right', poses['right'].as_matrix())
        return RobotState(values('joint_positions'), values('joint_velocities'), poses,
                          values('gripper_positions'), 'left_base' if self.extrinsics else 'arm_local', values('joint_names'),
                          time.time(), EMBODIMENT)

    def get_observation(self, camera_names=None):
        owners = self.cameras
        if isinstance(camera_names, str):
            camera_names = [camera_names]
        names = list(self._cfg.get('cameras', {})) if camera_names is None else list(camera_names)
        if len(set(names)) != len(names) or any(name not in owners for name in names):
            raise ValueError('requested_camera_not_bound')
        for name in names:
            owners[name].start()
        for name in names:
            owners[name].latest()
        before = self.get_robot_state() if self.extrinsics and names else None
        stamp = time.monotonic()
        frames = {}
        for name in names:
            cursor = stamp
            while True:
                frame = owners[name].latest(after=cursor)
                if before is None or frame.metadata['captured_s'] >= before.timestamp_s:
                    break
                cursor = frame.monotonic_s
            frames[name] = frame
        state = self.get_robot_state()
        cameras = {}
        for name, frame in frames.items():
            rgb, depth, k = rectify(frame)
            optical = name + '_color_optical'
            captured = frame.metadata['captured_s']
            if self.extrinsics:
                if frame.metadata['serial'] != self._cfg['cameras'][name]:
                    raise ValueError(f'{name}: calibrated_camera_serial_mismatch')
                interval = state.timestamp_s - before.timestamp_s
                if not (0 < interval and before.timestamp_s <= captured <= state.timestamp_s):
                    raise ValueError(f'{name}: camera_state_timing_mismatch')
                local_flange = None
                if name != 'global':
                    side = name.removesuffix('_wrist')
                    q0, q1 = before.joint_positions[side], state.joint_positions[side]
                    speeds = np.stack([s.joint_velocities[side] for s in (before,state)])
                    alpha = (captured - before.timestamp_s) / interval
                    local_flange = self.arms[side]._pose(q0 + alpha*(q1-q0)).as_matrix()
                pose = self.camera_pose(name, local_flange)
            else:
                pose = Pose([0., 0., 0.], [1., 0., 0., 0.], optical)
            cameras[name] = CameraObservation(rgb, depth, k, optical, pose, captured)
        self.observation_diagnostics = {name: dict(frame.metadata, extrinsics_calibrated=bool(self.extrinsics),
                                                  calibration_session=self.calibration_session)
                                        for name, frame in frames.items()}
        return Observation(cameras, state, timestamp_s=state.timestamp_s)

    def shared_flange(self, side, local_flange):
        matrix = self.extrinsics['T_left_base_from_right_base'] @ local_flange if side == 'right' else local_flange
        return matrix_to_pose(matrix, frame='left_base')

    def camera_pose(self, name, local_flange=None):
        if name == 'global':
            matrix = self.extrinsics['T_left_base_from_global_optical']
        else:
            side = name.removesuffix('_wrist')
            matrix = self.shared_flange(side, local_flange).as_matrix() @ self.extrinsics[f'T_{side}_flange_from_{side}_wrist_optical']
        return matrix_to_pose(matrix, frame='left_base')

    def check_teach_parked(self, first, state):
        for side in SIDES:
            q = self.arms[side]._joints(state.joint_positions[side], measured=True)
            velocity = np.asarray(state.joint_velocities[side])
            if velocity.shape != (6,) or not np.isfinite(velocity).all():
                raise ValueError(side + ': invalid_joint_feedback')

    def check_teach_start(self, side):
        unit = self.arms[side]
        if not unit.driver.allow_motion:
            raise ValueError('arx_motion_disabled')
        state = unit.get_robot_state()
        speed = np.abs(state.joint_velocities['primary'])
        if not np.isfinite(speed).all():
            raise ValueError('invalid_joint_feedback')

    def teach_drag(self, side):
        unit = self.arms[side]
        if not unit.driver.allow_motion:
            raise ValueError('arx_motion_disabled')
        if not unit.driver.arm.gravity_compensation():
            raise RuntimeError('gravity_compensation_not_confirmed')

    def _teach_feedback(self, side):
        # One SDK state snapshot: no separate q/v RPCs or FK in the dwell loop.
        status = self.arms[side].driver.arm.get_status()
        if (status.get('fault') or status.get('offline') or status.get('gripper_offline')
                or status.get('overlimit')):
            raise RuntimeError('arx_fault_or_offline: ' + str(status))
        q, v = (np.asarray(status[key], dtype=float) for key in ('q', 'v'))
        if q.shape != (6,) or v.shape != (6,) or not np.isfinite([q, v]).all():
            raise ValueError('invalid_arx_hold_feedback')
        return status, q, v

    def teach_hold(self, side, *, parked_reference=None):
        unit = self.arms[side]
        if not unit.driver.allow_motion:
            raise ValueError('arx_motion_disabled')
        initial, measured, velocity = self._teach_feedback(side)
        if initial.get('mode') not in ('JOINT', 'GRAVITY'):
            raise RuntimeError('hold_requires_healthy_joint_or_gravity_mode')
        target = self._target(side, measured, measured, [])
        started = time.monotonic()
        state, elapsed = unit._command_with_feedback(unit.driver.arm,
            lambda: unit.driver.arm.set_joint_positions(positions=target, joint_names=JOINT_NAMES, duration=0.))
        if state is None:
            raise RuntimeError('hold_command_not_confirmed')
        status, q, v = self._teach_feedback(side)
        if status.get('mode') != 'JOINT':
            raise RuntimeError('native_position_control_not_active')
        return dict(target_rad=target.tolist(), measured_rad=q.tolist(),
                    max_error_rad=float(np.max(np.abs(q-target))), max_speed_rad_s=float(np.max(np.abs(v))),
                    command_s=time.monotonic()-started, initial_mode=initial.get('mode'),
                    completion='sdk_reference_finished', hold_verified=False,
                    physical_arrival_verified=False, stable_s=None, samples=[status])

    def solve_ik(self, pose, seed=None, *, arm):
        if arm == 'right' and isinstance(pose, Pose) and pose.frame == 'left_base' and self.extrinsics:
            pose = matrix_to_pose(np.linalg.solve(self.extrinsics['T_left_base_from_right_base'],pose.as_matrix()),
                                  frame='right_base')
        if arm not in self.arms or not isinstance(pose, Pose) or pose.frame != arm + '_base':
            return IKResult(False, arm=arm, embodiment=EMBODIMENT,
                            error=ApiError(ErrorCode.INVALID_REQUEST, f'{arm}: expected {arm}_base pose'))
        flange = self.grasp_geometry.to_flange(arm, pose)
        result = self.arms[arm].solve_ik(replace(flange, frame='base'), seed)
        return replace(result, arm=arm, embodiment=EMBODIMENT)

    def _target(self, side, goal, measured, adjustments):
        unit = self.arms[side]
        q = unit._joints(goal, measured=True).copy()
        bounded = np.clip(q, unit.kin.q_min, unit.kin.q_max)
        for index in np.flatnonzero((q == measured) & (q != bounded)):
            adjustments.append(dict(arm=side, joint=JOINT_NAMES[index], measured_rad=q[index],
                                    target_rad=bounded[index]))
            q[index] = bounded[index]
        return unit._joints(q)

    def _plan(self, state, targets, *, synchronized, speed_factor=None, interpolation_dt_s=None,
              maximum_trajectory_dt_s=None, start_joints=None):
        rows, minimum, adjustments = {}, TIME_QUANTUM_S, []
        reference = self.arms['left']
        factor = resolve_speed_factor(speed_factor, reference.speed_factor, reference.max_speed_factor)
        for side, goal in targets.items():
            unit = self.arms[side]
            try:
                start = unit._joints(state.joint_positions[side] if start_joints is None else start_joints,
                                     measured=True)
                if isinstance(goal, Pose):
                    solution = self.solve_ik(goal, start, arm=side)
                    if not solution.ok:
                        raise solution.error
                    goal = solution.joint_positions
                target = self._target(side, goal, state.joint_positions[side], adjustments)
                minimum = max(minimum, unit._minimum_duration(start, target, factor))
                rows[side] = np.stack((start, target))
            except ValueError as error:
                raise ValueError(f'{side}: {error}') from error
        duration = minimum if interpolation_dt_s is None else interpolation_dt_s
        if maximum_trajectory_dt_s is not None and duration > maximum_trajectory_dt_s:
            raise ValueError('arx_point_to_point_exceeds_requested_sample_interval')
        if synchronized:
            trajectory = SynchronizedTrajectory(rows, duration, state.joint_names, 'arx_joint_pair',
                                                False, state, embodiment=EMBODIMENT)
        else:
            side, = rows
            trajectory = Trajectory(rows[side], duration, JOINT_NAMES, 'arx_joint_pair',
                                    False, state, arm=side, embodiment=EMBODIMENT)
        payload = self.prepare_trajectory(trajectory, speed_factor=factor, measured_state=state)
        payload['boundary_adjustments'] = adjustments
        return replace(trajectory, collision_aware=payload['collision_checked']), payload

    def reset_targets(self, recorded, adjustments=None):
        # These targets were themselves measurements in the captured snapshot.
        adjustments = [] if adjustments is None else adjustments
        return {side: self._target(side, recorded[side], np.asarray(recorded[side]), adjustments)
                for side in SIDES}

    def plan_recorded_return(self, state, recorded, arm):
        corrections = []
        goals = self.reset_targets(recorded, corrections)
        selected = goals if arm == 'both' else {arm: goals[arm]}
        trajectory, payload = self._plan(state, selected, synchronized=arm == 'both')
        payload['boundary_adjustments'].extend(item for item in corrections if item['arm'] in selected)
        return trajectory, payload

    def plan_motion(self, goal, *, arm, **options):
        return self._plan(self.get_robot_state(), {arm: goal}, synchronized=False, **options)

    def plan_pair(self, state, goals, **options):
        if set(goals) != set(SIDES):
            raise ValueError('synchronized_targets_must_name_left_and_right')
        return self._plan(state, goals, synchronized=True, speed_factor=options.get('time_dilation_factor'),
                          interpolation_dt_s=options.get('interpolation_dt_s'),
                          maximum_trajectory_dt_s=options.get('maximum_trajectory_dt_s'))

    def prepare_trajectory(self, trajectory, speed_factor=None, *, measured_state=None):
        rows = (trajectory.joint_positions if isinstance(trajectory, SynchronizedTrajectory)
                else {trajectory.arm: trajectory.joint_positions})
        if (trajectory.embodiment != EMBODIMENT or set(trajectory.expected_start.arms) != set(SIDES)
                or not set(rows) <= set(SIDES)):
            raise ValueError('invalid_dual_arx_trajectory')
        count = len(next(iter(rows.values())))
        dt = trajectory.dt_s
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('arx_invalid_segment_duration')
        if measured_state is None:
            measured_state = self.get_robot_state()
        expected = dict(trajectory.expected_start.joint_positions)
        for side, q in expected.items():
            self.arms[side]._joints(q, measured=True)
        hands = trajectory.gripper_positions
        if hands is None:
            hands = {}
        elif isinstance(trajectory, Trajectory):
            hands = {trajectory.arm: hands}
        for side in hands:
            if not self.arms[side].gripper:
                raise ValueError(f'{side}: gripper_calibration_required')
        stages, adjustments = [], []
        previous = expected.copy()
        # For N>1 row zero is the declared start. A single row is a target.
        for side, positions in rows.items():
            if tuple(trajectory.expected_start.joint_names[side]) != tuple(JOINT_NAMES):
                raise ValueError(f'{side}: unexpected_joint_order')
            if count > 1:
                previous[side] = self.arms[side]._joints(positions[0], measured=True)
        for index in range(int(count > 1), count):
            targets = previous.copy()
            segments = {}
            for side, positions in rows.items():
                try:
                    unit = self.arms[side]
                    measured = measured_state.joint_positions[side]
                    # A constructed trajectory may contain a caller-supplied
                    # expected_start. Only a held encoder reading can be clipped.
                    boundary = np.clip(measured, unit.kin.q_min, unit.kin.q_max)
                    held = (previous[side] == measured) | (previous[side] == boundary)
                    targets[side] = self._target(side, positions[index], np.where(held, measured, np.nan), adjustments)
                    segments[side] = self.arms[side].prepare_segment(
                        [previous[side], targets[side]], trajectory.dt_s, speed_factor)
                except ValueError as error:
                    raise ValueError(f'{side}, waypoint {index}: {error}') from error
            checked = self._check_collision(previous, targets, trajectory.dt_s, required=trajectory.collision_aware)
            stages.append(dict(segments=segments, expected=previous, duration_s=trajectory.dt_s,
                               waypoint=index, hands={side: float(values[index]) for side, values in hands.items()
                                                      if index == 0 or values[index] != values[index-1]}))
            previous = targets
        return dict(**{k: stages[0][k] for k in ('segments', 'expected', 'duration_s')},
                    stages=stages, waypoint_count=count, boundary_adjustments=adjustments,
                    initial_hands={s: float(v[0]) for s, v in hands.items()} if count > 1 else {},
                    speed_factor=next(iter(stages[0]['segments'].values()))['speed_factor'],
                    end_effector_references=self.end_effector_references,
                    collision_checked=checked, collision_missing=list(self.collision.missing))

    def check_execution(self, plan):
        measured, targets = {}, {}
        for side, unit in self.arms.items():
            state = unit.get_robot_state()
            measured[side] = state.joint_positions['primary']
            targets[side] = measured[side]
            segment = plan['segments'].get(side)
            if segment is not None:
                targets[side] = segment['target_joints']
            if segment is not None and not np.array_equal(segment['start_joints'], segment['target_joints']):
                error = unit.check_execution(segment, state=state)
                if error:
                    return f'{side}: {error}'
        try:
            self._check_collision(measured, targets, plan['duration_s'])
        except ValueError as error:
            return str(error)
        return None

    def _check_collision(self, start, target, duration, *, required=False):
        paths = None
        if not self.collision.missing:
            paths = {side: native_query(self.arms[side].driver.arm, 'joint_path',
                                       start=start[side], target=target[side], minimum=duration)
                     for side in SIDES}
        return self.collision.check(start, target, required=required, paths=paths)

    def calibration_increment(self, side, target, parked, before):
        if not self.allow_motion:
            raise ValueError("arx_motion_disabled")
        unit = self.arms[side]
        arm = unit.driver.arm
        state, elapsed = unit._command_with_feedback(arm, lambda: arm.set_gripper_pos(target, duration=None), channel='gripper')
        if state is None:
            raise RuntimeError('gripper_command_rejected')
        self.get_robot_state()
        after = {name: item.raw['gripper_rad'] for name, item in self.arms.items()}
        return ExecutionResult(True, 1, diagnostics=dict(side=side, target_rad=target,
            measured_rad=after[side], before_rad=before, after_rad=after, elapsed_s=elapsed,
            physical_arrival_verified=False, completion='sdk_reference_finished',
            empty_closed_verified=target <= EMPTY_CLOSED_MAX_RAD and 0 <= after[side] <= EMPTY_CLOSED_MAX_RAD))


    def check_hands(self, commands):
        for side, value in commands.items():
            if side not in self.arms or self.arms[side].gripper is None:
                raise ValueError(f'{side}: gripper_calibration_required')
            if isinstance(value, bool) or not isinstance(value, (float, int, np.number)) or not 0 <= value <= 1:
                raise ValueError(f'{side}: gripper_position_outside_0_1')
        self.get_robot_state()

    def move_hands(self, commands, permitted):
        self.check_hands(commands)
        receipts = {side: dict(dispatch='not_sent') for side in commands}
        phase = 'submission'
        failed_sides = []
        try:
            if not self.allow_motion or not permitted():
                return failure(ErrorCode.CANCELLED, 'paused_before_gripper_dispatch')
            for side, value in commands.items():
                failed_sides = [side]
                unit = self.arms[side]
                closed, opened = unit.gripper
                target = closed + value * (opened - closed)
                row = receipts[side]
                row.update(target_rad=target, initial_rad=unit.raw['gripper_rad'], dispatch='unknown',
                           grasp_verified=False, contact_detected=None)
                if not unit.driver.arm.set_gripper_pos(target, duration=None):
                    row['dispatch'] = 'rejected'
                    raise RuntimeError(f'{side}: gripper_target_rejected')
                row['dispatch'] = 'accepted'
            phase = 'feedback'
            while True:
                pending = []
                for side, row in receipts.items():
                    failed_sides = [side]
                    unit = self.arms[side]
                    unit.get_robot_state()
                    velocity = float(unit.driver.arm.get_gripper_vel())
                    if not np.isfinite(velocity):
                        raise ValueError(f'{side}: invalid_arx_gripper_velocity')
                    row.update(measured_rad=unit.raw['gripper_rad'],
                               error_rad=abs(unit.raw['gripper_rad']-row['target_rad']),
                               measured_velocity_rad_s=velocity,
                               target_reached=None, physical_arrival_verified=False,
                               completion='sdk_reference_finished', grasp_verified=False)
                    if commands[side] == 0:
                        row.update(closing_travel_rad=row['initial_rad']-row['measured_rad'],
                                   empty_closed_verified=0 <= row['measured_rad'] <= EMPTY_CLOSED_MAX_RAD)
                    if not native_query(unit.driver.arm, 'reference_status')['gripper_finished']:
                        pending.append(side)
                        row['completion'] = 'pending'
                if not pending:
                    break
                time.sleep(TIME_QUANTUM_S)
        except Exception as exc:
            for side, unit in self.arms.items():
                receipts.setdefault(side, {})['support'] = preserve_support(unit, cancel_motion=False)
            return ExecutionResult(False, 0, error=ApiError(ErrorCode.CONTROLLER_FAILED, str(exc), recoverable=False),
                                   diagnostics=dict(grippers=receipts, phase=phase, failed_arms=failed_sides,
                                                    outcome='unverified', steps_count_known=False))
        return ExecutionResult(True, 1, diagnostics=dict(grippers=receipts))

    def verify_goal(self, execution, targets, tolerance):
        checks, errors = {}, {}
        for side, goal in targets.items():
            row = execution.diagnostics['per_arm'][side]
            if isinstance(goal, Pose):
                current = row['measured_pose']
                if side == 'right' and goal.frame == 'left_base' and self.extrinsics:
                    current = self.shared_flange(side, current.as_matrix())
                distance = float(np.linalg.norm(current.position - goal.position))
                angle = float(2 * np.arccos(np.clip(abs(current.quaternion_wxyz @ goal.quaternion_wxyz), 0., 1.)))
                checks[side] = dict(position_error_m=distance, position_tolerance_m=tolerance,
                                    orientation_error_rad=angle, orientation_tolerance_rad=np.deg2rad(10.),
                                    reached=distance <= tolerance and angle <= np.deg2rad(10.))
                errors[side] = distance
            else:
                delta = float(np.max(np.abs(row['measured_joints'] - goal)))
                checks[side] = dict(joint_error_rad=delta, joint_tolerance_rad=tolerance, reached=delta <= tolerance)
                errors[side] = delta
        ok = all(item['reached'] for item in checks.values())
        error = None if ok else ApiError(ErrorCode.GOAL_TOLERANCE_NOT_MET, 'arx_goal_tolerance_not_met', details=checks)
        return replace(execution, ok=ok, status=None, error=error, final_errors=errors,
                       diagnostics=dict(execution.diagnostics, goal_checks=checks))

    def execute(self, plan, trajectory, *, permitted=lambda: True, targets=None, tolerance=None):
        completed = 0
        receipts = []
        last = None

        def finish(result, waypoint):
            diagnostics = dict(result.diagnostics, boundary_adjustments=plan.get('boundary_adjustments', []),
                               completed_waypoints=completed,
                               waypoint=waypoint, waypoint_receipts=receipts)
            if 'per_arm' in diagnostics:
                diagnostics['per_arm'] = {
                    side: dict(row, delta_rad=row['measured_joints'] - plan['expected'][side])
                    if 'measured_joints' in row else row for side, row in diagnostics['per_arm'].items()}
            return replace(result, steps_executed=completed, diagnostics=diagnostics)

        def move_hands(commands, index):
            if not commands:
                return None
            result = self.move_hands(commands, permitted)
            receipts.append(dict(waypoint=index, grippers=result.diagnostics.get('grippers', {}), ok=result.ok))
            return None if result.ok else result

        hand_failure = move_hands(plan['initial_hands'], 0)
        if hand_failure is not None:
            return finish(hand_failure, 0)
        for stage in plan['stages']:
            index = stage['waypoint']
            if not permitted() or not self.allow_motion:
                return finish(failure(ErrorCode.CANCELLED, 'paused_before_segment'), index)
            reason = self.check_execution(stage)
            if reason:
                return finish(failure(ErrorCode.STALE_PLAN, reason), index)
            if index == 1:
                completed = 1
            final_stage = stage is plan['stages'][-1]
            last = self._execute_segment(stage, targets=targets if final_stage else None,
                                         tolerance=tolerance if final_stage else None)
            receipts.append(dict(waypoint=index, per_arm=last.diagnostics.get('per_arm', {}),
                                 ok=last.ok))
            if not last.ok:
                if last.error.code == ErrorCode.GOAL_TOLERANCE_NOT_MET:
                    completed = index + 1
                return finish(last, index)
            hand_failure = move_hands(stage['hands'], index)
            if hand_failure is not None:
                return finish(hand_failure, index)
            completed = index + 1
        return finish(last, plan['stages'][-1]['waypoint'])

    def _execute_segment(self, plan, *, targets=None, tolerance=None):
        if not self.allow_motion:
            return failure(ErrorCode.SAFETY_INTERLOCK, 'arx_motion_disabled')
        # Native set_joint_positions queues a duration-based motion and returns.
        # Dispatch on the owner thread before polling either arm.
        segments = plan['segments']
        receipts = {side: {'dispatch': 'not_sent', 'port': self.ports[side]} for side in SIDES}
        start_times = []
        errors = {}
        phase = 'submission'
        failed_sides = []
        accuracy_targets = targets
        accuracy_tolerance = tolerance

        def measured_result():
            result = ExecutionResult(True, 2 if start_times else 0, final_errors=errors.copy(),
                diagnostics=dict(per_arm=receipts, motion_completed=bool(start_times),
                                 support_strategy='retain_position_control',
                                 submission_skew_s=max(start_times)-min(start_times) if start_times else 0.,
                                 duration_s=plan['duration_s']))
            result = replace(result, diagnostics=dict(result.diagnostics,
                completion='sdk_reference_finished', physical_arrival_verified=False))
            return self.verify_goal(result, accuracy_targets, accuracy_tolerance) if accuracy_targets is not None else result
        try:
            for side, segment in segments.items():
                if np.array_equal(segment['start_joints'], segment['target_joints']):
                    continue
                failed_sides = [side]
                arm = self.arms[side].driver.arm
                receipts[side]['dispatch'] = 'unknown'
                start_times.append(time.monotonic())
                if not arm.set_joint_positions(positions=segment['target_joints'], joint_names=JOINT_NAMES,
                                               duration=plan['duration_s']):
                    receipts[side]['dispatch'] = 'rejected'
                    raise RuntimeError(f'{side}: arx_joint_target_rejected')
                receipts[side]['dispatch'] = 'accepted'
            phase = 'feedback'
            while True:
                pending = []
                for side, unit in self.arms.items():
                    failed_sides = [side]
                    state = unit.get_robot_state()
                    q = state.joint_positions['primary']
                    initial = plan['expected'][side]
                    goal = segments[side]['target_joints'] if side in segments else initial
                    errors[side] = float(np.max(np.abs(q - goal)))
                    local_flange = replace(state.end_effector_poses['primary'], frame=side+'_base')
                    receipts[side].update(measured_joints=q,
                                          measured_velocities=state.joint_velocities['primary'],
                                          feedback_timestamp_s=state.timestamp_s,
                                          measured_pose=self.grasp_geometry.from_flange(side, local_flange),
                                          measured_flange_pose=local_flange,
                                          pose_reference=self.grasp_geometry.reference(side),
                                          delta_rad=q - initial, max_error_rad=errors[side])
                    if receipts[side]['dispatch'] == 'accepted':
                        reference = native_query(unit.driver.arm, 'reference_status')
                        receipts[side]['native_reference'] = reference
                        if not reference['joints_finished']:
                            pending.append(side)
                if not pending:
                    for side in segments:
                        if receipts[side]['dispatch'] == 'accepted':
                            status = self.arms[side].driver.arm.get_status()
                            receipts[side]['native_mode'] = status.get('mode')
                            if status.get('mode') != 'JOINT' or status.get('fault') or status.get('offline'):
                                failed_sides = [side]
                                raise RuntimeError(f'{side}: arx_position_control_not_healthy')
                    return measured_result()
                time.sleep(TIME_QUANTUM_S)
        except Exception as error:
            for side, unit in self.arms.items():
                receipts[side]['support'] = preserve_support(unit, cancel_motion=(
                    receipts[side]['dispatch'] in ('accepted', 'unknown') or
                    side in failed_sides and phase == 'feedback'))
            partial = any(r['dispatch'] == 'accepted' for r in receipts.values()) and phase == 'submission'
            return ExecutionResult(False, 0, error=ApiError(ErrorCode.CONTROLLER_FAILED, str(error), recoverable=False),
                                   diagnostics=dict(outcome='partial_dispatch' if partial else 'unverified',
                                                    phase=phase, failed_arms=failed_sides, per_arm=receipts, steps_count_known=False))
