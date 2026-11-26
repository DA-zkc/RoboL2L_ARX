"""Single station owner. Adapters alone touch devices; no generated code runs here."""
from contextlib import contextmanager
from dataclasses import fields, is_dataclass, replace
from collections.abc import Mapping
from enum import Enum
from pathlib import Path
import hashlib
import json
import os
import threading
import time
from uuid import uuid4
import numpy as np
from .contracts import SynchronizedPlanResult, ExecutionResult, PlanResult, IKResult, Pose, Trajectory, SynchronizedTrajectory
from .errors import ApiError, ErrorCode
from .execution_feedback import stops_runtime, feedback, execution_value, endpoint_accuracy, motion_not_started
from .adapters.arx_gripper_feedback import ARX_GRIPPER_TIMING


def public(value):
    if isinstance(value,ApiError):
        from .localization import chinese_error
        return dict(code=value.code.value,message=value.message,message_zh=chinese_error(value.message,value.details),recoverable=value.recoverable,details=public(value.details))
    if is_dataclass(value): return {f.name: public(getattr(value, f.name)) for f in fields(value)}
    if isinstance(value, Mapping):
        result = {str(k): public(v) for k, v in value.items()}
        if 'code' in result and isinstance(result.get('message'),str):
            from .localization import chinese_error
            result['message_zh'] = chinese_error(result['message'], result.get('details'))
        return result
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, Enum): return value.value
    if isinstance(value, (tuple, list)): return [public(v) for v in value]
    return value


def fingerprint(value):
    return hashlib.sha256(json.dumps(public(value), sort_keys=True, allow_nan=False).encode()).hexdigest()


def failure(code, message):
    return ExecutionResult(False, 0, error=ApiError(code, message))


class OutcomeUnknown(RuntimeError):
    """A dispatch occurred but no trustworthy terminal receipt is available."""
    def __init__(self, message, *, diagnostics=None):
        super().__init__(message)
        self.diagnostics = diagnostics or {}

def teaching_fault(side, error):
    if error.startswith('arx_joint_target_outside_limits:'):
        return dict(kind='limit', side=side, error=error)
    if error == 'hold_feedback_not_verified' or error.startswith('hold_feedback_not_verified:'):
        return dict(kind='feedback', side=side, error=error)
    return None


class StationRuntime:
    def __init__(self, adapter, directory, *, allow_motion=False):
        self.adapter = adapter
        self.directory = Path(directory); self.directory.mkdir(parents=True, exist_ok=True)
        self.allow_motion = allow_motion
        self.physical_session = None
        self.paused = False; self.faulted = False; self.closed = False
        self._execution_abort = threading.Event()
        self.connection = dict(status='not_connected', robot_state=None, cameras={})
        self.last_execution = None
        self.home_path = self.directory / "home.json"
        self.home = json.loads(self.home_path.read_text()) if self.home_path.exists() else None
        station_home = getattr(adapter, 'station_home', None)
        if station_home is not None and self.home != station_home:
            from .cap.storage import write_json
            self.home = station_home
            write_json(self.home_path, self.home)
        self.plans = {}; self.observations = {}; self.jobs = {}
        self._gate = threading.Lock(); self._mutex = threading.RLock(); self._local = threading.local()
        path = self.directory / 'jobs.json'
        if path.exists():
            self.jobs = json.loads(path.read_text())
            for job in self.jobs.values():
                if job['status'] in ('running', 'accepted'):
                    job['status'] = 'outcome_unknown' if job.get('dispatch') == 'sent' else 'interrupted'
                if job['status'] == 'outcome_unknown' or (job.get('dispatch')=='sent' and stops_runtime(job.get('result',{}))): self.faulted = True
        # CaP invokes the same dispatch path without an HTTP job. Its journal
        # must also latch failures or an interrupted dispatch across restart.
        journal=self.directory/'events.jsonl'
        last_unknown_details={}
        self.teach_fault = None
        pending_teach_side = None
        rechecking_side = None
        if journal.exists():
            pending=False
            for line in journal.read_text().splitlines():
                try:event=json.loads(line)
                except ValueError:
                    self.teach_fault=None;rechecking_side=None;self.faulted=True;continue
                if event['kind'] in ('dispatch','teach_dispatch','fault_acknowledged','execution','outcome_unknown'):
                    rechecking_side = None
                if event['kind'] in ('dispatch', 'teach_dispatch'):
                    pending_teach_side = event.get('arm') if event['kind'] == 'teach_dispatch' and not pending else None
                    self.teach_fault = None
                    pending=True;last_unknown_details={}
                elif event['kind']=='teach_held':pending=False
                elif event['kind']=='teach_recheck_dispatch':
                    rechecking_side = (event.get('arm') if self.teach_fault and self.teach_fault['kind']=='feedback'
                                       and self.teach_fault['side']==event.get('arm') else None)
                    self.teach_fault = None
                    self.faulted = True
                elif event['kind']=='teach_recheck_held' and rechecking_side and event.get('arm')==rechecking_side:
                    self.teach_fault = dict(kind='verified',side=rechecking_side,receipt=event['receipt'],
                                            error='hold_verified_check_parked_feedback')
                    rechecking_side = None
                elif event['kind']=='execution':
                    pending=False;self.faulted|=stops_runtime(event['result'])
                    self.last_execution=feedback(event['result'])
                elif event['kind']=='outcome_unknown':
                    if (pending and not self.faulted and event.get('operation') == 'teach_hold'
                            and event.get('arm') == pending_teach_side and pending_teach_side in ('left', 'right')):
                        self.teach_fault = teaching_fault(pending_teach_side, str(event.get('error', '')))
                    else:
                        self.teach_fault = None
                    pending=False;self.faulted|=not motion_not_started(event.get('diagnostics',{}))
                    last_unknown_details=event.get('diagnostics',{})
                    self.last_execution=dict(accuracy=None,warning_only=False,can_continue=False,error='outcome_unknown')
                elif event['kind']=='fault_acknowledged':
                    # A human read the station idle and error free and cleared the
                    # latch. Replay must not resurrect the fault they acknowledged;
                    # anything that fails afterwards latches again on its own.
                    self.teach_fault = None
                    pending=False;self.faulted=False
            self.faulted|=pending
        # Older receipts retain their original verdict. Add a read-only view of
        # their measured deviations from the archived plan, without rewriting evidence.
        for job in reversed(list(self.jobs.values())):
            if job.get('dispatch')!='sent':continue
            if not job.get('result'):
                self.last_execution=dict(accuracy=None,warning_only=False,can_continue=False,error=job.get('error','outcome_unknown'))
                break
            result=job['result'];execution=execution_value(result)
            self.last_execution=feedback(execution)
            if self.last_execution['accuracy'] is None:
                plan_id=result.get('plan',{}).get('diagnostics',{}).get('plan_id')
                if isinstance(plan_id,str) and len(plan_id)==32 and all(c in '0123456789abcdef' for c in plan_id):
                    try:
                        plan=json.loads((self.directory/('plan-'+plan_id)/'plan.json').read_text())
                        receipt=execution['diagnostics']['native_receipt']
                        self.last_execution['accuracy']=endpoint_accuracy(plan['trajectory']['joint_positions'][-1],plan['payload']['target_pose'],receipt)
                    except (OSError,ValueError,KeyError,TypeError,IndexError):pass
            self.last_execution['job_id']=job['id']
            break
        if last_unknown_details and self.last_execution:
            # Enrich the displayed legacy failure from its journal; keep the
            # original job and receipt unchanged.
            from .localization import chinese_error
            message=last_unknown_details.get('cause_message','outcome_unknown')
            self.last_execution.update(error=message,error_zh=chinese_error(message),diagnostics=last_unknown_details)
            if motion_not_started(last_unknown_details):
                self.last_execution.update(accuracy=None,can_continue=not self.faulted,result_ok=False)
        self._persist()
        configuration = self.adapter.configuration() if hasattr(self.adapter,'configuration') else self.adapter.describe()
        self.event('session_open', capabilities=self.describe(), configuration_binding=self.adapter.binding(), configuration=configuration)

    def _persist(self):
        with self._mutex:
            path = self.directory / 'jobs.json'; temporary = path.with_suffix('.tmp')
            with temporary.open('w') as stream:
                json.dump(self.jobs, stream, allow_nan=False); stream.flush(); os.fsync(stream.fileno())
            temporary.replace(path)

    @contextmanager
    def audit_motion(self, audit):
        previous = getattr(self._local, 'motion_audit', None)
        self._local.motion_audit = audit
        try:
            yield audit
        finally:
            self._local.motion_audit = previous

    def event(self, kind, **data):
        audit = getattr(self._local, 'motion_audit', None)
        if audit is not None: audit.event(kind, data)
        with self._mutex:
            with (self.directory / 'events.jsonl').open('a') as stream:
                stream.write(json.dumps(public(dict(kind=kind, timestamp_s=time.time(), **data)), allow_nan=False)+'\n')
                stream.flush(); os.fsync(stream.fileno())

    @contextmanager
    def lease(self):
        nested = getattr(self._local, 'owner', False)
        if not nested and not self._gate.acquire(False): raise ApiError(ErrorCode.SAFETY_INTERLOCK, 'station_busy')
        if not nested: self._local.owner = True
        try:
            if self.closed: raise ApiError(ErrorCode.SAFETY_INTERLOCK, 'station_closed')
            yield
        finally:
            if not nested:
                self._local.owner = False
                self._gate.release()

    @contextmanager
    def program_lease(self, should_stop=None):
        """Reserve one program while allowing its own typed calls to reuse the lease."""
        if getattr(self._local, 'owner', False):
            raise ApiError(ErrorCode.SAFETY_INTERLOCK, 'nested_program_not_allowed')
        with self.lease():
            self._local.owner = True
            self._local.stop_requested = should_stop
            self._execution_abort.clear()
            try:
                yield
            finally:
                self._local.stop_requested = None
                self._local.owner = False

    def request_execution_abort(self):
        """Explicit experiment End, distinct from dispatch-boundary Pause."""
        self._execution_abort.set()
        self.event('operator_execution_abort_requested')

    def describe(self):
        operations = ['get_robot_state','get_observation','solve_ik','plan_motion','plan_synchronized_motion',
                      'execute_trajectory','set_gripper','set_grippers','open_gripper','close_gripper',
                      'step','go_home','move_to_pose','move_to_joints','move_synchronized']
        unsupported = getattr(self.adapter, 'unsupported_operations', ())
        capabilities = self.adapter.describe()
        arx = capabilities.get('embodiment') in ('arx_x5_2023', 'arx_x5_2023_dual')
        return dict(capabilities, motion_enabled=self.allow_motion, paused=self.paused,
                    execution_abort_supported=bool(getattr(self.adapter, 'supports_execution_abort', False)),
                    physical_session=self.physical_session.describe() if self.physical_session else None,
                    faulted=self.faulted, busy=self._gate.locked(),
                    endpoint_accuracy_policy=(
                        'sdk_reference_completion; raw_deviation_reported; caller_goal_tolerance; physical_arrival_not_inferred' if arx else
                        'report_deviation_without_latching_when_motion_completed_idle_error_free'),
                    supported=[name for name in operations if name not in unsupported],
                    home=self.home, control_period_s=self.control_period_s,
                    execution_requirements='motion enabled, not paused, no unresolved device failure',
                    trajectory_gripper_timing=('unavailable' if 'set_gripper' in unsupported else
                                               ARX_GRIPPER_TIMING if arx else
                                               'blocking hand commands at change waypoints; adds dwell to arm timing'))

    def get_robot_state(self):
        with self.lease():
            try:
                state = self.adapter.get_robot_state()
            except Exception as error:
                self.connection.update(status='error', error=str(error),
                    error_details=public(error) if isinstance(error,ApiError) else None)
                raise
            self.connection.update(status='state_read', error=None,
                robot_state=public(state), checked_at_s=time.time())
            self._capture_home(state)
            audit = getattr(self._local, 'motion_audit', None)
            if audit is not None: audit.sample(state)
            return state

    def get_observation(self, camera_names=None):
        with self.lease():
            self.connection.update(status='connecting', error=None)
            try:
                obs = self.adapter.get_observation(camera_names)
            except Exception as error:
                self.connection.update(status='error', error=str(error))
                raise
            audit = getattr(self._local, 'motion_audit', None)
            if audit is not None: audit.sample(obs.robot_state)
            self._capture_home(obs.robot_state)
            key = uuid4().hex
            folder = self.directory / ('observation-'+key); folder.mkdir()
            for name, camera in obs.cameras.items():
                # Camera names are metadata, never paths.
                index = list(obs.cameras).index(name)
                np.savez_compressed(folder / f'camera-{index}.npz', rgb=camera.rgb, depth_m=camera.depth_m, intrinsics=camera.intrinsics)
            metadata = {f.name:public(getattr(obs,f.name)) for f in fields(obs) if f.name!='cameras'}
            metadata['cameras']={name:{f.name:public(getattr(camera,f.name)) for f in fields(camera) if f.name not in ('rgb','depth_m')}
                                 for name,camera in obs.cameras.items()}
            metadata['device_binding'] = self.adapter.device_binding() if hasattr(self.adapter,'device_binding') else self.adapter.binding()
            if getattr(self.adapter,'raw',None) is not None:
                metadata['raw_robot_state'] = public(self.adapter.raw)
            if hasattr(self.adapter,'observation_diagnostics'):
                metadata['measurement_diagnostics']=public(self.adapter.observation_diagnostics)
            (folder/'metadata.json').write_text(json.dumps(metadata))
            self.observations[key] = (time.monotonic(), obs, self.adapter.binding())
            self.event('observation', observation_id=key, artifact=folder.name)
            self.connection.update(status='observation_read', error=None,
                robot_state=public(obs.robot_state), checked_at_s=time.time(), observation_id=key,
                cameras={name:dict(rgb_shape=list(cam.rgb.shape), depth_shape=list(cam.depth_m.shape),
                    valid_depth_fraction=float(np.mean(cam.depth_m>0))) for name,cam in obs.cameras.items()})
            return obs

    def solve_ik(self, target_pose, seed_joints=None, *, arm='primary', backend=None):
        if hasattr(self.adapter, 'plan_pair') and backend in (None, 'station'):
            with self.lease():
                return self.adapter.solve_ik(target_pose, seed_joints, arm=arm)
        if 'solve_ik' in getattr(self.adapter, 'unsupported_operations', ()) or arm != 'primary' or backend not in (None,'station'):
            return IKResult(False, error=ApiError(ErrorCode.UNSUPPORTED, 'backend_or_arm_unavailable'))
        with self.lease(): return self.adapter.solve_ik(target_pose, seed_joints)

    def plan_motion(self, goal, start_joints=None, *, arm='primary', strategy=None, speed_factor=None,
                    interpolation_dt_s=None, maximum_trajectory_dt_s=None):
        arms = self.adapter.describe().get('arms', ('primary',))
        if arm not in arms:
            return PlanResult(False, error=ApiError(ErrorCode.NOT_FOUND, 'arm_unavailable'))
        if strategy is not None:
            return self._public_tools()['plan_motion'](goal,start_joints,arm=arm,strategy=strategy)
        with self.lease():
            try:
                before = self.adapter.binding()
                options={k:v for k,v in dict(speed_factor=speed_factor,start_joints=start_joints,
                    interpolation_dt_s=interpolation_dt_s,maximum_trajectory_dt_s=maximum_trajectory_dt_s).items() if v is not None}
                if arm != 'primary':
                    options['arm'] = arm
                trajectory, payload = self.adapter.plan_motion(goal,**options)
                key = self._store_plan(trajectory, payload, before)
                preview=[]
                if hasattr(self.adapter, 'preview_xyz'):
                    preview=self.adapter.preview_xyz(trajectory)
                    # The visual preview is decimated across the complete curve;
                    # the public Trajectory and native execution retain every sample.
                    if len(preview)>60:
                        preview=[preview[i] for i in np.linspace(0,len(preview)-1,60,dtype=int)]
                return PlanResult(True, trajectory=trajectory, diagnostics={'plan_id':key, 'backend':trajectory.planner,'preview_xyz':preview,
                    **({k:payload[k] for k in ('speed_factor','max_speed_factor','duration_s','collision_checked','collision_scope','collision_missing','boundary_adjustments','automatic_scene','fk_position_residual_m','fk_rotation_residual_rad','path_position_residual_m','path_rotation_residual_rad','end_effector_references') if k in payload}
                       if isinstance(payload,dict) else {})})
            except (ApiError, ValueError) as error:
                if not isinstance(error,ApiError):
                    error=ApiError(ErrorCode.PLANNING_FAILED,str(error),details=getattr(error,'details',{}))
                self.event('planning_rejected',error=error,goal=goal,dispatch='not_sent')
                return PlanResult(False,error=error)

    def _store_plan(self, trajectory, payload, binding):
        key = uuid4().hex
        self.plans[key] = dict(trajectory=trajectory, payload=payload, binding=binding,
                               digest=fingerprint(trajectory), created=time.monotonic())
        self.event('plan', plan_id=key, trajectory=trajectory, binding=binding)
        folder = self.directory / ('plan-' + key)
        folder.mkdir()
        record = dict(trajectory=trajectory, payload=payload, binding=binding)
        (folder / 'plan.json').write_text(json.dumps(public(record), allow_nan=False))
        return key

    def plan_pair(self, state, targets, **options):
        with self.lease():
            try:
                binding = self.adapter.binding()
                trajectory, payload = self.adapter.plan_pair(state, targets, **options)
                key = self._store_plan(trajectory, payload, binding)
                return SynchronizedPlanResult(True, trajectory=trajectory, diagnostics={
                    'plan_id': key, 'duration_s': trajectory.dt_s, 'speed_factor': payload['speed_factor'],
                    'collision_checked': payload['collision_checked'], 'collision_missing': payload['collision_missing'],
                    'boundary_adjustments': payload['boundary_adjustments'],
                    **({k: payload[k] for k in ('end_effector_references',) if k in payload})})
            except (ValueError, ApiError) as exc:
                error = exc if isinstance(exc, ApiError) else ApiError(ErrorCode.PLANNING_FAILED, str(exc))
                self.event('planning_rejected', error=error, targets=targets, dispatch='not_sent')
                return SynchronizedPlanResult(False, error=error)

    def bind_public_plan(self, plan_id, trajectory):
        """Bind the upstream API's final typed trajectory to its native payload."""
        with self.lease():
            plan = self.plans[plan_id]
            old = plan['trajectory']
            if not np.array_equal(old.joint_positions, trajectory.joint_positions) or old.dt_s != trajectory.dt_s:
                return  # Cache miss: execution will prepare the caller's actual curve.
            plan.update(trajectory=trajectory, digest=fingerprint(trajectory))
            path=self.directory/('plan-'+plan_id)/'plan.json'
            path.write_text(json.dumps(public(dict(trajectory=trajectory,payload=plan['payload'],binding=plan['binding'])),allow_nan=False))
            self.event('public_plan_bound', plan_id=plan_id, trajectory=trajectory)

    def _dispatch(self, operation, function):
        if not self.allow_motion or self.paused or self.faulted:
            return failure(ErrorCode.SAFETY_INTERLOCK, 'motion_disabled_paused_or_faulted')
        if self.physical_session:
            if not self.physical_session.permits(self.physical_session.task_name):return failure(ErrorCode.SAFETY_INTERLOCK,'physical_session_expired')
            self.physical_session.dispatches+=1
        job = getattr(self._local, 'job', None)
        self.event('dispatch', operation=operation, job_id=job, dispatch='sent')
        if job:
            self.jobs[job]['dispatch'] = 'sent'; self._persist()
        try:
            result = function()
            if not isinstance(result, ExecutionResult): raise OutcomeUnknown('missing_execution_receipt')
            if stops_runtime(result): self.faulted = True
            self.last_execution=feedback(result)
            self.last_execution['job_id']=job
            self.event('execution', operation=operation, result=result, physical_evidence=not self.adapter.synthetic)
            return result
        except Exception as error:
            diagnostics=error.diagnostics if isinstance(error, OutcomeUnknown) else {
                'cause_type':type(error).__name__,'cause_message':str(error)}
            if motion_not_started(diagnostics):
                result=ExecutionResult(False,0,error=ApiError(ErrorCode.STALE_PLAN,
                    diagnostics['cause_message'],details=diagnostics),
                    diagnostics=dict(diagnostics,outcome='not_started',steps_count_known=True))
                self.last_execution=feedback(result)
                self.last_execution['job_id']=job
                self.event('execution',operation=operation,result=result,physical_evidence=not self.adapter.synthetic)
                return result
            self.faulted = True
            if diagnostics.get('cause_type')=='ControlException':
                # libfranka explicitly reports an aborted control operation.
                # This is a known failure, even if no successful terminal
                # receipt exists. Partial progress remains unknown.
                result=ExecutionResult(False,0,error=ApiError(ErrorCode.CONTROLLER_FAILED,
                    diagnostics['cause_message'],recoverable=False,details=diagnostics),
                    diagnostics=dict(diagnostics,outcome='aborted',steps_count_known=False))
                self.last_execution=feedback(result)
                self.last_execution['job_id']=job
                self.event('execution',operation=operation,result=result,physical_evidence=not self.adapter.synthetic)
                return result
            self.last_execution=dict(accuracy=None,warning_only=False,can_continue=False,error='outcome_unknown')
            self.event('outcome_unknown', operation=operation, error_type=type(error).__name__,
                       diagnostics=diagnostics)
            raise OutcomeUnknown('dispatch_without_verified_terminal_receipt',diagnostics=diagnostics) from error

    def execute_trajectory(self, trajectory, stop_on_termination=True, *, targets=None, tolerance=None, max_steps=None):
        if stop_on_termination is not True: return failure(ErrorCode.UNSUPPORTED, 'stop_on_termination_required')
        paired = isinstance(trajectory, SynchronizedTrajectory) and hasattr(self.adapter, "plan_pair")
        if isinstance(trajectory, SynchronizedTrajectory) and not paired:
            if set(trajectory.joint_positions) != {'primary'}:
                return failure(ErrorCode.NOT_FOUND, 'arm_unavailable')
            trajectory=Trajectory(trajectory.joint_positions['primary'],trajectory.dt_s,
                trajectory.joint_names['primary'],trajectory.planner,trajectory.collision_aware,
                trajectory.expected_start,arm='primary',embodiment=trajectory.embodiment,
                gripper_positions=None if trajectory.gripper_positions is None else trajectory.gripper_positions['primary'])
        if not paired and not isinstance(trajectory, Trajectory):return failure(ErrorCode.INVALID_REQUEST,'trajectory_required')
        with self.lease():
            state=self.get_robot_state();expected=trajectory.expected_start
            if (not paired and trajectory.arm not in state.arms) or state.embodiment!=trajectory.embodiment or state.joint_names!=expected.joint_names:
                return failure(ErrorCode.INVALID_REQUEST,'robot_identity_changed')
            self.event('trajectory_start_measurement',joint_delta={a:state.joint_positions[a]-expected.joint_positions[a] for a in expected.arms},
                joint_velocities=state.joint_velocities,gripper_positions=state.gripper_positions)
            if hasattr(self.adapter, 'plan_pair'):
                count = trajectory.waypoint_count if paired else len(trajectory.joint_positions)
                if max_steps is not None and count > max_steps:
                    return ExecutionResult(False, 0, error=ApiError(ErrorCode.INVALID_REQUEST,
                        'trajectory_exceeds_max_steps', details={'waypoint_count': count, 'max_steps': max_steps}))
                return self._execute_arm(trajectory, targets=targets, tolerance=tolerance)
            if trajectory.gripper_positions is not None:
                if 'set_gripper' in getattr(self.adapter, 'unsupported_operations', ()):
                    return failure(ErrorCode.UNSUPPORTED, 'gripper_trajectory_unavailable')
                from .motion import execute_with_gripper
                return execute_with_gripper(self,trajectory)
            return self._execute_arm(trajectory)

    def _execute_arm(self, trajectory, *, targets=None, tolerance=None):
        # Registration is a native-curve cache, not permission to execute. A
        # constructed/edited/repeated trajectory is a valid public API input.
        digest=fingerprint(trajectory)
        plan=next((p for p in self.plans.values() if p['digest']==digest and p['binding']==self.adapter.binding()),None)
        try:
            payload=plan['payload'] if plan else self.adapter.prepare_trajectory(trajectory)
        except ApiError as error:
            return ExecutionResult(False, 0, error=error)
        except ValueError as error:
            return failure(ErrorCode.INVALID_REQUEST, str(error))
        self.event('trajectory_reference',trajectory=trajectory,native_cache_hit=plan is not None)
        if hasattr(self.adapter,'check_execution'):
            try:
                error=self.adapter.check_execution(payload)
            except ApiError as error:
                return ExecutionResult(False, 0, error=error)
            if error:return failure(ErrorCode.CONTROLLER_FAILED,error)
        if hasattr(self.adapter, 'plan_pair'):
            def execute_pair():
                return self.adapter.execute(payload, trajectory, permitted=self._segment_permitted,
                                            targets=targets, tolerance=tolerance,
                                            **({'abort_requested': self._execution_abort.is_set}
                                               if getattr(self.adapter, 'supports_execution_abort', False) else {}))
            return self._dispatch('execute_trajectory', execute_pair)
        return self._dispatch('execute_trajectory',lambda:self.adapter.execute(payload,trajectory))

    def _segment_permitted(self):
        stopping = getattr(self._local, 'stop_requested', None)
        return (not self.paused and not self.closed and not self.faulted and self.allow_motion
                and not (stopping and stopping()))

    def _pair_grippers(self, positions):
        with self.lease():
            try:
                self.adapter.check_hands(positions)
            except (ValueError, ApiError) as error:
                return failure(ErrorCode.INVALID_REQUEST, str(error))
            return self._dispatch('set_grippers', lambda: self.adapter.move_hands(positions, self._segment_permitted,
                **({'abort_requested': self._execution_abort.is_set}
                   if getattr(self.adapter, 'supports_execution_abort', False) else {})))

    def set_gripper(self, position, *, arm='primary'):
        if hasattr(self.adapter, 'plan_pair'):
            return self._pair_grippers({arm: position})
        if 'set_gripper' in getattr(self.adapter,'unsupported_operations', ()) or arm != 'primary': return failure(ErrorCode.UNSUPPORTED, 'gripper_or_arm_unavailable')
        if isinstance(position, bool) or not np.isfinite(position) or not 0<=position<=1:
            raise ValueError('position must be finite in [0,1]')
        with self.lease():
            try:error = self.adapter.check_gripper(position)
            except ApiError as error:return ExecutionResult(False,0,error=error)
            if error: return failure(ErrorCode.SAFETY_INTERLOCK, error)
            return self._dispatch('set_gripper', lambda: self.adapter.set_gripper(position))

    def open_gripper(self, *, arm='primary'):
        return self.set_gripper(1., arm=arm)

    def close_gripper(self, *, arm='primary'):
        if hasattr(self.adapter, 'plan_pair'):
            return self.set_gripper(0., arm=arm)
        if 'close_gripper' in getattr(self.adapter,'unsupported_operations', ()) or arm != 'primary': return failure(ErrorCode.UNSUPPORTED, 'gripper_or_arm_unavailable')
        if not hasattr(self.adapter, 'close_gripper'): return self.unsupported()
        with self.lease():
            try:error = self.adapter.check_close_gripper()
            except ApiError as error:return ExecutionResult(False,0,error=error)
            if error: return failure(ErrorCode.SAFETY_INTERLOCK, error)
            return self._dispatch('close_gripper', self.adapter.close_gripper)

    def install_scene(self, scene):
        """Optional offline/mock fixture API; physical dispatch has no scene model."""
        with self.lease():
            if not hasattr(self.adapter,'install_scene'):
                raise ApiError(ErrorCode.UNSUPPORTED,'scene_installation_not_supported_by_adapter')
            self.adapter.install_scene(scene)
            self.event('scene_installed', scene=scene, binding=self.adapter.binding())

    def unsupported(self, *args, **kwargs): return failure(ErrorCode.UNSUPPORTED, 'operation_not_supported_by_adapter')

    @property
    def control_period_s(self):
        return float(getattr(self.adapter,'control_period_s',1./30.))

    def _capture_home(self,state):
        configured=getattr(getattr(self.adapter,'sdk',None),'cfg',{}).get('home_joint_positions')
        home_binding = getattr(self.adapter, 'home_binding', None)
        if self.home is not None and self.home.get('device_binding') == home_binding and (configured is None or (self.home['source']=='station_configuration'
                and self.home['joint_positions']==public(configured))):return
        self.home=dict(joint_positions=public(configured if configured is not None else state.joint_positions),
            embodiment=state.embodiment,joint_names=public(state.joint_names),
            source='station_configuration' if configured is not None else 'first_measured_state_in_session',timestamp_s=time.time())
        if home_binding is not None:
            self.home['device_binding'] = home_binding
        self.home_path.write_text(json.dumps(self.home,allow_nan=False))
        self.event('home_reference',reference=self.home)

    def _public_tools(self):
        # Convenience entry points reuse CapApi composition and the same owner.
        from .cap.api import PublicAPI
        from .contracts import TaskContext
        if not hasattr(self,'_motion_api'):
            context=TaskContext('deployment',0,'runtime','Runtime motion API','manual',{})
            self._motion_api=PublicAPI(self,context,providers={})
        return self._motion_api.tools()

    def move_to_joints(self,*args,**kwargs):return self._public_tools()['move_to_joints'](*args,**kwargs)
    def move_to_pose(self,*args,**kwargs):return self._public_tools()['move_to_pose'](*args,**kwargs)
    def move_synchronized(self,*args,**kwargs):return self._public_tools()['move_synchronized'](*args,**kwargs)
    def plan_synchronized_motion(self,*args,**kwargs):return self._public_tools()['plan_synchronized_motion'](*args,**kwargs)

    def return_recorded_joints(self, recorded, *, arm='both', speed_factor=None):
        with self.lease():
            try:
                state = self.get_robot_state()
                if arm != 'both' and arm not in state.arms:
                    return failure(ErrorCode.NOT_FOUND, 'arm_unavailable')
                options = {} if speed_factor is None else {'speed_factor': speed_factor}
                trajectory, payload = self.adapter.plan_recorded_return(state, recorded, arm, **options)
                self._store_plan(trajectory, payload, self.adapter.binding())
            except (ValueError, ApiError) as exc:
                return ExecutionResult(False, 0, error=exc if isinstance(exc, ApiError) else
                                       ApiError(ErrorCode.PLANNING_FAILED, str(exc)))
            return self.execute_trajectory(trajectory)

    def go_home(self,*,arm=None):
        with self.lease():
            pair = hasattr(self.adapter, 'plan_pair')
            if pair and self.home is not None and self.home.get('device_binding') != self.adapter.home_binding:
                return failure(ErrorCode.INVALID_REQUEST, 'home_device_changed')
            state=self.get_robot_state()
            if self.home['embodiment']!=state.embodiment or self.home['joint_names']!=public(state.joint_names):
                return failure(ErrorCode.INVALID_REQUEST,'home_robot_identity_changed')
            if pair:
                side = self.adapter.describe()['arm_aliases'].get(arm, arm) or 'both'
                self.event('home_requested', reference=self.home, arm=side)
                return self.return_recorded_joints(self.home['joint_positions'], arm=side)
            arm = arm or 'primary'
            if arm not in state.arms:return failure(ErrorCode.NOT_FOUND,'arm_unavailable')
            joints=self.home['joint_positions']
            goal=joints[arm] if isinstance(joints,dict) else joints
            self.event('home_requested',reference=self.home)
            return self.move_to_joints(goal,arm=arm)

    def set_grippers(self,positions):
        if hasattr(self.adapter, 'plan_pair'):
            if not isinstance(positions, Mapping) or set(positions) != set(self.adapter.arms):
                return failure(ErrorCode.INVALID_REQUEST, 'positions_must_name_left_and_right')
            return self._pair_grippers(positions)
        if not isinstance(positions,Mapping) or set(positions)!={'primary'}:
            return failure(ErrorCode.INVALID_REQUEST,'positions_must_name_primary')
        return self.set_gripper(positions['primary'])

    def step(self,action):
        if 'step' in getattr(self.adapter, 'unsupported_operations', ()):
            from .contracts import StepResult
            return StepResult(False, error=ApiError(ErrorCode.UNSUPPORTED, 'step_unavailable_on_station'))
        from .motion import step
        return step(self,action)

    def pause(self, paused=True):
        self.paused = bool(paused); self.event('operator_pause', paused=self.paused)

    def unlock(self):
        """Operator-pressed controller unlock: the library's own error recovery.

        Pressed in the console by the human standing at the station, in place of
        the same recovery in Franka Desk; nothing in the dispatch path reaches it.
        It clears the controller's error state and moves nothing, so it works
        while this runtime still holds a latched fault. Dropping that latch stays
        with the console's human gates, which read the device back as idle first.
        """
        with self.lease():
            if not hasattr(self.adapter, 'recover'):
                raise ApiError(ErrorCode.UNSUPPORTED, 'recovery_not_supported_by_adapter')
            receipt = self.adapter.recover()
            self.event('human_unlock', receipt=receipt, synthetic=self.adapter.synthetic)
            return receipt

    def acknowledge_fault(self, reason, *, validate=None):
        """Clear the latched execution fault on an explicit human acknowledgement.

        Called only from a host entry point a human just operated, never from
        generated code. No controller recovery is sent: the device itself must
        report idle and error free, which `get_robot_state` enforces, so a station
        still in a reflex keeps refusing. The journal records the acknowledgement,
        so the cleared latch does not come back on the next restart.
        """
        if not isinstance(reason, str) or not reason.strip(): raise ValueError('acknowledgement_reason_required')
        if self.closed: raise ApiError(ErrorCode.SAFETY_INTERLOCK, 'station_closed')
        state = self.get_robot_state()
        if validate is not None:
            validate(state)
        if self.faulted:
            self.event('fault_acknowledged', reason=reason, state=state)
            self.faulted = False
            self.teach_fault = None
        return state

    def submit(self, request_id, operation, arguments):
        allowed = {'get_robot_state','get_observation','plan_motion','solve_ik','execute_trajectory','set_gripper','open_gripper','close_gripper'}
        if operation not in allowed or not isinstance(arguments, dict): raise ValueError('invalid_operation')
        if not isinstance(request_id, str) or not request_id: raise ValueError('invalid_request_id')
        fields = {'get_robot_state':set(), 'get_observation':{'camera_names'},
                  'plan_motion':{'goal','start_joints','arm','strategy'},
                  'solve_ik':{'target_pose','seed_joints','arm','backend'},
                  'execute_trajectory':{'plan_id'},'set_gripper':{'position','arm'},
                  'open_gripper':{'arm'},'close_gripper':{'arm'}}
        required = {'plan_motion':'goal','solve_ik':'target_pose','execute_trajectory':'plan_id','set_gripper':'position'}
        if set(arguments)-fields[operation] or (operation in required and required[operation] not in arguments):raise ValueError('invalid_arguments')
        return self._submit(request_id, operation, arguments)

    def _submit(self, request_id, operation, arguments, *, invoke=None):
        """Internal job runner shared with the fixed, typed operator API."""
        if not isinstance(request_id, str) or not request_id:raise ValueError('invalid_request_id')
        signature = fingerprint([operation, arguments])
        with self._mutex:
            if self.closed:raise ValueError('station_closed')
            if request_id in self.jobs:
                if self.jobs[request_id]['signature'] != signature: raise ValueError('request_id_conflict')
                return dict(self.jobs[request_id])
            if not self._gate.acquire(False): raise ApiError(ErrorCode.SAFETY_INTERLOCK, 'station_busy')
            self.jobs[request_id] = dict(id=request_id, operation=operation, signature=signature, status='accepted', dispatch='not_sent')
            self._persist()
        def worker():
            self._local.owner=True; self._local.job=request_id
            try:
                job=self.jobs[request_id];job['status']='running';self._persist()
                args=dict(arguments)
                if invoke is None and operation=='execute_trajectory': args={'trajectory':self.plans[args['plan_id']]['trajectory']}
                result=invoke(args) if invoke is not None else getattr(self, operation)(**args)
                if operation=='get_observation' and invoke is None:
                    # Pixel arrays are already archived. Avoid expanding them into Python lists.
                    job['observation_id']=next(reversed(self.observations))
                    summary = {f.name: public(getattr(result, f.name)) for f in fields(result) if f.name != 'cameras'}
                    summary['cameras'] = {name: {f.name: public(getattr(cam, f.name)) for f in fields(cam)
                                                if f.name not in ('rgb', 'depth_m')} for name, cam in result.cameras.items()}
                else:
                    summary = public(result)
                job.update(status='completed', result=summary)
            except OutcomeUnknown as error: job.update(status='outcome_unknown', error=str(error))
            except Exception as error:
                from .localization import chinese_error
                job.update(status='failed', error=str(error),message_zh=chinese_error(error))
                if isinstance(error,ApiError):job['error_details']=public(error)
            finally:
                self._persist();self._local.owner=False;self._gate.release()
        threading.Thread(target=worker, daemon=True).start()
        return dict(self.jobs[request_id])

    def snapshot(self):
        with self._mutex: return public(dict(capabilities=self.describe(), connection=self.connection, last_execution=self.last_execution, jobs=self.jobs))

    def localize_pixel(self, observation_id, camera_name, pixel):
        stamp, obs, binding = self.observations[observation_id]
        cam=obs.cameras[camera_name];x,y=map(int,pixel)
        if not (0<=y<cam.depth_m.shape[0] and 0<=x<cam.depth_m.shape[1]): raise ValueError('pixel_outside_image')
        z=cam.depth_m[y,x]
        if z<=0: raise ValueError('invalid_depth')
        point=np.linalg.solve(cam.intrinsics, [x,y,1])*z
        return {'point':(cam.camera_pose.as_matrix()@np.r_[point,1])[:3].tolist(), 'frame':cam.camera_pose.frame}

    def measure_region(self, observation_id, camera_name, box):
        stamp,obs,binding=self.observations[observation_id]
        cam=obs.cameras[camera_name];x,y,w,h=map(int,box)
        if min(w,h)<=0 or min(x,y)<0 or x+w>cam.depth_m.shape[1] or y+h>cam.depth_m.shape[0]:raise ValueError('invalid_region')
        yy,xx=np.mgrid[y:y+h,x:x+w];z=cam.depth_m[y:y+h,x:x+w];valid=z>0
        if not valid.any():raise ValueError('no_valid_depth')
        points=(np.linalg.inv(cam.intrinsics)@np.stack([xx[valid],yy[valid],np.ones(valid.sum())]))*z[valid]
        transform=cam.camera_pose.as_matrix();points=(transform[:3,:3]@points+transform[:3,3,None]).T
        return dict(frame=cam.camera_pose.frame,point_count=len(points),center=np.median(points,axis=0).tolist(),lower=points.min(0).tolist(),upper=points.max(0).tolist(),scope='visible_surface_only')

    def close(self):
        self.paused=True
        with self._gate:
            self.closed=True; self.adapter.close()
