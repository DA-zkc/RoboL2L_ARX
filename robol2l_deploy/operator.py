"""Human calls to the same public CaP API, with typed result references.

No eval, source execution, user-selected imports, or separate device owner.
Manual references live only here and are never injected into autonomous CaP.
"""
from collections.abc import Mapping
from dataclasses import fields, is_dataclass
import inspect
import threading
import time
from uuid import uuid4
import numpy as np
from .contracts import Pose, TaskContext
from .cap.api import PublicAPI, MOTION, CONSTRUCTORS
from .cap.program import compact

MANUAL_TASK = TaskContext('deployment', 0, 'manual-workbench',
    'Human-operated API debugging; not autonomous CaP evidence.', 'manual', {})


class OperatorConsole:
    def __init__(self, runtime, cap, *, providers=None):
        self.runtime, self.cap = runtime, cap
        self.api = PublicAPI(runtime, MANUAL_TASK, providers=providers)
        self.tools = self.api.tools()
        self.references = {}
        self.lock = threading.RLock()

    def idle(self):
        if self.runtime.closed:raise ValueError('station_closed')
        if self.cap.thread and self.cap.thread.is_alive():
            raise ValueError('cap_session_busy; pause and wait for completion')

    def describe(self):
        info = self.api.describe()
        supported = set(info['capabilities']['supported'])
        with self.lock:
            return dict(tools=[dict(name=name, signature=str(inspect.signature(fn)),
                supported=name in supported, motion=name in MOTION)
                for name, fn in self.tools.items()],
                references=[dict(id=key, type=type(row['value']).__name__,
                    operation=row['operation'], observation_id=row['observation_id'])
                    for key, row in self.references.items()],
                providers=info['perception_providers'], units=info['units'],
                manual_context=True, cap_running=bool(self.cap.thread and self.cap.thread.is_alive()))

    def decode(self, value):
        if isinstance(value, list):
            return [self.decode(v) for v in value]
        if not isinstance(value, dict):
            return value
        if '$ref' in value:
            if set(value)-{'$ref', 'path'}:raise ValueError('invalid_reference')
            row = self.references[value['$ref']]
            result = row['value']
            for key in value.get('path', []):
                if is_dataclass(result) and key in {f.name for f in fields(result)}:
                    result = getattr(result, key)
                elif isinstance(result, Mapping):result = result[key]
                elif isinstance(result, (tuple, list, np.ndarray)) and type(key) is int:result = result[key]
                else:raise ValueError('invalid_reference_path')
            return result
        if '$type' in value:
            constructor=CONSTRUCTORS[value['$type']]
            return constructor(**{key:self.decode(item) for key,item in value.items() if key!='$type'})
        if '$pose' in value:
            if set(value) != {'$pose'}:raise ValueError('invalid_pose')
            return Pose(**value['$pose'])
        if '$array' in value:
            if set(value) != {'$array'}:raise ValueError('invalid_array')
            return np.asarray(value['$array'], dtype=float)
        return {key:self.decode(item) for key, item in value.items()}

    def submit(self, request_id, operation, arguments):
        self.idle()
        if operation not in self.tools or not isinstance(arguments, dict):
            raise ValueError('unsupported_operator_operation')
        def invoke(args):
            started = time.time()
            with self.lock:
                if operation in MOTION and not self.runtime.adapter.synthetic:
                    policy = self.runtime.physical_session
                    if not policy or not policy.permits(MANUAL_TASK.task_name):
                        from .physical import authorize
                        authorize(self.runtime,MANUAL_TASK.task_name,'Human invoked '+operation+' in the API panel')
                elif operation in MOTION:self.runtime.allow_motion=True
                inputs = self.decode(args)
                inspect.signature(self.tools[operation]).bind(**inputs)
                self.runtime.event('operator_call', operation=operation, inputs=compact(inputs),
                    autonomy_evidence=False)
                try:result = self.tools[operation](**inputs)
                except Exception as error:
                    # An explicit read can reconnect a dead socket. Never retry
                    # a command or recover a controller after an execution error.
                    if operation not in ('get_robot_state','get_observation') or not any(
                        text in str(error) for text in ('Net Exception','NetworkException','Connection reset','Broken pipe')):
                        raise
                    driver=getattr(self.runtime.adapter,'driver',None)
                    if driver is None:raise
                    self.runtime.event('operator_read_reconnect',operation=operation,error_type=type(error).__name__,motion_replayed=False)
                    driver.close();self.runtime.plans.clear();self.api.environment.invalidate()
                    result=self.tools[operation](**inputs)
                key = uuid4().hex
                self.references[key] = dict(value=result, operation=operation,
                    observation_id=self.api.environment.observation_id)
                reply = dict(handle=key, operation=operation, value=compact(result),
                    observation_id=self.api.environment.observation_id,
                    synthetic=self.runtime.adapter.synthetic, manual=True)
                arrays = {}
                def collect(value, path='result'):
                    if isinstance(value, np.ndarray):arrays[path] = value
                    elif is_dataclass(value):
                        for field in fields(value):collect(getattr(value, field.name), path+'.'+field.name)
                    elif isinstance(value, Mapping):
                        for name, item in value.items():collect(item, path+'.'+str(name))
                    elif isinstance(value, (list, tuple)):
                        for index, item in enumerate(value):collect(item, path+'.'+str(index))
                collect(result)
                from .cap.storage import write_json
                folder = self.runtime.directory/'operator'/key;folder.mkdir(parents=True)
                if arrays:np.savez_compressed(folder/'arrays.npz', **arrays)
                write_json(folder/'result.json', reply)
                self.runtime.event('operator_result', **reply, elapsed_s=time.time()-started)
                return reply
        return self.runtime._submit(request_id, 'operator:'+operation, arguments, invoke=invoke)

    def authorize(self, reason, distance=.02):
        self.idle()
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError('describe_the_authorized_physical_experiment')
        with self.runtime.program_lease():
            if self.runtime.adapter.synthetic:
                self.runtime.allow_motion = True
                self.runtime.event('operator_mock_motion_enabled', reason=reason)
            else:
                from .physical import authorize
                authorize(self.runtime, MANUAL_TASK.task_name, reason,
                    probe_distance_m=distance, require_probe=False)
            self.api.environment.invalidate()
        return self.runtime.snapshot()

    def manual_action(self, request_id, action, arguments):
        """One explicit human click, using the same public API as CaP."""
        self.idle()
        if action == 'move':
            if set(arguments) == {'delta_m'}:
                delta=np.asarray(arguments['delta_m'],dtype=float)
                if delta.shape!=(3,) or not np.isfinite(delta).all():
                    raise ValueError('invalid_manual_offset')
                target=None
            elif set(arguments) == {'target_pose'}:
                target=Pose(**arguments['target_pose']);delta=None
            else:raise ValueError('invalid_manual_move')
        elif action == 'gripper' and set(arguments) == {'position'}:
            position=arguments['position']
            if type(position) not in (int,float) or not np.isfinite(position) or not 0<=position<=1:
                raise ValueError('invalid_gripper_position')
        else:raise ValueError('invalid_manual_action')

        def invoke(args):
            with self.lock:
                if self.runtime.paused or self.runtime.faulted:raise ValueError('station_faulted_or_paused')
                self.runtime.event('operator_manual_action',action=action,arguments=args,autonomy_evidence=False)
                if self.runtime.adapter.synthetic:
                    self.runtime.allow_motion=True
                else:
                    policy=self.runtime.physical_session
                    if not policy or not policy.permits(MANUAL_TASK.task_name):
                        from .physical import authorize
                        authorize(self.runtime,MANUAL_TASK.task_name,
                            'Human clicked '+action+' in the manual panel',require_probe=False)
                self.api.environment.invalidate()
                plan=None
                if action=='move':
                    if 'move_to_pose' not in self.tools:
                        raise ValueError('cartesian_motion_unavailable_for_this_station')
                    goal=target
                    if goal is None:
                        pose=self.tools['get_robot_state']().end_effector_poses['primary']
                        goal=Pose(pose.position+delta,pose.quaternion_wxyz,pose.frame)
                    plan=self.tools['plan_motion'](goal=goal)
                    if not plan.ok:
                        result=dict(ok=False,stage='planning',error=compact(plan.error))
                    else:
                        execution=self.tools['execute_trajectory'](plan.trajectory)
                        result=dict(ok=execution.ok,stage='execution',execution=compact(execution),error=compact(execution.error))
                else:
                    execution=self.tools['set_gripper'](position=position)
                    result=dict(ok=execution.ok,stage='execution',execution=compact(execution),error=compact(execution.error))
                if plan is not None:result['plan']=compact(plan)
                if result['stage']=='execution':
                    from .execution_feedback import feedback
                    result.update(compact(feedback(execution)))
                result.update(synthetic=self.runtime.adapter.synthetic,manual=True)
                self.runtime.event('operator_manual_result',action=action,result=result,autonomy_evidence=False)
                return result
        return self.runtime._submit(request_id,'manual:'+action,arguments,invoke=invoke)

    def revoke(self):
        # Can close the dispatch gate during a call; does not interrupt an arm.
        self.runtime.allow_motion = False
        if self.runtime.physical_session:self.runtime.physical_session.active = False
        if hasattr(self.runtime.adapter, 'driver'):self.runtime.adapter.driver.allow_motion = False
        self.runtime.event('operator_motion_disabled')
        return self.runtime.snapshot()
