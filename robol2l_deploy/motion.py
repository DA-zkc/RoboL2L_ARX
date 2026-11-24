"""Single-arm compositions over Runtime; no device connections or controllers."""
from dataclasses import replace
import numpy as np
from .contracts import Trajectory, RobotAction, StepResult, ExecutionResult
from .errors import ApiError, ErrorCode


def execute_with_gripper(runtime, trajectory):
    """Franka Hand is blocking: change width at a waypoint, then resume the arm.

    Preserve every arm sample and its dt. Hand changes insert explicit dwell;
    they are never silently dropped or advertised as simultaneous servo ticks.
    """
    values=np.asarray(trajectory.gripper_positions).reshape(-1)
    changes=np.r_[0,np.flatnonzero(np.diff(values)!=0)+1]
    cursor=0;receipts=[];steps=0
    def record(result):
        nonlocal steps
        receipts.append(result);steps+=result.steps_executed
        return result.ok and not result.terminated and not result.truncated
    def finish(result):
        return replace(result,steps_executed=steps,diagnostics=dict(result.diagnostics,
            gripper_timing='blocking_at_change_waypoints',arm_dt_s=trajectory.dt_s,
            gripper_dwell_added=True,segments=[dict(ok=r.ok,steps=r.steps_executed,
                error=r.error,diagnostics=r.diagnostics) for r in receipts]))
    for index in changes:
        if index>cursor:
            segment=replace(trajectory,joint_positions=trajectory.joint_positions[cursor:index+1],gripper_positions=None)
            result=runtime._execute_arm(segment)
            if not record(result):return finish(result)
        result=runtime.set_gripper(float(values[index]),arm=trajectory.arm)
        if not record(result):return finish(result)
        cursor=int(index)
    if cursor<len(values)-1 or len(values)==1:
        segment=replace(trajectory,joint_positions=trajectory.joint_positions[cursor:],gripper_positions=None)
        result=runtime._execute_arm(segment)
        record(result)
    return finish(result)


def step(runtime,action):
    """One configured control-period target; observation overhead is additional."""
    if hasattr(runtime.adapter, 'plan_pair'):
        return pair_step(runtime, action)
    if not isinstance(action,RobotAction) or set(action.arms)!={'primary'}:
        return StepResult(False,error=ApiError(ErrorCode.INVALID_REQUEST,'action_must_name_primary'))
    with runtime.lease():
        state=runtime.get_robot_state();command=action.arms['primary']
        if action.embodiment!=state.embodiment:
            return StepResult(False,error=ApiError(ErrorCode.INVALID_REQUEST,'action_embodiment_mismatch'))
        # Native position control begins at q_d, as ordinary franky plans do.
        start=state.joint_positions['primary']
        if getattr(getattr(runtime.adapter,'driver',None),'backend',None)=='franky':
            start=np.asarray(runtime.adapter.raw['q_d'])
        trajectory=Trajectory(np.stack([start,command.target]),runtime.control_period_s,
            state.joint_names['primary'],'station_step',False,state,embodiment=state.embodiment,
            gripper_positions=None if command.gripper_position is None else np.full(2,command.gripper_position))
        try:execution=runtime.execute_trajectory(trajectory)
        except Exception as error:
            from .runtime import OutcomeUnknown
            if not isinstance(error,OutcomeUnknown):raise
            return StepResult(False,error=ApiError(ErrorCode.OUTCOME_UNKNOWN,str(error),recoverable=False,details=error.diagnostics))
        diagnostics=dict(execution.diagnostics,control_period_s=runtime.control_period_s,
            motion_ok=execution.ok,observation_overhead_separate=True)
        if not execution.ok:return StepResult(False,error=execution.error,diagnostics=diagnostics)
        try:observation=runtime.get_observation()
        except Exception as error:
            # The motion receipt remains successful; a camera error is neither a
            # controller failure nor an unknown physical outcome.
            return StepResult(False,error=ApiError(ErrorCode.ADAPTER_FAILED,
                'step_observation_failed_after_motion',details={'cause':str(error)}),diagnostics=diagnostics)
        return StepResult(True,observation=observation,diagnostics=diagnostics)

def pair_step(runtime, action):
    adapter = runtime.adapter
    if not isinstance(action, RobotAction) or action.embodiment != adapter.describe()['embodiment']:
        return StepResult(False, error=ApiError(ErrorCode.INVALID_REQUEST, 'action_embodiment_mismatch'))
    if not action.arms or not set(action.arms) <= set(adapter.arms):
        return StepResult(False, error=ApiError(ErrorCode.INVALID_REQUEST, 'action_requires_left_or_right'))
    goals = {side: command.target for side, command in action.arms.items()}
    hands = {side: command.gripper_position for side, command in action.arms.items()
             if command.gripper_position is not None}
    with runtime.lease():
        state = runtime.get_robot_state()
        try:
            if hands:
                adapter.check_hands(hands)
            trajectory, payload = adapter._plan(state, goals, synchronized=len(goals) == 2,
                                                 interpolation_dt_s=runtime.control_period_s)
            payload['stages'][-1]['hands'] = hands
        except (ValueError, ApiError) as exc:
            return StepResult(False, error=exc if isinstance(exc, ApiError) else
                              ApiError(ErrorCode.INVALID_REQUEST, str(exc)))
        result = runtime._dispatch('step', lambda: adapter.execute(payload, trajectory,
                                                                   permitted=runtime._segment_permitted))
        diagnostics = dict(result.diagnostics, control_period_s=runtime.control_period_s)
        if not result.ok:
            return StepResult(False, error=result.error, diagnostics=diagnostics)
        try:
            observation = runtime.get_observation()
        except Exception as exc:
            return StepResult(False, error=ApiError(ErrorCode.ADAPTER_FAILED, 'step_observation_failed_after_motion',
                                                     details={'cause': str(exc)}), diagnostics=diagnostics)
        return StepResult(True, observation=observation, diagnostics=diagnostics)
