"""Measured endpoint accuracy, separate from controller faults and unknown outcomes."""
from collections.abc import Mapping
import numpy as np
from scipy.spatial.transform import Rotation


def completed_motion(receipt):
    """Accept only normal completion evidence from the installed native backends."""
    if receipt.get('has_errors') is not False or receipt.get('mode_name') != 'idle':
        return False
    report = receipt.get('controller_report', {})
    if receipt.get('source') == 'physical_FR3_franky':
        return report.get('completed') is True
    if receipt.get('source') == 'physical_FR3_panda_py':
        duration = report.get('curve_duration_s', 0)
        elapsed = report.get('control_duration_s', -1)
        return (isinstance(duration, (int, float)) and isinstance(elapsed, (int, float))
            and np.isfinite([duration, elapsed]).all() and duration > 0 and elapsed >= duration)
    return False


def endpoint_accuracy(goal_joints, goal_pose, receipt, tolerance_rad=None, *, joint_count=7):
    joints = np.asarray(receipt['q'], dtype=float)
    delta = joints - np.asarray(goal_joints, dtype=float)
    actual = np.asarray(receipt['T_base_ee'], dtype=float)
    target = np.asarray(goal_pose['position'], dtype=float)
    quaternion = np.asarray(goal_pose['quaternion_wxyz'], dtype=float)
    if joints.shape != (joint_count,) or delta.shape != (joint_count,) or actual.shape != (4, 4) or target.shape != (3,) or quaternion.shape != (4,):
        raise ValueError('invalid_endpoint_measurement')
    if not all(np.isfinite(v).all() for v in (joints, delta, actual, target, quaternion)):
        raise ValueError('invalid_endpoint_measurement')
    desired = Rotation.from_quat(quaternion[[1, 2, 3, 0]])
    offset = actual[:3, 3] - target
    maximum = float(np.max(np.abs(delta)))
    return dict(frame='base', target_position_m=target.tolist(), actual_position_m=actual[:3, 3].tolist(),
        position_delta_mm=(offset*1000).tolist(), position_error_mm=float(np.linalg.norm(offset)*1000),
        orientation_error_deg=float(np.rad2deg((desired.inv()*Rotation.from_matrix(actual[:3, :3])).magnitude())),
        joint_delta_rad=delta.tolist(), max_joint_error_rad=maximum,
        max_joint_error_deg=float(np.rad2deg(maximum)), joint_tolerance_rad=None,
        joint_tolerance_deg=None, accuracy_verified=None,acceptance_policy='measured_deviation_only')


def get(value, key, default=None):
    return value.get(key, default) if isinstance(value, Mapping) else getattr(value, key, default)


def execution_value(value):
    """Unwrap human job envelopes without changing the public ExecutionResult."""
    if get(value, 'execution') is not None:return get(value, 'execution')
    if get(value, 'handle') is not None:return get(value, 'value')
    return value


def deviation_only(value):
    value = execution_value(value)
    error = get(value, 'error')
    diagnostics = get(value, 'diagnostics', {})
    receipt=get(diagnostics, 'native_receipt', {})
    if get(error,'message') in ('gripper_target_not_verified','gripper_target_not_reached'):
        return (get(value,'ok') is False and receipt.get('ok') is True
            and isinstance(receipt.get('width_m'),(float,int)) and np.isfinite(receipt['width_m']))
    if get(error,'message') in ('grasp_lost','untracked_held_object'):
        # Preserve old records, but a removed software inference must not keep
        # latching a normally completed native motion after reload.
        return get(value,'ok') is False and completed_motion(receipt)
    return (get(value, 'ok') is False and get(error, 'message') == 'endpoint_not_verified'
        and completed_motion(get(diagnostics, 'native_receipt', {})))


def motion_not_started(diagnostics):
    """The native init guard rejected the start before nextCommandImpl ran."""
    return (get(diagnostics,'driver')=='franky'
        and get(diagnostics,'cause_type')=='RuntimeError'
        and get(diagnostics,'cause_message')=='franky_trajectory_start_state_changed'
        and get(get(diagnostics,'control_trace',{}),'samples')==0)


def settled_pair_feedback(value):
    error = get(value, 'error')
    code = get(error, 'code')
    code = getattr(code, 'value', code)
    if code == 'goal_tolerance_not_met':
        return True
    if get(error, 'message') in ('paused_before_segment', 'paused_before_gripper_dispatch'):
        return True
    return code == 'stale_plan' and 'completed_waypoints' in get(value, 'diagnostics', {})


def stops_runtime(value):
    value = execution_value(value)
    error=get(value,'error');code=get(error,'code')
    if settled_pair_feedback(value):
        return False
    if motion_not_started(get(value,'diagnostics',{})) or motion_not_started(get(error,'details',{})):
        return False
    if getattr(code,'value',code)=='grasp_failed':
        receipt=get(get(value,'diagnostics',{}),'native_receipt',{})
        if (type(receipt.get('ok')) is bool and type(receipt.get('is_grasped')) is bool
            and isinstance(receipt.get('width_m'),(float,int)) and np.isfinite(receipt['width_m'])):
            return False  # A completed unsuccessful grasp is task feedback, not a controller fault.
    return get(value, 'ok') is False and not deviation_only(value)


def feedback(value):
    from .localization import chinese_error
    value = execution_value(value)
    diagnostics = get(value, 'diagnostics', {})
    accuracy = get(diagnostics, 'endpoint_accuracy')
    warning = deviation_only(value) or bool(get(diagnostics, 'warning_only', False))
    retryable=settled_pair_feedback(value) or motion_not_started(diagnostics) or motion_not_started(get(get(value,'error'),'details',{}))
    return dict(accuracy=accuracy, warning_only=warning, can_continue=get(value, 'ok') is True or warning or retryable,
        accuracy_verified=get(accuracy, 'accuracy_verified') if accuracy else None,
        error=get(get(value, 'error'), 'message'),
        error_zh=chinese_error(get(get(value,'error'),'message')) if get(value,'error') else None,
        diagnostics=diagnostics, result_ok=get(value, 'ok'))
