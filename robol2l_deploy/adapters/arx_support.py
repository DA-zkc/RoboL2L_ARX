"""Preserve active support on adapter errors; never override native protection.

PROTECT is damping, not a pose lock. Only an already healthy JOINT controller
can accept a replacement position target. This is not a mechanical brake and
an accepted request is not evidence that the physical arm stayed in place.
"""
import numpy as np


def preserve_support(unit, *, cancel_motion):
    receipt = dict(strategy='retain_position_control', hold_verified=False,
                   gripper_target_unchanged=True)
    try:
        arm = unit.driver.arm
        status = arm.get_status()
        receipt['native_mode'] = status.get('mode')
        if (arm.fault or arm.offline_joints or status.get('fault') or status.get('offline')
                or status.get('gripper_offline') or status.get('overlimit')):
            receipt.update(strategy='native_safety_retained', available=False,
                           reason='native_fault_or_offline_or_overlimit')
            return receipt
        if status.get('mode') != 'JOINT':
            receipt.update(available=False, reason='native_position_control_not_active')
            return receipt
        q, velocity = (np.asarray(status[key], dtype=float) for key in ('q', 'v'))
        if q.shape != (6,) or velocity.shape != (6,) or not np.isfinite([q, velocity]).all():
            raise ValueError('invalid_arx_support_feedback')
        receipt.update(available=True, measured_joints=q.copy(), measured_velocities=velocity.copy())
        if cancel_motion:
            # Public SDK target replacement keeps JOINT gains/feedforward and
            # replans with its native velocity/acceleration/jerk limits. Duration
            # zero asks the SDK to choose that duration, not an instantaneous stop.
            from .arx import JOINT_NAMES
            unit._joints(q, measured=True)
            target = np.clip(q, unit.kin.q_min, unit.kin.q_max)
            receipt.update(strategy='replace_motion_with_measured_position', target_joints=target,
                           request_accepted=bool(arm.set_joint_positions(
                               positions=target, joint_names=JOINT_NAMES, duration=0.)))
            if not receipt['request_accepted']:
                receipt.update(available=False, reason='hold_target_rejected')
        return receipt
    except Exception as error:
        receipt.update(available=False, reason=str(error))
        return receipt
