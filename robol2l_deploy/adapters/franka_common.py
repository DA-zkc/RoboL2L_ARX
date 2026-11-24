"""State conversion and caller scheduling shared by libfranka-based adapters."""
import os
import time
from contextlib import contextmanager
import numpy as np


@contextmanager
def library_thread_policy():
    """libfranka may promote its caller; restore it before Python perception.

    The native control loop retains enforced realtime scheduling while running.
    Python serialization/planning must not inherit that realtime priority.
    """
    policy=os.sched_getscheduler(0);parameters=os.sched_getparam(0)
    try:yield
    finally:
        if os.sched_getscheduler(0)!=policy or os.sched_getparam(0)!=parameters:
            os.sched_setscheduler(0,policy,parameters)


def robot_state(state, hand, backend, *, has_errors=None):
    def matrix(value):
        value = value.matrix if hasattr(value, 'matrix') else value
        return np.asarray(value).reshape((4,4),order='F').tolist()
    mode=int(state.robot_mode.value) if hasattr(state.robot_mode,'value') else int(state.robot_mode)
    modes=['other','idle','move','guiding','reflex','user_stopped','automatic_error_recovery']
    return dict(q=list(state.q),q_d=list(state.q_d),dq=list(state.dq),dq_d=list(state.dq_d),
        T_base_ee=matrix(state.O_T_EE),T_base_ee_commanded=matrix(state.O_T_EE_c),
        F_T_EE=matrix(state.F_T_EE),EE_T_K=matrix(state.EE_T_K),
        cartesian_contact=list(state.cartesian_contact),external_wrench=list(state.O_F_ext_hat_K),
        joint_external_torque=list(state.tau_ext_hat_filtered),joint_torque=list(state.tau_J),
        commanded_joint_torque=list(state.tau_J_d),control_command_success_rate=float(state.control_command_success_rate),
        load=dict(ee_mass_kg=float(state.m_ee),payload_mass_kg=float(state.m_load),total_mass_kg=float(state.m_total),
            flange_com_ee_m=list(state.F_x_Cee),flange_com_payload_m=list(state.F_x_Cload),
            flange_com_total_m=list(state.F_x_Ctotal),ee_inertia=np.asarray(state.I_ee).flatten(order='F').tolist(),payload_inertia=np.asarray(state.I_load).flatten(order='F').tolist()),
        has_errors=bool(state.current_errors) if has_errors is None else bool(has_errors),mode=mode,mode_name=modes[mode] if 0<=mode<len(modes) else 'unknown',
        gripper=dict(width_m=hand.width,max_width_m=hand.max_width,is_grasped=hand.is_grasped,temperature_c=hand.temperature),
        timestamp_s=time.time(),host_monotonic_s=time.monotonic(),source='physical_FR3_'+backend,synthetic=False)
