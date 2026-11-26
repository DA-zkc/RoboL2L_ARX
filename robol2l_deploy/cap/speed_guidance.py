"""ARX speed semantics shared by the tool description and model instructions."""

ARX_SPEED_GUIDANCE = '''ARX uses its installed SDK planner timing and native limits.
Despite its name, MotionStrategy.time_dilation_factor is a SPEED factor s in this
adapter, not a multiplier applied to duration. Smaller positive s means SLOWER.
Without an explicit interpolation_dt_s, duration is max(T_sdk, T_sdk / s), rounded
up to the SDK time quantum. For the same start and goal, s=0.1 gives approximately
10 times the SDK duration; it does NOT compress execution time. s=1 uses SDK timing;
s>1 does not shorten the duration below the SDK-planned duration.
Set it explicitly on each plan with
MotionStrategy(ik_solver="station", pose_planner="station", time_dilation_factor=s).
strategy=None (or an omitted factor) uses the configured adapter default, normally
1.0; it does NOT inherit experiment.operator_speed_recommendation or a prior plan.
The operator's recommendation is advisory, not a host-enforced cap. When supplied,
its time_dilation_factor is already input_multiplier * default_speed_factor.
Use that computed value as the starting choice; do not multiply it again.
Pass the chosen factor explicitly;
explain any departure using the actual speed semantics and task evidence.
Historical reasoning that 0.1 compresses time, or that choosing 1.0 instead makes
the same motion slower, is incorrect and must not be repeated.
Check plan.diagnostics.speed_factor and duration_s before execution. This factor
does not set gripper speed or change controller gains. No project joint-speed cap
or default slowdown is added. This is a CaP adapter convention, not an ARX SDK
speed-factor parameter.'''
