"""Physical task language; completion judgment belongs to the agent."""
from ..contracts import TaskContext


class PhysicalProbeTask:
    """A user-requested control diagnostic, separate from manipulation success."""
    def __init__(self,distance_m):
        self.context=TaskContext('deployment',2,'arm-tracking-probe',
            f'Execute exactly one {distance_m*1000:g} mm upward clearance probe from the current TCP pose, '
            'with unchanged orientation, after fresh automatic observation and registered planning. '
            'Follow the physical_session clearance requirements. Check the execution feedback and stop. '
            'Do not grasp, move the gripper, return, repeat, Home or recover. '
            'This is a control diagnostic, not the green cube pick/place task.',
            'arm-tracking',{'mode':'physical_probe','distance_m':distance_m})

    def verify(self,runtime):
        passed=bool(runtime.physical_session and runtime.physical_session.arm_probe and not runtime.faulted)
        return dict(passed=passed,probe_verified=passed,physical_task_success=False,
                    reason='verified_motion_receipt' if passed else 'probe_not_verified')


class DualJointTask:
    """An operator-defined joint task with encoder evidence for the model's review."""
    def __init__(self, goal):
        if not isinstance(goal, str) or not goal.strip():
            raise ValueError('dual_joint_goal_required')
        self.context = TaskContext('deployment', 3, 'dual-joint-task', goal.strip(),
                                   'dual-arm-joints', {'mode': 'joint_feedback'})

    def verify(self, runtime):
        return dict(required=False, passed=False, physical_task_success=False,
                    reason='use_measured_per_arm_execution_and_encoder_feedback')


class PhysicalPickPlaceTask:
    context=TaskContext('deployment',1,'green-cube-to-plate',
        'Pick up the green cube and place it inside the plate, then leave the gripper open and arm clear. '
        'This physical experiment is explicitly authorized. Use actual feedback and fresh observations. '
        'Inspect the provided camera images and choose which public perception and planning '
        'tools to call. Segmentation and grasp generation are optional; closing does not '
        'require target registration. Check every result. Revise recoverable perception, IK and grasp '
        'failures using fresh observations. Stop on actual controller faults or unknown '
        'execution outcomes; never reset the controller or Home automatically. '
        'Use only observed geometry to choose placement. Claim completion only from fresh visual '
        'evidence after release, not from motion success.',
        'single-arm-pick-place',{'mode':'physical_task'})

    def verify(self,runtime):
        # Task judgment belongs to the agent. No numeric scorer gates its finish.
        return dict(required=False,passed=False,physical_task_success=False,
            reason='agent_reports_completion; independent_numeric_evaluator_disabled')
