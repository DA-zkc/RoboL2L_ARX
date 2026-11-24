"""Task checks stay outside the program and hardware drivers."""
from dataclasses import dataclass
import numpy as np
from ..contracts import TaskContext


@dataclass(frozen=True)
class MockReachTask:
    context: TaskContext = TaskContext('deployment-mock',0,'mock-reach-and-gripper',
        'Move the mock end effector to [0, 0, 0.31] metres in base, retain orientation, '
        'and set the mock gripper to 0.5. This is synthetic software validation.',
        'mock',{'target_position_m':[0,0,.31],'gripper_position':.5})

    def verify(self,runtime):
        if not runtime.adapter.synthetic:
            return {'passed':False,'physical_task_success':False,'reason':'mock task requires synthetic adapter'}
        state=runtime.get_robot_state()
        position_error=float(np.linalg.norm(state.end_effector_poses['primary'].position-[0,0,.31]))
        gripper_error=abs(state.gripper_positions['primary']-.5)
        quaternion=state.end_effector_poses['primary'].quaternion_wxyz
        # MockAdapter starts at identity; q and -q represent the same rotation.
        orientation_error=2*float(np.arccos(np.clip(abs(quaternion[0]),0,1)))
        return {'passed':position_error<1e-6 and gripper_error<1e-6 and orientation_error<1e-6,
                'synthetic':True,'position_error_m':position_error,'orientation_error_rad':orientation_error,
                'gripper_error':gripper_error,'physical_task_success':False}


def prepare_pick_place():
    """No hardware calls or object coordinates inferred from previous experiments."""
    return {
        'task':{'suite':'deployment','task_id':1,'task_name':'green-cube-to-plate',
                'language':'Pick up the green cube and place it inside the plate.',
                'family':'single-arm-pick-place'},
        'status':'available_from_cap_task_panel',
        'motion_enabled':False,'physical_task_success':False,
        'available':['robot state','calibrated RGB-D','stationary pixel localization',
                     'IK and native pose trajectory planning (collision unchecked)','versioned CaP feedback',
                     'Native gripper width/contact-close commands and measured feedback'],
        'required_before_execution':[
            'Fresh semantic object/plate localization and uncertainty checks; do not reuse old pixels or poses.',
            'Agent-selected grasp and placement geometry; grasp-network inference is optional.',
            'Commission the installed contact-grasp/holding/release adapter on hardware; software tests are not physical evidence.',
            'Agent-selected feasible approach/lift/transfer/place waypoints; no mandatory scene model.',
            'Independent before/after evidence for lift, release and object retained inside the plate.',
            'Start the physical task explicitly in the panel; the installed Runtime dispatches generated API calls.'],
        'success_evidence':['object visibly lifted with retained grasp',
                            'object released inside plate and stable in later frames',
                            'gripper open and arm clear'],
        'program_contract':'RoboL2L public tools, six constructors, top-level result, captured print',
    }


@dataclass(frozen=True)
class VisualPreparationTask:
    """A read-only deliverable. Its evaluator is never sent to the model."""
    read_only: bool = True
    context: TaskContext = TaskContext('deployment',1,'green-cube-to-plate-preparation',
        'Prepare to pick up the green cube and place it inside the plate. Use the camera observations '
        'and public perception APIs to identify the objects and obtain a grasp and placement pose. '
        'Prepare a feasible approach motion from the current robot state. This session is read-only: '
        'do not execute motion or operate the gripper. Return your observation-derived proposal and '
        'planning results. Finish only when those public results support the preparation.',
        'visual-pick-place',{'mode':'read_only_preparation'})

    def verify(self,runtime):
        # Tool selection and completion belong to the agent, not a forced pipeline.
        return dict(required=False,passed=False,physical_task_success=False,
            full_pick_place_plan_verified=False,
            reason='agent_reports_preparation; no_required_perception_pipeline')
