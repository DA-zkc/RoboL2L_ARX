"""Gripper feedback is reported by the adapter without a project stability classifier.

Native reference completion does not establish physical arrival or object possession.
"""

ARX_GRIPPER_TIMING = (
    'ARX gripper commands use SDK direct-target mode (duration=None), including '
    'gripper changes at trajectory waypoints. gripper_finished means the SDK has '
    'no active gripper interpolation trajectory. In direct-target mode this can '
    'already be true when the target is submitted: the call may return immediately '
    'while the fingers are still moving. No physical closing dwell is guaranteed '
    'before the next program operation or arm waypoint.'
)

ARX_GRASP_SEMANTICS = (
    'When calibrated, normalized opening 0 commands the closed encoder target '
    'and 1 the open target; close_gripper is a position command, not force-contact '
    'grasping. ' + ARX_GRIPPER_TIMING + ' '
    'ok=True with completion=sdk_reference_finished reports command acceptance '
    'and SDK reference completion, not measured arrival or object possession. '
    'target_reached=None and physical_arrival_verified=False remain unverified; '
    'grasp_verified=False and contact_detected=None, when supplied, do not confirm '
    'a grasp. Measured angle and velocity are samples at receipt time. A single '
    'opening above the empty-close range cannot distinguish an object from fingers '
    'still closing, obstruction or stall. Interpret subsequent encoder and scene '
    'observations in the generated program; the tool does not select a waiting '
    'period, stability threshold or grasp-verification policy.'
)
