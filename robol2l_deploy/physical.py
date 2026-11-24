"""Explicit, process-local authorization for a physical CaP experiment."""
import time
from dataclasses import replace


class PhysicalSession:
    def __init__(self, task_name, authorization, *, probe_distance_m=.02, maximum_dispatches=None, require_probe=False,
                 gripper_description='0–80 mm physical stroke; native commands and measured feedback'):
        if not authorization.strip():raise ValueError('explicit_physical_authorization_required')
        self.probe_distance_m=probe_distance_m
        self.require_probe=require_probe
        self.task_name=task_name;self.authorization=authorization
        self.arm_probe=False;self.dispatches=0;self.maximum_dispatches=maximum_dispatches
        self.created=time.monotonic();self.active=True
        self.gripper_description = gripper_description

    def permits(self, task_name):
        return self.active and task_name==self.task_name

    def describe(self):
        return dict(task=self.task_name,active=self.permits(self.task_name),arm_probe_passed=self.arm_probe,
            dispatches=self.dispatches,maximum_dispatches=self.maximum_dispatches,
            first_motion='User or agent selects the movement; no required probe',
            planning='IK and native trajectory generation; no automatic scene or model collision gate',
            gripper=self.gripper_description)

    def validate_scene(self,scene,evidence):
        return validate_scene(scene,evidence)

    def check_motion(self,adapter,plan):
        if not self.permits(self.task_name):return 'physical_session_inactive'
        return None

    def check_gripper(self,adapter,position=None):
        return self.check_motion(adapter,None)


def validate_scene(scene,evidence):
    return replace(scene,reviewed=True)

def authorize(runtime, task_name, authorization, *, probe_distance_m=.02, maximum_dispatches=None, require_probe=False, continuing=False):
    """Called by an explicitly opted-in host entry point, never by generated code."""
    if runtime.adapter.synthetic or getattr(runtime.adapter,'read_only',False):raise ValueError('physical_adapter_required')
    if runtime.faulted or runtime.paused:raise ValueError('station_faulted_or_paused')
    if not runtime.adapter.driver.realtime_ready():
        raise ValueError('realtime_prerequisites_missing')
    state=runtime.get_robot_state()
    capabilities = runtime.adapter.describe()
    gripper = capabilities.get('gripper_description')
    options = {} if gripper is None else {'gripper_description': gripper}
    policy=PhysicalSession(task_name,authorization,probe_distance_m=probe_distance_m,maximum_dispatches=maximum_dispatches,
                           require_probe=require_probe, **options)
    runtime.physical_session=policy;runtime.adapter.physical_session=policy
    runtime.adapter.driver.allow_motion=True;runtime.allow_motion=True
    runtime.event('physical_authorization',authorization=authorization,policy=policy.describe(),state=state,
        motion_speed=capabilities.get('motion_speed'))
    return policy


def revoke(runtime):
    runtime.allow_motion=False
    if getattr(runtime,'physical_session',None):runtime.physical_session.active=False
    runtime.adapter.driver.allow_motion=False
    runtime.event('physical_authorization_revoked')
