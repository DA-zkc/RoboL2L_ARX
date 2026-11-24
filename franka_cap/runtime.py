"""Main-pipeline adapter: read-only CaP programs backed by the project SDK."""
import base64
import hashlib
import time
from pathlib import Path
from uuid import uuid4
from .sdk import CapSDK
from .program import Program, ProgramPaused
from .common import write_json

READ_ONLY_TOOLS = {'describe', 'get_robot_state', 'get_observation', 'localize_pixel',
                   'detect_color', 'offset_pose', 'solve_ik', 'plan_motion'}


class FrankaRuntime:
    def __init__(self, sdk=None):
        self.sdk = sdk or CapSDK()
        self.event_start = len(self.sdk.events)
        self.policy = "read_perceive_plan"

    def validate(self, config):
        if config.get('station') != self.sdk.cfg['station']:
            raise ValueError('station_mismatch')
        self.policy=config.get('_execution_policy','read_perceive_plan')
        if self.policy=='guarded_motion':
            if self.sdk._motion is None or config.get('_task_goal_sha256')!=self.sdk._motion.profile['goal_sha256']:
                raise ValueError('motion_task_not_authorized')
        elif self.policy!='read_perceive_plan':raise ValueError('Unknown execution policy')

    def describe(self):
        description = self.sdk.describe()
        description['methods'] = {k: v for k, v in description['methods'].items()
                                  if k.split('(')[0] in self.allowed_tools()}
        description['execution_policy'] = self.policy
        if self.policy!='guarded_motion':description['motion_enabled']=False
        return description

    def allowed_tools(self):
        return READ_ONLY_TOOLS | ({"move_to_pose","set_gripper","grasp","measure_region","step_toward"} if self.policy=="guarded_motion" else set())

    def observe(self):
        obs = self.sdk.get_observation()
        images = []
        for name, cam in obs['cameras'].items():
            path = Path(cam['rgb_path']); mime = 'image/png' if path.suffix == '.png' else 'image/jpeg'
            images.append({'name': name, 'data_url': 'data:' + mime + ';base64,' + base64.b64encode(path.read_bytes()).decode()})
        return dict(obs, images=images)

    def execute_program(self, source, should_pause=lambda: False):
        directory = self.sdk.run_dir / ('program-' + uuid4().hex[:12]); directory.mkdir()
        (directory / 'source.py').write_text(source)
        if getattr(self.sdk, '_motion', None) is not None:self.sdk._motion.begin_program(should_pause)
        interpreter = Program(self.sdk, allowed_tools=self.allowed_tools(), should_pause=should_pause)
        result = {'id': directory.name, 'source_sha256': hashlib.sha256(source.encode()).hexdigest(),
                  'source_path': str(directory / 'source.py'), 'result_path': str(directory / 'result.json'),
                  'motion_sent': False, 'started_s': time.time()}
        try:
            result.update(ok=True, status='returned', value=interpreter.run(source))
        except ProgramPaused:
            result.update(ok=False, status='paused', error='pause_requested_before_next_api_call')
        except Exception as error:
            # SDK has no model credentials. Never include transport exceptions/response bodies here.
            result.update(ok=False, status='failed', error_type=type(error).__name__, error=str(error)[:2000])
        result.update(api_calls=interpreter.calls, finished_s=time.time())
        if getattr(self.sdk, '_motion', None) is not None:
            result['motion_sent']=bool(self.sdk._motion.calls)
            result['fatal_motion_fault']=self.sdk._motion.failed
        write_json(directory / 'result.json', result)
        return result

    def validation_evidence(self):
        if self.policy=="guarded_motion":return self.physical_evidence()
        events = self.sdk.events[self.event_start:]
        observations = [e['result'] for e in events if e['method'] == 'get_observation']
        reads = [e for e in events if e['method'] == 'get_robot_state']
        colors = [e for e in events if e['method'] == 'detect_color' and e['result']]
        surfaces = [e for e in events if e['method'] == 'localize_pixel']
        plans = [e['result'] for e in events if e['method'] == 'plan_motion'
                 and e['result'].get('status') == 'offline_path_checked'
                 and e['result'].get('executed') is False and e['result'].get('motion_executable') is False]
        checks = {'robot_read': bool(reads or any(o.get('robot') for o in observations)),
                  'dual_rgbd': any(all(o.get('cameras', {}).get(n, {}).get('depth_available')
                                      for n in ('wrist', 'global')) for o in observations),
                  'color_detection': bool(colors), 'surface_localization': bool(surfaces),
                  'offline_plan': bool(plans)}
        return {'checks': checks, 'passed': all(checks.values()),
                'observation_ids': [o['id'] for o in observations],
                'surface_points': [e['result'] for e in surfaces][-8:],
                'plan_ids': [p['id'] for p in plans], 'motion_sent': False,
                'physical_task_success': False, 'scope': 'read_perceive_plan',
                'limitations': ['Color regions are not verified semantic identity or grasp poses.',
                                'A small local probe is not a complete pick/place plan.',
                                'No object was lifted or placed by this validation.']}

    def physical_evidence(self):
        import numpy as np
        from . import perception
        motion=self.sdk._motion
        checks={'confirmed_grasp':motion.grasp_seen,'confirmed_lift':motion.lift_seen,
                'confirmed_release':motion.release_seen,'stable_object_in_plate':False,'gripper_open':False,'arm_retracted':False}
        positions=[];observations=[]
        if motion.latest_release:
            for event in self.sdk.events[self.event_start:]:
                if event['method']!='get_observation':continue
                obs=event['result']
                if obs['timestamp_s']<=motion.latest_release['timestamp_s']:continue
                cam=obs['cameras'].get('global')
                if not cam:continue
                plate=np.array(motion.profile['plate_center_m']);candidates=[]
                for region in perception.color_regions(cam,'green'):
                    if region['area_px']<100:continue
                    try:point=perception.localize_pixel(cam,region['pixel'])['point_base_m']
                    except ValueError:continue
                    if np.linalg.norm(np.array(point)[:2]-plate[:2])<.025 and .015<point[2]-plate[2]<.09:
                        candidates.append(point)
                if len(candidates)==1:
                    positions.append(candidates[0]);observations.append(obs['id'])
                robot=obs['robot'];checks['gripper_open']=not robot['gripper']['is_grasped'] and robot['gripper']['width_m']>.07
                checks['arm_retracted']=np.array(robot['T_base_ee'])[2,3]>.12
            if len(positions)>=2:checks['stable_object_in_plate']=float(np.linalg.norm(np.array(positions[-1])-positions[-2]))<.005
        passed=all(checks.values()) and not motion.failed
        return {'checks':checks,'passed':passed,'physical_task_success':passed,'scope':'guarded_motion',
                'motion_sent':any(e['method'] in ('move_to_pose','grasp','set_gripper') for e in self.sdk.events),
                'verification_observation_ids':observations[-2:],'observed_object_surface_points_m':positions[-2:]}

    def act(self, tool, arguments):
        raise RuntimeError('motion_not_commissioned')

    def close(self):
        self.sdk.close()


def create_runtime(config):
    return FrankaRuntime()
