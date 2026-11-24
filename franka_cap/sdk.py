"""The single public API used by generated programs and the web console."""
from __future__ import annotations
import time
from pathlib import Path
from uuid import uuid4
import numpy as np
from . import devices, perception, planning
from .common import config, new_run, write_json, hardware_lock, digest, read_json
from .geometry import shifted, matrix, pose_matrix


class CapSDK:
    def __init__(self, station_config=None, run_dir=None, motion_profile=None):
        self.cfg=config(station_config)
        self.run_dir=Path(run_dir) if run_dir else new_run()
        self.run_dir.mkdir(parents=True,exist_ok=True)
        self.observations={};self.events=[]
        self._motion = None
        if motion_profile is not None:
            from .motion import MotionBackend
            self._motion = MotionBackend(self, motion_profile)
        self.capabilities={
            'station':self.cfg['station'],'robot':'FR3','arms':['single'],
            'cameras':{'wrist':{'serial':self.cfg['wrist_serial'],'rgb':True,'depth':True},
                       'global':{'serial':self.cfg['global_serial'],'rgb':True,'depth':True}},
            'coordinate_contract':{'frame':'base','position_unit':'meters','quaternion_order':'wxyz','target':'configured_EE'},
            'available':['robot_state','dual_camera_observation','color_regions','pixel_to_base','offline_6d_ik','sampled_path_collision_checks'],
            'motion_enabled':False,'execution_status':'not_commissioned',
            'unavailable':['physical_motion','arbitrary_Python_execution','semantic_text_segmentation','learned_grasp_proposals'],
            'program_contract':'def run(api), bounded Python subset; call api.describe() for signatures',
        }

    def _record(self, method, arguments, result):
        self.events.append({'time_s':time.time(),'method':method,'arguments':arguments,'result':result})
        write_json(self.run_dir/'events.json',self.events)
        return result

    def describe(self):
        result = dict(self.capabilities, methods={
            'get_robot_state()':'Actual FR3 joint/EE/gripper state; no motion',
            'get_observation(camera_names=None)':'Capture wrist/global images; returns observation id and explicit camera metadata',
            'localize_pixel(observation_id, camera, pixel, radius=2)':'Calibrated visible-surface 3D point; requires aligned depth',
            'detect_color(observation_id, camera, color)':'HSV regions; no semantic object claims',
            'offset_pose(pose, delta_base_m)':'Pure geometry; adds metres in base frame',
            'solve_ik(target_pose)':'Offline joint solution from fresh physical state; not a motion plan',
            'plan_motion(target_pose)':'Checked small 6D Cartesian path from current physical state; does not move',
            'move_to_pose(target_pose) / set_gripper(position)':'Fail explicitly: motion not commissioned',
        })
        if self._motion is not None:
            result.update(motion_enabled=True,execution_status='guarded_task_session',
                          execution_policy='guarded_motion',
                          commissioning={'arm_probe_passed':self._motion.initial_probe_passed,
                                         'gripper_probe_passed':self._motion.gripper_probe_passed},
                          environment_geometry={'support_point_m':self._motion.profile['support_point_m'],'support_normal':self._motion.profile['support_normal'],'plate_center_m':self._motion.profile['plate_center_m'],'plate_release_center_tolerance_m':.025},
                          workspace_min_m=self._motion.profile['workspace_min_m'],workspace_max_m=self._motion.profile['workspace_max_m'],
                          motion_constraints={'translation_per_call_m':.05,'rotation_per_call_rad':.15,'one_physical_command_per_program':True,
                                              'gripper_width_m':[.02,.07],'grasp_force_n':[3,10],'downward_tool_axis_z_max':-.85})
            result['unavailable']=[x for x in result['unavailable'] if x!='physical_motion']
            result['methods'].pop('move_to_pose(target_pose) / set_gripper(position)',None)
            result['methods'].update({
                'move_to_pose(target_pose)':'Real bounded movement: internally plans/checks the full path, then executes. First movement <=5 mm above z=0.20 m; later <=5 cm and <=0.15 rad. One physical command per program. Pose is base-frame configured EE, quaternion wxyz.',
                'set_gripper(position)':'position=width/0.08. Empty commissioning: change width <=5 mm above z=.20 m. Opening/release only after that; closing around object uses grasp. Held-object release only low over plate.',
                'measure_region(observation_id,camera,bbox_xywh,color=None)':'Aligned RGB-D surface statistics in base frame. For green object use color="green"; a non-truncated region produces object_id required by grasp. Hidden surfaces remain uncertain; obtain another view if needed.',
                'step_toward(start_pose,goal_pose)':'Pure geometry: returns a local pose <=4 cm translation and <=0.12 rad rotation along the 6D path to a desired goal; does not move.',
                'grasp(width_m,force_n=5,object_id=None)':'Real force-limited grasp of the measured green region. Requires arm/gripper probes, an object_id, and EE in the measured gripping region. Confirms measured grasp before allowing lift.'})
        return result

    def get_robot_state(self):
        with hardware_lock(): result=devices.robot_state(self.cfg)
        return self._record('get_robot_state',{},result)

    def get_observation(self,camera_names=None):
        names=['wrist','global'] if camera_names is None else camera_names
        with hardware_lock():
            obs=devices.capture(self.cfg,self.run_dir/('obs-'+uuid4().hex[:12]),names)
        self.observations[obs['id']]=obs
        return self._record('get_observation',{'camera_names':names},obs)

    def _camera(self,observation_id,camera):
        if observation_id not in self.observations:raise ValueError('Unknown observation id; acquire in this SDK session first')
        obs=self.observations[observation_id]
        if camera not in obs['cameras']:raise ValueError('Camera not acquired')
        cam=obs['cameras'][camera]
        if digest(self.cfg[camera+'_calibration'])!=cam['calibration_sha256']:raise ValueError('Calibration changed since observation')
        return cam

    def localize_pixel(self,observation_id,camera,pixel,radius=2):
        result=perception.localize_pixel(self._camera(observation_id,camera),pixel,radius)
        return self._record('localize_pixel',{'observation_id':observation_id,'camera':camera,'pixel':pixel},result)

    def detect_color(self,observation_id,camera,color):
        result=perception.color_regions(self._camera(observation_id,camera),color)
        return self._record('detect_color',{'observation_id':observation_id,'camera':camera,'color':color},result)

    def offset_pose(self,pose,delta_base_m):
        return shifted(pose,delta_base_m)

    def solve_ik(self,target_pose):
        pose_matrix(target_pose)
        state=self.get_robot_state();cal,_,sha=devices.calibration(self.cfg,'wrist',state)
        model=planning.Kinematics()
        current=model.forward(state['q']);actual=matrix(state['T_base_ee'])
        q=model.solve(pose_matrix(target_pose),state['q'],self.cfg['planning']['joint_margin_rad'])
        result={'joint_positions':q.tolist(),'target_pose':target_pose,'executed':False,'collision_checked':False,'calibration_sha256':sha,
                'fk_position_residual_m':float(np.linalg.norm(current[:3,3]-actual[:3,3])),
                'ik_position_residual_m':float(np.linalg.norm(model.forward(q)[:3,3]-pose_matrix(target_pose)[:3,3]))}
        return self._record('solve_ik',{'target_pose':target_pose},result)

    def plan_motion(self,target_pose):
        pose_matrix(target_pose)
        with hardware_lock():
            state=devices.robot_state(self.cfg)
            cal,_,sha=devices.calibration(self.cfg,'wrist',state)
            result=planning.plan_path(state,target_pose,self.cfg,cal)
            after=devices.robot_state(self.cfg)
            result['start_measurement']={'joint_delta':(np.array(after['q'])-state['q']).tolist(),'velocities':after['dq']}
        result.update(id='plan-'+uuid4().hex[:12],created_s=time.time(),calibration_sha256=sha,station=self.cfg['station'])
        result['path']=str(self.run_dir/(result['id']+'.json'))
        write_json(result['path'],result)
        return self._record('plan_motion',{'target_pose':target_pose},result)

    def move_to_pose(self,target_pose):
        pose_matrix(target_pose)
        if self._motion is None:
            raise RuntimeError('motion_not_commissioned: use plan_motion for offline validation; no movement was sent')
        result=self._motion.move(target_pose)
        return self._record('move_to_pose',{'target_pose':target_pose},result)

    def set_gripper(self,position):
        from .common import scalar
        scalar(position,'position',0,1)
        if self._motion is None:
            raise RuntimeError('motion_not_commissioned: gripper actuation is disabled; no command was sent')
        result=self._motion.gripper(position=position)
        return self._record('set_gripper',{'position':position},result)

    def measure_region(self,observation_id,camera,bbox_xywh,color=None):
        if self._motion is None:raise ValueError('Region geometry tool requires the installed motion runtime')
        result=self._motion.measure(self._camera(observation_id,camera),bbox_xywh,color)
        return self._record('measure_region',{'observation_id':observation_id,'camera':camera,'bbox_xywh':bbox_xywh,'color':color},result)

    def step_toward(self,start_pose,goal_pose):
        from scipy.spatial.transform import Rotation,Slerp
        from .geometry import pose_dict
        start=pose_matrix(start_pose);goal=pose_matrix(goal_pose)
        distance=float(np.linalg.norm(goal[:3,3]-start[:3,3]));angle=float(np.linalg.norm(Rotation.from_matrix(start[:3,:3].T@goal[:3,:3]).as_rotvec()))
        fraction=min(1.,.04/max(distance,1e-12),.12/max(angle,1e-12))
        out=start.copy();out[:3,3]+=fraction*(goal[:3,3]-start[:3,3])
        out[:3,:3]=Slerp([0,1],Rotation.from_matrix([start[:3,:3],goal[:3,:3]]))(fraction).as_matrix()
        return pose_dict(out)

    def grasp(self,width_m,force_n=5,object_id=None):
        if self._motion is None:raise RuntimeError('motion_not_commissioned')
        result=self._motion.gripper(width=width_m,force=force_n,object_id=object_id)
        return self._record('grasp',{'width_m':width_m,'force_n':force_n,'object_id':object_id},result)

    def close(self):
        # Acquisition methods release pipelines before returning. Never Home or actuate here.
        pass

    def __enter__(self): return self
    def __exit__(self,*args): self.close()
