"""Offline, sequential FR3 path preview. This module never dispatches a device command."""
import copy
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation, Slerp
from franka_cap.planning import plan_path, Kinematics
from .contracts import Pose
from .manipulation import Manipulation
from .adapters.fr3_scene import PayloadCollisionScene
from .runtime import public


def split_pose(start, goal, translation=.04, rotation=.12):
    first, last = start.as_matrix(), goal.as_matrix()
    angle = Rotation.from_matrix(first[:3,:3].T@last[:3,:3]).magnitude()
    count = max(1, int(np.ceil(np.linalg.norm(last[:3,3]-first[:3,3])/translation)), int(np.ceil(angle/rotation)))
    rotations = Slerp([0,1],Rotation.from_matrix([first[:3,:3],last[:3,:3]]))
    for t in np.linspace(0,1,count+1)[1:]:
        yield Pose(first[:3,3]+t*(last[:3,3]-first[:3,3]),rotations(t).as_quat()[[3,0,1,2]],'base')


def pose_of(matrix):
    matrix=np.asarray(matrix)
    return Pose(matrix[:3,3],Rotation.from_matrix(matrix[:3,:3]).as_quat()[[3,0,1,2]],'base')


def preview(raw_state, scene, operations, cfg, calibration, directory):
    """Future starts are predictions, never registered executable Runtime plans."""
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    state=copy.deepcopy(raw_state);manipulation=Manipulation(scene);kin=Kinematics()
    records=[];paths=[]
    for operation in operations:
        record={'operation':operation['name'],'kind':operation['kind'],'status':'pending'}
        records.append(record)
        try:
            if operation['kind']=='motion':
                target=operation['pose'];segments=[];record['segments']=segments
                for goal in split_pose(pose_of(state['T_base_ee']),target):
                    factory=lambda cal,obstacles:PayloadCollisionScene(cal,obstacles,manipulation)
                    plan=plan_path(state,public(goal),cfg,calibration,scene_factory=factory)
                    plan['evidence']='offline prediction from frozen physical observation'
                    plan['holding_object']=manipulation.holding
                    index=sum(len(r.get('segments',[])) for r in records)
                    name=f'segment-{index:03d}.json'
                    (directory/name).write_text(json.dumps(public(plan),allow_nan=False))
                    u=np.linspace(0,1,101);s=u**4*(35+u*(-84+u*(70-20*u)))
                    q=CubicSpline(np.linspace(0,1,len(plan['q_samples'])),plan['q_samples'],bc_type='natural')(s)
                    xyz=np.array([kin.forward(row)[:3,3] for row in q]);paths.append(xyz)
                    segments.append({'artifact':name,'duration_s':plan['duration_s'],'preview_xyz':xyz.tolist()})
                    state=copy.deepcopy(state)
                    state.update(q=q[-1].tolist(),q_d=q[-1].tolist(),dq=[0]*7,T_base_ee=kin.forward(q[-1]).tolist())
            elif operation['kind']=='grasp':
                ee=np.asarray(state['T_base_ee']);box,width=manipulation.grasp_candidate(ee)
                if not 0<=width<=.08:raise ValueError('grasp_width_outside_native_limits')
                collision=PayloadCollisionScene(calibration,cfg['planning']['obstacles'],manipulation,box.name)
                try:
                    for value in np.linspace(state['gripper']['width_m'],width,21):
                        collision.set_state(state['q'],value)
                        if collision.violations():raise ValueError('gripper_sweep_collision')
                finally:collision.close()
                manipulation.attach(box,ee,width)
                state=copy.deepcopy(state);state['gripper'].update(width_m=width,is_grasped=True)
                record['assumption']='contact grasp succeeds; real execution must verify its receipt'
            elif operation['kind']=='release':
                collision=PayloadCollisionScene(calibration,cfg['planning']['obstacles'],manipulation)
                try:
                    for value in np.linspace(state['gripper']['width_m'],.08,21):
                        collision.set_state(state['q'],value)
                        if collision.violations():raise ValueError('release_sweep_collision')
                finally:collision.close()
                manipulation.release(np.asarray(state['T_base_ee']))
                state=copy.deepcopy(state);state['gripper'].update(width_m=.08,is_grasped=False)
                record['assumption']='released object stays at predicted pose; fresh RGB-D must verify settling'
            else:raise ValueError('unknown_preview_operation')
            record['status']='checked'
        except ValueError as error:
            record.update(status='blocked',error=str(error))
            break
    report=dict(status='checked' if len(records)==len(operations) and all(r['status']=='checked' for r in records) else 'blocked',
        physical_motion_executed=False,physical_task_success=False,executable=False,
        observation_id=scene.observation_id,operations=records,
        unchecked_operations=[o['name'] for o in operations[len(records):]],
        limitations=['future states and successful contact grasp are predictions','sampled collision checks, not a physical trial',
                     'execution must reobserve and replan; these files are not registered Runtime trajectories'])
    (directory/'preview.json').write_text(json.dumps(public(report),indent=2,allow_nan=False))
    return report,paths
