"""FR3 API adapter with one startup-selected control library. No implicit connection."""
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation
from ..contracts import Pose,RobotState,Observation,CameraObservation,Trajectory,IKResult,ExecutionResult
from ..errors import ApiError,ErrorCode
from ..runtime import fingerprint,OutcomeUnknown
from ..cameras import rectify

class FR3Adapter:
    synthetic=False
    def __init__(self,sdk,cameras=None,*,driver=None,backend='panda_py'):
        from .drivers import make_driver
        self.sdk=sdk;self.raw=None
        self.control_period_s=float(sdk.cfg.get("control_period_s",1./30.))
        if not np.isfinite(self.control_period_s) or self.control_period_s<=0:raise ValueError("invalid_control_period")
        self.driver=driver if driver is not None else make_driver(backend,sdk.cfg["robot_ip"],motion=sdk.cfg.get('motion'))
        if driver is None:self.driver.trace_directory=sdk.run_dir/'control-traces'
        if cameras is None:
            from ..cameras import CameraOwner
            from .rgbd import RealSenseSource,GeminiSource
            cameras={'wrist':CameraOwner(RealSenseSource(sdk.cfg['wrist_serial'])),
                     'global':CameraOwner(GeminiSource(sdk.cfg['global_serial']))}
        self.cameras=cameras
    def configuration(self):
        from franka_cap.common import read_json
        return dict(station=self.sdk.cfg,robot_backend=getattr(self.driver,'backend','custom'),controller=getattr(self.driver,'control_profile',None),
                    calibrations={name:read_json(self.sdk.cfg[name+'_calibration']) for name in ('wrist','global')})
    def describe(self):
        return dict(synthetic=False,embodiment='franka_fr3',arms=['primary'],cameras=list(self.cameras),
                    robot_backend=getattr(self.driver,'backend','custom'),
                    planning_backend='scipy_ik+'+getattr(self.driver,'planner','custom'), gripper_normalization_width_m=.08,
                    motion_speed=dict(default_speed_factor=getattr(self.driver,'speed_factor',.05),
                                      max_speed_factor=getattr(self.driver,'max_speed_factor',1.),
                                      parameter='MotionStrategy.time_dilation_factor',source='station_configuration',
                                      library_minimum_speed_factor=getattr(self.driver,'minimum_speed_factor',None)),
                    accepted_goals=['base_frame_pose','joint_positions','public_trajectory'], gripper_allowed_width_m=[0.,.08],
                    planning_limits={'joint_limits':'FR3 URDF','collision_checked':False},
                    execution_policy=dict(automatic_scene=False,model_collision_gate=False,
                        grasp_target_required=False,held_object_consistency_gate=False),
                    contact_grasp=True, holding_object=self.raw['gripper']['is_grasped'] if self.raw else None,
                    holding_state_source='native_gripper_is_grasped',
                    grasp_force_n=5., grasp_monitoring='native_feedback_only')
    def device_binding(self):
        from franka_cap.common import digest
        from franka_cap.planning import MODEL
        return fingerprint(dict(config=self.sdk.cfg,calibrations={n:digest(self.sdk.cfg[n+'_calibration']) for n in ('wrist','global')},
                                model=digest(MODEL/'fr3_hand.urdf')))
    def binding(self):
        return fingerprint(dict(device=self.device_binding(),
            backend=getattr(self.driver,'backend','custom'),
            controller=getattr(self.driver,'control_profile',None)))
    def preview_xyz(self,trajectory):
        from franka_cap.planning import Kinematics
        kin=Kinematics()
        return [kin.forward(q)[:3,3].tolist() for q in trajectory.joint_positions]
    def recover(self):
        self.raw=self.driver.recover();return self.raw
    def get_robot_state(self):
        from franka_cap.geometry import pose_dict
        s=self.driver.read();self.raw=s
        if s['has_errors'] or s['mode_name']!='idle':
            raise ApiError(ErrorCode.SAFETY_INTERLOCK,'robot_not_idle',details={'robot_state':s})
        p=pose_dict(s['T_base_ee'])
        # Encoder measurements can slightly exceed the nominal 80 mm stroke.
        # Keep raw metres in the device receipt; the public control coordinate
        # is saturated to its declared [0,1] range.
        opening=float(np.clip(s['gripper']['width_m']/.08,0.,1.))
        return RobotState(s['q'],s['dq'],Pose(p['position'],p['quaternion_wxyz'],'base'),opening,'base',
                          joint_names=[f'fr3_joint{i}' for i in range(1,8)],timestamp_s=s['timestamp_s'],embodiment='franka_fr3')
    def get_observation(self,camera_names=None):
        from franka_cap.devices import calibration,_check_intrinsics
        names=list(self.cameras) if camera_names is None else camera_names
        if not names or len(set(names))!=len(names):raise ValueError('invalid_camera_names')
        # Initialize every owner before bracketing a fresh observation. Camera
        # startup latency must not age the other camera's measurement frame.
        for name in names:self.cameras[name].start()
        for name in names:self.cameras[name].latest()
        before=self.get_robot_state();stamp=time.monotonic();frames={}
        for name in names: frames[name]=self.cameras[name].latest(after=stamp)
        after=self.get_robot_state()
        self.observation_diagnostics=dict(joint_delta=after.joint_positions['primary']-before.joint_positions['primary'],
            before_velocities=before.joint_velocities['primary'],after_velocities=after.joint_velocities['primary'])
        cameras={}
        for name,frame in frames.items():
            cal,transform,sha=calibration(self.sdk.cfg,name,self.raw);_check_intrinsics(frame.metadata,cal)
            if frame.metadata['serial']!=self.sdk.cfg[name+'_serial']:raise ValueError('camera_serial_mismatch')
            rgb,depth,k=rectify(frame)
            quat=Rotation.from_matrix(transform[:3,:3]).as_quat()
            cameras[name]=CameraObservation(rgb,depth,k,name+'_color_optical',Pose(transform[:3,3],quat[[3,0,1,2]],'base'),time.time())
            folder=self.sdk.run_dir/('raw-'+name+'-'+str(time.time_ns()));folder.mkdir()
            np.savez_compressed(folder/'frame.npz',rgb=frame.rgb,depth_m=frame.depth_m)
            (folder/'metadata.json').write_text(json.dumps(dict(frame.metadata,calibration_sha256=sha)))
        return Observation(cameras,after,timestamp_s=time.time())

    def solve_ik(self,target,seed=None):
        from franka_cap.planning import Kinematics
        try:
            if isinstance(target,dict):target=Pose(**target)
            if target.frame!='base':raise ValueError('base_frame_required')
            state=self.get_robot_state()
            kin=Kinematics()
            q=kin.solve(target.as_matrix(),state.joint_positions['primary'] if seed is None else seed)
            actual=kin.forward(q);desired=target.as_matrix()
            return IKResult(True,joint_positions=q,embodiment='franka_fr3',diagnostics=dict(
                position_residual_m=float(np.linalg.norm(actual[:3,3]-desired[:3,3])),
                rotation_residual_rad=float(Rotation.from_matrix(desired[:3,:3].T@actual[:3,:3]).magnitude()),
                joint_limit_clearance_rad=np.minimum(q-kin.lower,kin.upper-q).tolist(),
                posture_selection='seed_relative_joint_limit_barrier',
                acceptance_policy='pose_equality_solver_convergence; residuals_reported_without_project_threshold'))
        except ValueError as e:return IKResult(False,error=ApiError(ErrorCode.IK_FAILED,str(e)))
    def plan_motion(self,goal,*,speed_factor=None,start_joints=None,interpolation_dt_s=None,maximum_trajectory_dt_s=None):
        from .drivers import resolve_speed_factor
        speed_factor=resolve_speed_factor(speed_factor,self.driver.speed_factor,self.driver.max_speed_factor)
        from franka_cap.planning import plan_path, Kinematics
        if isinstance(goal,dict):goal=Pose(**goal)
        state=self.get_robot_state();raw=dict(self.raw)
        planning_state=dict(raw)
        # Position control starts at the controller's last commanded position;
        # torque control starts at the measured position. Do not reject normal
        # tracking offsets as a separate motion interlock.
        if getattr(self.driver,'backend',None)=='franky':planning_state['q']=raw['q_d']
        if start_joints is not None:planning_state['q']=np.asarray(start_joints,dtype=float).tolist()
        if isinstance(goal,Pose):
            if goal.frame!='base':raise ValueError('base_frame_pose_required')
            target={'position':goal.position.tolist(),'quaternion_wxyz':goal.quaternion_wxyz.tolist(),'frame':'base'}
            plan=plan_path(planning_state,target,self.sdk.cfg,None,scene_factory=None)
        else:
            from franka_cap.geometry import vector,pose_dict
            joints=vector(goal,7,'joint target');kin=Kinematics()
            if np.any(joints<kin.lower) or np.any(joints>kin.upper):raise ValueError('joint_target_outside_native_range')
            plan=dict(q_samples=[planning_state['q'],joints.tolist()],target_pose=pose_dict(kin.forward(joints)))
        if np.array_equal(plan['q_samples'][0],plan['q_samples'][-1]):
            from .curves import sample_interval
            timed=self.driver.prepare_samples([plan['q_samples'][0]],sample_interval(interpolation_dt_s,maximum_trajectory_dt_s))
        else:
            timing={k:v for k,v in dict(interpolation_dt_s=interpolation_dt_s,maximum_trajectory_dt_s=maximum_trajectory_dt_s).items() if v is not None}
            timed=self.driver.plan(plan['q_samples'],speed_factor=speed_factor,**timing)
        q=timed.pop('joint_positions')
        # Check the actual library curve, not the earlier path planner spline.
        kin=Kinematics();model_margin=0.
        if np.any(q<kin.lower+model_margin) or np.any(q>kin.upper-model_margin):
            raise ValueError('library_curve_joint_margin')
        plan.update(timed,interpolation=timed['backend'],planning_backend=getattr(self.driver,'backend','custom'))
        plan.update(collision_checked=False,collision_scope='not_checked',automatic_scene=False)
        trajectory=Trajectory(q,timed['dt_s'],state.joint_names['primary'],timed['backend'],False,state,embodiment='franka_fr3')
        return trajectory,plan

    def prepare_trajectory(self,trajectory):
        from franka_cap.planning import Kinematics
        from franka_cap.geometry import pose_dict
        result=self.driver.prepare_samples(trajectory.joint_positions,trajectory.dt_s)
        return dict(result,target_pose=pose_dict(Kinematics().forward(trajectory.joint_positions[-1])))
    def check_execution(self,plan):
        self.get_robot_state()
        return None
    def execute(self,plan,trajectory):
        raw_receipt=self.driver.execute(plan['token'])
        receipt=dict(raw_receipt,**raw_receipt.get('gripper',{}))
        final=np.asarray(receipt['q'],dtype=float)
        if final.shape!=(7,) or not np.isfinite(final).all():raise OutcomeUnknown('invalid_final_state')
        from ..execution_feedback import completed_motion,endpoint_accuracy
        try:accuracy=endpoint_accuracy(trajectory.joint_positions[-1],plan['target_pose'],receipt)
        except (ValueError,KeyError) as error:raise OutcomeUnknown('invalid_endpoint_measurement') from error
        error=accuracy['max_joint_error_rad']
        diagnostics=dict(native_receipt=receipt,endpoint_accuracy=accuracy,elapsed_s=receipt['elapsed_s'],
            steps_unit='public_reference_samples',motion_completed=completed_motion(receipt))
        if not diagnostics['motion_completed']:
            return ExecutionResult(False,0,error=ApiError(ErrorCode.CONTROLLER_FAILED,'controller_completion_not_confirmed'),
                final_errors={'primary':error},diagnostics=diagnostics)
        diagnostics.update(can_continue=True)
        if getattr(self,'physical_session',None):
            self.physical_session.arm_probe=True
        return ExecutionResult(True,len(trajectory.joint_positions),final_errors={'primary':error},diagnostics=diagnostics)
    def check_gripper(self,position):
        if not 0.<=position*.08<=.08:return 'width_outside_native_limits'
        self.get_robot_state()
        return None
    def check_close_gripper(self):
        self.get_robot_state()
        return None
    def close_gripper(self):
        receipt=self.driver.grasp(force=5.)
        ok=receipt['ok'] and receipt['is_grasped']
        if not ok:return ExecutionResult(False,1,error=ApiError(ErrorCode.GRASP_FAILED,'grasp_not_confirmed'),diagnostics={'native_receipt':receipt})
        return ExecutionResult(True,1,diagnostics={'native_receipt':receipt})
    def set_gripper(self,position):
        receipt=self.driver.move_gripper(position*.08)
        # libfranka's completed move is the operation result. Width error is
        # measured feedback, not a second, invented controller-fault threshold.
        ok=receipt['ok']
        diagnostics={'native_receipt':receipt,'target_width_mm':position*80.,'actual_width_mm':receipt['width_m']*1000.,'width_error_mm':(receipt['width_m']-position*.08)*1000.}
        if not ok:return ExecutionResult(False,1,error=ApiError(ErrorCode.CONTROLLER_FAILED,'gripper_target_not_reached'),diagnostics=diagnostics)
        return ExecutionResult(True,1,diagnostics=dict(diagnostics,steps_unit='completed_gripper_commands'))
    def close(self):
        for owner in self.cameras.values():owner.close()
        self.driver.close()
        self.sdk.close()
