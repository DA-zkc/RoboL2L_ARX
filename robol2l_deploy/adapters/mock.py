"""Explicit synthetic station for software commissioning; never physical evidence."""
import time
import numpy as np
from ..contracts import Pose,RobotState,Observation,CameraObservation,Trajectory,IKResult,ExecutionResult
from ..cameras import CameraOwner,Frame
from ..manipulation import Manipulation
from ..runtime import fingerprint

class MockCamera:
    def open(self): pass
    def close(self): pass
    def read(self):
        time.sleep(.05)
        rgb=np.zeros((120,160,3),np.uint8);rgb[40:80,60:100,1]=200
        return Frame(rgb,np.full((120,160),.5),dict(fx=100,fy=100,ppx=80,ppy=60,model='distortion.none',coeffs=[0]*5),time.monotonic())

class MockAdapter:
    synthetic=True
    def __init__(self, dof=7, camera_names=('wrist','global')):
        self.dof=dof;self.embodiment='franka_fr3' if dof==7 else 'yam_real'
        self.q=np.zeros(dof);self.pose=Pose([0,0,.3],[1,0,0,0],"base");self.gripper=1.;self.generation=0;self.executions=0
        self.manipulation=Manipulation();self.is_grasped=False
        self.cameras={name:CameraOwner(MockCamera()) for name in camera_names}
    def describe(self): return dict(synthetic=True, physical_evidence=False, embodiment=self.embodiment, cameras=list(self.cameras), arms=['primary'])
    def device_binding(self): return 'mock-'+str(self.dof)
    def binding(self): return fingerprint(self.generation)
    def install_scene(self,scene):
        if scene.calibration_binding!=self.device_binding():raise ValueError('scene_device_binding_mismatch')
        self.manipulation=Manipulation(scene)
    def recover(self):
        # No controller to clear; say what this is instead of implying a device receipt.
        return dict(synthetic=True,physical_evidence=False,has_errors=False,mode_name='idle')
    def get_robot_state(self):
        return RobotState(self.q,np.zeros(self.dof),self.pose,self.gripper,'base',timestamp_s=time.time(),embodiment=self.embodiment)
    def get_observation(self,camera_names=None):
        result={}
        for name in self.cameras if camera_names is None else camera_names:
            f=self.cameras[name].latest()
            result[name]=CameraObservation(f.rgb,f.depth_m,[[100,0,80],[0,100,60],[0,0,1]],name,Pose([0,0,0],[1,0,0,0],'base'),time.time())
        return Observation(result,self.get_robot_state(),timestamp_s=time.time())
    def solve_ik(self, target, seed=None): return IKResult(True,joint_positions=self.q,embodiment=self.embodiment)
    def preview_xyz(self, trajectory):
        return (trajectory.joint_positions[:, :3]+[0,0,.3]).tolist()
    def plan_motion(self,goal,*,start_joints=None,speed_factor=None,interpolation_dt_s=None,maximum_trajectory_dt_s=None):
        if isinstance(goal,dict): goal=Pose(**goal)
        state=self.get_robot_state()
        if isinstance(goal,Pose):
            if goal.frame!='base':raise ValueError('base_frame_pose_required')
            end=self.q.copy();end[:3]=goal.position-[0,0,.3]
        else:
            end=np.asarray(goal,dtype=float)
            if end.shape!=(self.dof,) or not np.isfinite(end).all():raise ValueError('invalid_joint_target')
            goal=Pose(end[:3]+[0,0,.3],self.pose.quaternion_wxyz,'base')
        start=self.q if start_joints is None else np.asarray(start_joints,dtype=float)
        dt=min(v for v in (interpolation_dt_s,maximum_trajectory_dt_s,.1) if v is not None)
        trajectory=Trajectory(np.linspace(start,end,11),dt,state.joint_names['primary'],'mock',False,state,embodiment=self.embodiment)
        return trajectory, (end,goal)
    def prepare_trajectory(self,trajectory):
        end=trajectory.joint_positions[-1]
        return end,Pose(end[:3]+[0,0,.3],self.pose.quaternion_wxyz,'base')
    def execute(self,payload,trajectory):
        self.executions+=1;self.q=payload[0].copy();self.pose=payload[1]
        return ExecutionResult(True,len(trajectory.joint_positions),diagnostics={'synthetic':True})
    def check_execution(self,payload):return None
    def check_gripper(self,position):
        return None
    def set_gripper(self,position):
        self.gripper=position;self.executions+=1;self.is_grasped=False
        self.manipulation.release(self.pose.as_matrix())
        return ExecutionResult(True,1,diagnostics={'synthetic':True})
    def check_close_gripper(self):
        return None
    def close_gripper(self):
        self.executions+=1
        try:box,width=self.manipulation.grasp_candidate(self.pose.as_matrix())
        except ValueError:
            # A commanded empty close completes without a grasp; it is not a
            # pre-dispatch target-registration gate or a controller fault.
            from ..errors import ApiError,ErrorCode
            self.gripper=0.;self.is_grasped=False
            return ExecutionResult(False,1,error=ApiError(ErrorCode.GRASP_FAILED,'grasp_not_confirmed'),
                diagnostics={'synthetic':True,'native_receipt':dict(ok=False,is_grasped=False,width_m=0.)})
        self.gripper=width/.08;self.is_grasped=True
        self.manipulation.attach(box,self.pose.as_matrix(),width)
        return ExecutionResult(True,1,diagnostics={'synthetic':True,'attached_object':box.name})
    def close(self):
        for owner in self.cameras.values():owner.close()
