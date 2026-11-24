"""Project-local guarded actuation. No task choreography or implicit recovery."""
import copy
import json
import signal
import subprocess
import time
from pathlib import Path
from uuid import uuid4
import cv2
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation
from . import devices, planning
from .common import ROOT, read_json, write_json, digest, hardware_lock, scalar
from .geometry import matrix, pose_matrix, vector


class MotionFault(RuntimeError):
    pass


class MotionScene(planning.CollisionScene):
    def __init__(self, calibration, profile):
        super().__init__(calibration)
        self.profile=profile;pb=self.pb
        normal=vector(profile['support_normal'],3);point=vector(profile['support_point_m'],3)+normal*.003
        shape=pb.createCollisionShape(pb.GEOM_PLANE,planeNormal=normal,physicsClientId=self.client)
        self.table=pb.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,basePosition=point,physicsClientId=self.client)
        # The plate is represented by its bottom and rim, leaving its receiving cavity free.
        self.plate=[];center=vector(profile['plate_center_m'],3)
        shape=pb.createCollisionShape(pb.GEOM_CYLINDER,radius=.09,height=.006,physicsClientId=self.client)
        self.plate.append(pb.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,basePosition=[center[0],center[1],center[2]-.003],physicsClientId=self.client))
        for angle in np.linspace(0,2*np.pi,24,endpoint=False):
            shape=pb.createCollisionShape(pb.GEOM_BOX,halfExtents=[.008,.012,.012],physicsClientId=self.client)
            pos=center+np.array([.082*np.cos(angle),.082*np.sin(angle),.008])
            body=pb.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,basePosition=pos,
                                   baseOrientation=Rotation.from_euler('z',angle).as_quat(),physicsClientId=self.client)
            self.plate.append(body)
        self.object_body=None;self.object_pose=None;self.attachment=None

    def set_object(self, target, attachment=None):
        if self.object_body is not None:self.pb.removeBody(self.object_body,physicsClientId=self.client)
        shape=self.pb.createCollisionShape(self.pb.GEOM_BOX,halfExtents=np.array(target['size_m'])/2,
                                          physicsClientId=self.client)
        self.object_body=self.pb.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,physicsClientId=self.client)
        self.object_pose=np.eye(4);self.object_pose[:3,3]=target['center_m'];self.attachment=attachment

    def check(self,q,width,contact=False,holding=False):
        self.set_state(q,width);hits=self.violations(.01);pb=self.pb
        for link in self.shapes:
            if self.links[link]!='fr3_link0':
                pts=pb.getClosestPoints(self.robot,self.table,.01,linkIndexA=link,physicsClientId=self.client)
                if pts:hits.append({'links':[self.links[link],'table'],'distance_m':min(p[8] for p in pts)})
            for body in self.plate:
                pts=pb.getClosestPoints(self.robot,body,.008,linkIndexA=link,physicsClientId=self.client)
                if pts:hits.append({'links':[self.links[link],'plate'],'distance_m':min(p[8] for p in pts)})
        for body in [self.table,*self.plate]:
            if pb.getClosestPoints(self.camera,body,.01,physicsClientId=self.client):hits.append({'links':['camera','support']})
        if self.object_body is not None:
            pose=planning.Kinematics().forward(q)@self.attachment if holding else self.object_pose
            pb.resetBasePositionAndOrientation(self.object_body,pose[:3,3],Rotation.from_matrix(pose[:3,:3]).as_quat(),physicsClientId=self.client)
            for link in self.shapes:
                pts=pb.getClosestPoints(self.robot,self.object_body,.002,linkIndexA=link,physicsClientId=self.client)
                if pts and not (contact and link in self.fingers):hits.append({'links':[self.links[link],'object']})
            if pb.getClosestPoints(self.camera,self.object_body,.01,physicsClientId=self.client):hits.append({'links':['camera','object']})
            if holding:
                # Object may start at its original support, but must not penetrate support geometry.
                for body in [self.table,*self.plate]:
                    pts=pb.getClosestPoints(self.object_body,body,0,physicsClientId=self.client)
                    if pts and min(p[8] for p in pts)<-.004:hits.append({'links':['held_object','support']})
        if hits:raise ValueError('Scene collision/clearance: '+json.dumps(hits[:3]))


class MotionBackend:
    def __init__(self,sdk,profile_path):
        self.sdk=sdk;self.path=Path(profile_path);self.profile=read_json(self.path)
        p=self.profile
        if p.get('authorized_task')!='green-cube-to-plate' or p.get('user_authorized') is not True:
            raise ValueError('Motion authorization missing')
        self.failed=False;self.should_stop=lambda:False;self.calls=0;self.target=None;self.attachment=None
        self.initial_probe_passed=False;self.gripper_probe_passed=False;self.grasp_seen=False;self.lift_seen=False;self.release_seen=False
        self.latest_release=None
        for name in ('wrist','global'):
            if digest(sdk.cfg[name+'_calibration'])!=p[name+'_calibration_sha256']:
                raise ValueError('Motion calibration mismatch')

    def begin_program(self,should_stop):self.calls=0;self.should_stop=should_stop

    def guard(self):
        if self.failed:raise MotionFault('Motion latched after an execution fault; operator review required')
        if self.should_stop():raise MotionFault('Operator requested stop')
        if self.calls>=1:raise ValueError('One physical command per program; return feedback and reobserve before the next')

    def state(self):
        state=devices.robot_state(self.sdk.cfg)
        if state['mode_name']!='idle' or state['has_errors'] or np.max(np.abs(state['dq']))>.005:
            self.failed=True
            raise MotionFault('Robot must be idle, stationary and error-free; motion stopped')
        if any(state.get('cartesian_contact',[])):
            self.failed=True
            raise MotionFault('Robot reports contact; motion stopped')
        return state

    def invoke(self,name,args,timeout):
        self.calls+=1
        p=subprocess.Popen([str(ROOT/'native/build'/name),*map(str,args)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        deadline=time.monotonic()+timeout
        try:
            while p.poll() is None:
                if time.monotonic()>deadline:
                    p.send_signal(signal.SIGINT)
                    try:p.communicate(timeout=4)
                    except subprocess.TimeoutExpired:p.kill();p.communicate()
                    raise MotionFault('Actuation stopped: cancellation or timeout')
                time.sleep(.02)
            out,err=p.communicate()
            if p.returncode:raise MotionFault(err.strip() or 'Native motion reported failure: '+out[:500])
            return json.loads(out)
        except BaseException:
            self.failed=True
            if p.poll() is None:p.send_signal(signal.SIGINT)
            raise

    def move(self,target_pose):
        self.guard();target=pose_matrix(target_pose)
        lo=vector(self.profile['workspace_min_m'],3);hi=vector(self.profile['workspace_max_m'],3)
        if np.any(target[:3,3]<lo) or np.any(target[:3,3]>hi):raise ValueError('Target outside commissioned workspace')
        if target[2,2]>-.85:raise ValueError('Tool approach exceeds commissioned downward orientation cone')
        with hardware_lock():
            state=self.state();holding=state['gripper']['is_grasped']
            if holding!=(self.attachment is not None):raise ValueError('Untracked held object; no arm motion')
            if not self.initial_probe_passed and np.linalg.norm(target[:3,3]-np.array(state['T_base_ee'])[:3,3])>.00501:
                raise ValueError('First physical arm command must be a <=5 mm commissioning move, above 0.20 m')
            if not self.initial_probe_passed and np.linalg.norm(Rotation.from_matrix(np.array(state['T_base_ee'])[:3,:3].T@target[:3,:3]).as_rotvec())>.01:
                raise ValueError('Initial commissioning move must retain tool orientation')
            if not self.initial_probe_passed and target[2,3]<.2:raise ValueError('Commissioning move must remain above 0.20 m')
            # Plan from the desired stationary joints: the native command starts without a q/q_d jump.
            reference=copy.deepcopy(state);reference['q']=state['q_d']
            cal,_,_=devices.calibration(self.sdk.cfg,'wrist',state)
            plan=planning.plan_path(reference,target_pose,self.sdk.cfg,cal)
            scene=MotionScene(cal,self.profile)
            try:
                if self.target:scene.set_object(self.target,self.attachment)
                curve=CubicSpline(np.linspace(0,1,101),np.array(plan['q_samples']),bc_type='natural')
                for f in np.linspace(0,1,201):scene.check(curve(f),state['gripper']['width_m'],holding,holding)
            finally:scene.close()
            current=self.state()
            if np.max(np.abs(np.array(current['q'])-state['q']))>.001:raise ValueError('State changed while checking path')
            folder=self.sdk.run_dir/('motion-'+uuid4().hex[:12]);folder.mkdir()
            trajectory=folder/'trajectory.txt';start=planning.Kinematics().forward(reference['q']);delta=target[:3,3]-start[:3,3]
            values=[*start.flatten(order='F'),*delta,plan['duration_s'],plan['q_samples'][-1][2],state['gripper']['width_m'],101,*np.array(plan['q_samples']).flatten()]
            trajectory.write_text('FRANKA_CAP_JOINT_V1\n'+' '.join(format(float(x),'.17g') for x in values)+'\n')
            devices.native('franka_cap_motion','--validate',trajectory,timeout=5)
            write_json(folder/'plan.json',plan);write_json(folder/'result.json',{'status':'dispatching','motion_sent':True})
            try:
                result=self.invoke('franka_cap_motion',[self.sdk.cfg['robot_ip'],trajectory,'holding' if holding else 'empty'],plan['duration_s']+8)
                after=self.state();actual=np.array(after['T_base_ee']);position_error=float(np.linalg.norm(actual[:3,3]-target[:3,3]))
                rotation_error=float(np.linalg.norm(Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).as_rotvec()))
                if position_error>.002 or rotation_error>.005:raise MotionFault('Motion endpoint mismatch')
                if holding and (not after['gripper']['is_grasped'] or abs(after['gripper']['width_m']-state['gripper']['width_m'])>.002):raise MotionFault('Grasp state changed during arm motion')
                self.initial_probe_passed=True
                if holding and actual[2,3]-self.grasp_ee_z>.025:self.lift_seen=True
                result={'ok':True,'motion_sent':True,'state':after,'position_error_m':position_error,'rotation_error_rad':rotation_error,'directory':str(folder),'native':result}
                write_json(folder/'result.json',result)
                return result
            except BaseException as error:
                self.failed=True;write_json(folder/'result.json',{'status':'fault','motion_sent':True,'error':str(error)[:2000]});raise MotionFault(str(error)) from None

    def measure(self,camera,bbox,color=None):
        x,y,w,h=map(int,vector(bbox,4,'bbox_xywh'))
        if w<5 or h<5 or x<0 or y<0 or x+w>camera['width'] or y+h>camera['height']:raise ValueError('Invalid image region')
        rgb=cv2.imread(camera['rgb_path']);depth=np.load(camera['depth_path'],allow_pickle=False)
        mask=np.zeros(depth.shape,np.uint8);mask[y:y+h,x:x+w]=1
        if color is not None:
            if color!='green':raise ValueError('This task supports green object measurement')
            hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV);mask &= cv2.inRange(hsv,(35,90,50),(85,255,255))>0
        mask=cv2.erode(mask,np.ones((3,3),np.uint8));ys,xs=np.where((mask>0)&(depth>0))
        if len(xs)<100:raise ValueError('Too few valid region depth points')
        k=np.array([[camera['fx'],0,camera['ppx']],[0,camera['fy'],camera['ppy']],[0,0,1.]])
        rays=cv2.undistortPoints(np.stack([xs,ys],axis=-1)[:,None,:].astype(float),k,np.array(camera['coeffs']))[:,0,:]
        z=depth[ys,xs];T=matrix(camera['T_base_camera']);points=np.column_stack([rays*z[:,None],z])@T[:3,:3].T+T[:3,3]
        low,mid,high=np.percentile(points,[2,50,98],axis=0)
        top=points[points[:,2]>=np.percentile(points[:,2],70)]
        rectangle=cv2.minAreaRect(top[:,:2].astype(np.float32))
        result={'top_visible_rectangle_xy':{'center_m':list(rectangle[0]),'extent_m':list(rectangle[1]),'angle_deg':float(rectangle[2])},'bbox_xywh':[x,y,w,h],'point_count':len(points),'base_min_m':low.tolist(),'base_median_m':mid.tolist(),'base_max_m':high.tolist(),'visible_extent_m':(high-low).tolist(),
                'truncated':x<=1 or y<=1 or x+w>=camera['width']-1 or y+h>=camera['height']-1,'frame':'base',
                'limitation':'Visible surfaces only; hidden dimensions and object identity are not certified.'}
        if color=='green' and not result['truncated'] and self.attachment is None:
            bottom=float(np.dot(np.array(self.profile['support_point_m']),np.array(self.profile['support_normal'])))
            height=high[2]-bottom
            if not .02<height<.08:raise ValueError('Object height outside task envelope')
            # Conservative world-aligned payload envelope; jaw-width measurement refines it after grasp.
            size=np.maximum(high-low,[.05,.05,height]);size[:2]=np.minimum(size[:2]+.006,.075);size[2]=height
            center=(low+high)/2;center[2]=bottom+height/2
            object_id='object-'+uuid4().hex[:10]
            self.target={'id':object_id,'center_m':center.tolist(),'size_m':size.tolist(),'observed_median_m':mid.tolist(),'created_s':time.time()}
            result['object_id']=object_id;result['payload_envelope']=self.target
        return result

    def gripper(self,position=None,width=None,force=5,object_id=None):
        self.guard()
        with hardware_lock():
            state=self.state();current=state['gripper']['width_m'];holding=state['gripper']['is_grasped']
            if width is None:
                target=scalar(position,'position',0,1)*.08;action='move';force=0
                if target<current-.0051:raise ValueError('Use grasp(width_m,force_n,object_id) for closing around an object')
                if not self.gripper_probe_passed:
                    if holding or np.array(state['T_base_ee'])[2,3]<.2 or abs(target-current)>.0051:raise ValueError('First gripper command is an empty <=5 mm opening/closing probe above 0.20 m')
                if holding:
                    ee=np.array(state['T_base_ee']);object_pose=ee@self.attachment
                    plate=np.array(self.profile['plate_center_m'])
                    if np.linalg.norm(object_pose[:2,3]-plate[:2])>.025 or not plate[2]+.01<object_pose[2,3]<plate[2]+.065:
                        raise ValueError('Release is only allowed over the measured plate interior at low height')
            else:
                action='grasp';target=scalar(width,'width_m',.02,.07);force=scalar(force,'force_n',3,10)
                if not self.initial_probe_passed or not self.gripper_probe_passed:raise ValueError('Complete small arm and empty gripper commissioning first')
                if holding or self.target is None or object_id!=self.target['id']:raise ValueError('A measured, non-truncated green object is required before grasp')
                ee=np.array(state['T_base_ee']);center=np.array(self.target['center_m']);size=np.array(self.target['size_m'])
                if np.linalg.norm(ee[:2,3]-center[:2])>.018 or not center[2]<ee[2,3]<center[2]+size[2]/2+.005:
                    raise ValueError('EE is not at the measured object gripping region')
                cal,_,_=devices.calibration(self.sdk.cfg,'wrist',state);scene=MotionScene(cal,self.profile)
                try:
                    scene.set_object(self.target)
                    for jaw in np.linspace(current,target,21):scene.check(state['q_d'],float(jaw),contact=True)
                finally:scene.close()
            folder=self.sdk.run_dir/('gripper-'+uuid4().hex[:12]);folder.mkdir();write_json(folder/'command.json',{'action':action,'width_m':target,'force_n':force,'before':state,'status':'dispatching'})
            try:
                result=self.invoke('franka_cap_gripper',[self.sdk.cfg['robot_ip'],action,target,force],18)
                after=self.state()
                if action=='grasp':
                    if not after['gripper']['is_grasped'] or abs(after['gripper']['width_m']-target)>.005:raise MotionFault('Grasp not confirmed; lift forbidden')
                    self.grasp_seen=True;self.grasp_ee_z=np.array(after['T_base_ee'])[2,3]
                    pose=np.eye(4);pose[:3,3]=self.target['center_m'];self.attachment=np.linalg.inv(np.array(after['T_base_ee']))@pose
                else:
                    self.gripper_probe_passed=True
                    if holding:
                        if after['gripper']['is_grasped']:raise MotionFault('Gripper did not release')
                        self.release_seen=True;self.latest_release={'timestamp_s':time.time(),'object_pose':object_pose.tolist()};self.attachment=None;self.target=None
                result.update(motion_sent=True,state=after,directory=str(folder));write_json(folder/'result.json',result);return result
            except BaseException as error:
                self.failed=True;write_json(folder/'result.json',{'status':'fault','error':str(error),'motion_sent':True});raise MotionFault(str(error)) from None
