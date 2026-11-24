"""Offline 6D Cartesian-path IK and sampled FR3 collision diagnostics."""
from __future__ import annotations
import itertools
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares, minimize
from scipy.spatial.transform import Rotation, Slerp
from .geometry import matrix, pose_matrix, vector

MODEL = Path(__file__).with_name("models")


def contact_record(points, first, second):
    """Closest points in the collision model, never physical contact telemetry."""
    point = min(points, key=lambda p: p[8])
    return dict(links=[first, second], distance_m=float(point[8]),
                position_on_a_m=list(point[5]), position_on_b_m=list(point[6]))


class CollisionPrediction(ValueError):
    def __init__(self, hits, *, stage, margin=0., **context):
        super().__init__('planning_collision_predicted')
        self.details = dict(source='geometric_prediction', check_stage=stage,
            dispatch='not_sent', frame='base', physical_contact_measured=False,
            collision_margin_m=margin, collisions=hits, **context)


class Kinematics:
    def __init__(self):
        root = ET.parse(MODEL / "fr3_hand.urdf").getroot()
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain=[]; link="fr3_hand_tcp"
        while link in by_child:
            joint=by_child[link]; chain.append(joint); link=joint.find("parent").get("link")
        self.chain=[]; self.lower=np.zeros(7); self.upper=np.zeros(7)
        for joint in reversed(chain):
            origin=joint.find("origin"); transform=np.eye(4)
            if origin is not None:
                transform[:3,3]=np.fromstring(origin.get("xyz","0 0 0"),sep=" ")
                transform[:3,:3]=Rotation.from_euler("xyz",np.fromstring(origin.get("rpy","0 0 0"),sep=" ")).as_matrix()
            index=None;axis=None
            if joint.get("type") != "fixed":
                name=joint.get("name")
                if name not in [f"fr3_joint{i}" for i in range(1,8)]: raise ValueError("Unexpected joint in TCP chain")
                index=int(name[-1])-1
                axis=np.fromstring(joint.find("axis").get("xyz"),sep=" ")
                limit=joint.find("limit");self.lower[index]=float(limit.get("lower"));self.upper[index]=float(limit.get("upper"))
            self.chain.append((transform,index,axis))
        if sorted(i for _,i,_ in self.chain if i is not None)!=list(range(7)):
            raise ValueError("Expected exactly seven FR3 joints")

    def forward(self,q):
        q=vector(q,7,"joints"); result=np.eye(4)
        for origin,index,axis in self.chain:
            result=result@origin
            if index is not None:
                motion=np.eye(4);motion[:3,:3]=Rotation.from_rotvec(axis*q[index]).as_matrix();result=result@motion
        return result

    def solve(self,target,seed,margin=0.):
        target=matrix(target);seed=vector(seed,7,"seed")
        lower=self.lower+margin;upper=self.upper-margin
        if np.any(seed<lower) or np.any(seed>upper): raise ValueError("Start joints outside planning margin")
        span=upper-lower
        reference=(seed-lower)/span
        # A log-barrier metric is undefined exactly on a bound. Only in that
        # case use the centre as the preference for that joint; no bounds shrink.
        reference=np.where((reference>0)&(reference<1),reference,.5)

        def pose_error(u):
            actual=self.forward(lower+span*u)
            return np.r_[actual[:3,3]-target[:3,3],
                Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).as_rotvec()]

        def posture_cost(u):
            if np.any(u<=0) or np.any(u>=1):return np.inf
            delta=u-reference
            # Bregman distance of the joint-limit log barrier. Its minimum is
            # the seed, but approaching a physical bound costs infinitely more.
            # Pose is an equality constraint, never traded against this cost.
            return float(np.sum(-np.log1p(delta/reference)
                -np.log1p(-delta/(1-reference))
                +delta*(1/reference-1/(1-reference))))

        def posture_gradient(u):
            return -1/u+1/(1-u)+1/reference-1/(1-reference)

        def optimize(initial):
            return minimize(posture_cost,initial,jac=posture_gradient,method='SLSQP',
                bounds=[(0.,1.)]*7,constraints={'type':'eq','fun':pose_error},
                options={'ftol':1e-12,'maxiter':200})

        solved=optimize(reference)
        if not solved.success:
            # A pose-only solve provides another initial point, not a successful
            # approximate IK fallback. The same equality solve must still finish.
            initial=least_squares(pose_error,reference,bounds=(np.zeros(7),np.ones(7)))
            solved=optimize(initial.x)
        if not solved.success or not np.isfinite(solved.fun):
            raise ValueError('IK pose-constrained solver did not converge: '+solved.message)
        return lower+span*solved.x


class CollisionScene:
    def __init__(self,wrist_calibration,obstacles=()):
        import pybullet as pb
        self.pb=pb;self.client=pb.connect(pb.DIRECT)
        try:
            self.robot=pb.loadURDF(str(MODEL/"fr3_hand.urdf"),useFixedBase=True,flags=pb.URDF_USE_SELF_COLLISION,physicsClientId=self.client)
            self.links={-1:pb.getBodyInfo(self.robot,physicsClientId=self.client)[0].decode()};self.joints={}
            for i in range(pb.getNumJoints(self.robot,physicsClientId=self.client)):
                info=pb.getJointInfo(self.robot,i,physicsClientId=self.client);self.links[i]=info[12].decode();self.joints[info[1].decode()]=i
            self.arm=[self.joints[f"fr3_joint{i}"] for i in range(1,8)]
            self.fingers=[self.joints[f"fr3_finger_joint{i}"] for i in (1,2)]
            self.tcp=next(i for i,n in self.links.items() if n=="fr3_hand_tcp")
            self.shapes=[i for i in self.links if pb.getCollisionShapeData(self.robot,i,physicsClientId=self.client)]
            excluded={frozenset((e.get("link1"),e.get("link2"))) for e in ET.parse(MODEL/"fr3_hand.srdf").findall("disable_collisions")}
            self.pairs=[(a,b) for a,b in itertools.combinations(self.shapes,2) if frozenset((self.links[a],self.links[b])) not in excluded]
            self.X=matrix(wrist_calibration["T_EE_camera"])
            shape=pb.createCollisionShape(pb.GEOM_BOX,halfExtents=[.05,.0175,.017525],physicsClientId=self.client)
            self.camera=pb.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,physicsClientId=self.client)
            self.obstacles=[]
            for obj in obstacles:
                if set(obj)!={"name","center_m","half_extents_m"}:raise ValueError("Obstacle requires name, center_m and half_extents_m")
                center=vector(obj['center_m'],3);half=vector(obj['half_extents_m'],3)
                if np.any(half<=0):raise ValueError("Invalid obstacle size")
                shape=pb.createCollisionShape(pb.GEOM_BOX,halfExtents=half.tolist(),physicsClientId=self.client)
                body=pb.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,basePosition=center.tolist(),physicsClientId=self.client)
                self.obstacles.append((obj['name'],body))
        except BaseException:
            self.close();raise

    def close(self):
        self.pb.disconnect(self.client)

    def evidence(self):
        return dict(robot_geometry='FR3 URDF + wrist camera envelope',
                    obstacle_geometry='configured boxes')

    def set_state(self,q,width):
        pb=self.pb
        for i,v in zip(self.arm,q):pb.resetJointState(self.robot,i,float(v),physicsClientId=self.client)
        for i in self.fingers:pb.resetJointState(self.robot,i,width/2,physicsClientId=self.client)
        link=pb.getLinkState(self.robot,self.tcp,computeForwardKinematics=True,physicsClientId=self.client)
        ee=np.eye(4);ee[:3,:3]=Rotation.from_quat(link[5]).as_matrix();ee[:3,3]=link[4]
        camera=ee@self.X;center=camera[:3,3]+camera[:3,:3]@np.array([.0325,0,-.008225])
        pb.resetBasePositionAndOrientation(self.camera,center.tolist(),Rotation.from_matrix(camera[:3,:3]).as_quat().tolist(),physicsClientId=self.client)
        pb.performCollisionDetection(physicsClientId=self.client)

    def violations(self,margin=0.):
        pb=self.pb;hits=[]
        def query(a,b,ia,ib,na,nb,distance=margin):
            values=pb.getClosestPoints(a,b,distance=distance,linkIndexA=ia,linkIndexB=ib,physicsClientId=self.client)
            if values:hits.append(contact_record(values, na, nb))
        for a,b in self.pairs:query(self.robot,self.robot,a,b,self.links[a],self.links[b])
        for a in self.shapes:
            # Rigidly mounted hand-camera pair: reject overlap of the padded camera envelope.
            query(self.robot,self.camera,a,-1,self.links[a],'wrist_camera',0 if self.links[a]=='fr3_hand' else margin)
            for name,body in self.obstacles:query(self.robot,body,a,-1,self.links[a],name)
        for name,body in self.obstacles:query(self.camera,body,-1,-1,'wrist_camera',name)
        return hits


def plan_path(state,target_pose,cfg,wrist_calibration,*,scene_factory=CollisionScene):
    options=cfg['planning'];kin=Kinematics();seed=vector(state['q'],7)
    if state['has_errors']:raise ValueError('Robot reports errors')
    start=kin.forward(seed);measured=matrix(state['T_base_ee']);target=pose_matrix(target_pose)
    position_error=float(np.linalg.norm(start[:3,3]-measured[:3,3]))
    rotation_error=float(np.linalg.norm(Rotation.from_matrix(start[:3,:3].T@measured[:3,:3]).as_rotvec()))
    # Follow the requested absolute target; do not silently shift it to an FK-relative goal.
    distance=float(np.linalg.norm(target[:3,3]-start[:3,3]))
    angle=float(np.linalg.norm(Rotation.from_matrix(start[:3,:3].T@target[:3,:3]).as_rotvec()))
    margin=0.
    if np.any(seed<kin.lower+margin) or np.any(seed>kin.upper-margin):raise ValueError('Start joint margin violated')
    duration=float(options['duration_s'])
    if not np.isfinite(duration) or duration<=0:raise ValueError('invalid_trajectory_duration')
    rotations=Slerp([0,1],Rotation.from_matrix([start[:3,:3],target[:3,:3]]))
    # Longer requests increase sampling density instead of being rejected.
    count=max(101,int(np.ceil(distance/.002))+1,int(np.ceil(angle/.01))+1)
    knots=np.linspace(0,1,count);qs=[seed]
    for fraction in knots[1:]:
        goal=np.eye(4);goal[:3,:3]=rotations(fraction).as_matrix();goal[:3,3]=start[:3,3]+fraction*(target[:3,3]-start[:3,3])
        qs.append(kin.solve(goal,qs[-1],margin))
    spline=CubicSpline(knots,np.array(qs),bc_type='natural')
    t=np.linspace(0,duration,int(duration/.002)+1);u=t/duration
    progress=u**4*(35+u*(-84+u*(70-20*u)))
    sampled=spline(progress);velocity=np.gradient(sampled,t,axis=0);acceleration=np.gradient(velocity,t,axis=0);jerk=np.gradient(acceleration,t,axis=0)
    dynamics={key:float(np.max(np.abs(value))) for key,value in [('velocity_rad_s',velocity),('acceleration_rad_s2',acceleration),('jerk_rad_s3',jerk)]}
    # The installed controller time-parameterizes this path using its joint
    # velocity/acceleration/jerk limits. This offline curve is diagnostic only.
    if np.any(sampled<kin.lower+margin-1e-8) or np.any(sampled>kin.upper-margin+1e-8):raise ValueError('Spline violates joint margin')
    scene=scene_factory(wrist_calibration,options['obstacles']) if scene_factory is not None else None
    clearance=0.
    path_position_error=0.;path_rotation_error=0.
    try:
        for f in np.linspace(0,1,2*count-1):
            q=spline(f)
            if scene is not None:
                scene.set_state(q,state['gripper']['width_m']);hits=scene.violations(clearance)
                if hits:raise CollisionPrediction(hits, stage='cartesian_path', path_fraction=float(f),
                    joint_positions=q.tolist(), **scene.evidence())
            actual=kin.forward(q);expected=start[:3,3]+f*(target[:3,3]-start[:3,3])
            path_position_error=max(path_position_error,float(np.linalg.norm(actual[:3,3]-expected)))
            path_rotation_error=max(path_rotation_error,float(np.linalg.norm(Rotation.from_matrix(rotations(f).as_matrix().T@actual[:3,:3]).as_rotvec())))
    finally:
        if scene is not None:scene.close()
    return {'status':'offline_path_checked' if scene is not None else 'kinematic_path_planned','executed':False,'motion_executable':False,'target_pose':target_pose,
            'duration_s':duration,'q_samples':np.array(qs).tolist(),'interpolation':'natural_cubic_with_seventh_order_time_scaling',
            'start_robot_state':state,'fk_position_residual_m':position_error,'fk_rotation_residual_rad':rotation_error,
            'path_position_residual_m':path_position_error,'path_rotation_residual_rad':path_rotation_error,
            'path_length_m':distance,'rotation_rad':angle,'sampled_collision_checks':2*count-1 if scene is not None else 0,'dynamics':dynamics,
            'collision_checked':scene is not None,
            'environment_geometry_reviewed':options['environment_geometry_reviewed'] if scene is not None else False,
            'collision_scope':'FR3, hand, wrist-camera body and configured boxes' if scene is not None else 'not_checked',
            'limitations':['Planning output is not physical execution evidence.']}
