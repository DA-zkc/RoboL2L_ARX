"""A transparent RGB-D baseline for the green-cube task, not a general detector.

Image regions are supplied per frozen observation. Hidden geometry is explicitly
estimated; the resulting scene is a preview and is never marked reviewed here.
"""
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from ..contracts import Pose
from ..manipulation import Box, Scene


def load_camera(folder, name):
    folder=Path(folder)
    metadata=json.loads((folder/'metadata.json').read_text())
    index=list(metadata['cameras']).index(name)
    with np.load(folder/f'camera-{index}.npz',allow_pickle=False) as data:
        rgb,depth,k=data['rgb'].copy(),data['depth_m'].copy(),data['intrinsics'].copy()
    transform=Pose(**metadata['cameras'][name]['camera_pose']).as_matrix()
    v,u=np.indices(depth.shape)
    points=np.stack(((u-k[0,2])*depth/k[0,0],(v-k[1,2])*depth/k[1,1],depth),axis=-1)
    points=points@transform[:3,:3].T+transform[:3,3]
    valid=np.isfinite(depth)&(depth>.05)&(depth<3)
    return metadata,rgb,points,valid


def region_mask(shape, region):
    x,y,w,h=region
    if any(type(v) is not int for v in region) or min(x,y)<0 or min(w,h)<5 or x+w>shape[1] or y+h>shape[0]:
        raise ValueError('invalid_image_region')
    mask=np.zeros(shape,np.uint8);mask[y:y+h,x:x+w]=1
    return mask.astype(bool)


def dominant(mask):
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    if count<2:raise ValueError('no_color_component')
    index=1+np.argmax(stats[1:,cv2.CC_STAT_AREA])
    if stats[index,cv2.CC_STAT_AREA]<100:raise ValueError('insufficient_color_component')
    return labels==index


def build_scene(folder, camera_name, regions, binding):
    meta,rgb,points,valid=load_camera(folder,camera_name)
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    support_mask=np.zeros(valid.shape,bool)
    for region in regions['table']:support_mask|=region_mask(valid.shape,region)
    sample=points[support_mask&valid][::5]
    if len(sample)<300:raise ValueError('insufficient_table_depth')
    # Robust plane z=ax+by+c; grooves and invalid depth are excluded iteratively.
    keep=np.ones(len(sample),bool)
    for _ in range(5):
        design=np.column_stack((sample[keep,:2],np.ones(keep.sum())))
        coef=np.linalg.lstsq(design,sample[keep,2],rcond=None)[0]
        residual=sample[:,2]-np.column_stack((sample[:,:2],np.ones(len(sample))))@coef
        keep=np.abs(residual)<.003
        if keep.sum()<300:raise ValueError('table_plane_unstable')
    normal=np.r_[-coef[:2],1.];normal/=np.linalg.norm(normal)
    if normal[2]<.98:raise ValueError('table_not_near_horizontal')
    support=np.array([0.,0.,coef[2]])
    cube_mask=dominant(region_mask(valid.shape,regions['cube'])&(cv2.inRange(hsv,(35,80,40),(90,255,255))>0))
    ys,xs=np.where(cube_mask)
    x,y,w,h=regions['cube']
    if xs.min()<=x or xs.max()>=x+w-1 or ys.min()<=y or ys.max()>=y+h-1:
        raise ValueError('cube_region_truncated')
    interior=cv2.erode(cube_mask.astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool)&valid
    visible=points[interior]
    if len(visible)<100:raise ValueError('insufficient_cube_depth')
    heights=(visible-support)@normal;height=float(np.percentile(heights,98))
    if not .02<height<.08:raise ValueError('cube_height_outside_baseline_range')
    top=visible[(heights>height-.005)&(heights<height+.003)]
    rectangle=cv2.minAreaRect(top[:,:2].astype(np.float32))
    center_xy=np.asarray(rectangle[0]);yaw=np.deg2rad(rectangle[2])
    # Cube assumption supplies hidden faces. Do not label it measured full geometry.
    side=max(height,*rectangle[1]);center_z=float(coef[:2]@center_xy+coef[2]+height/2)
    cube=Box('green_cube',Pose([*center_xy,center_z],Rotation.from_euler('z',yaw).as_quat()[[3,0,1,2]],'base'),(side,side,height),True)
    x,y,w,h=regions['plate'];ellipse=np.zeros(valid.shape,np.uint8)
    cv2.ellipse(ellipse,(x+w//2,y+h//2),(w//2-2,h//2-2),0,0,360,1,-1)
    plate_mask=dominant(ellipse.astype(bool)&(cv2.inRange(hsv,(0,0,100),(179,65,255))>0))
    plate_points=points[plate_mask&valid]
    if len(plate_points)<300:raise ValueError('insufficient_plate_depth')
    low,high=np.percentile(plate_points[:,:2],[2,98],axis=0)
    plate_xy=(low+high)/2
    radii=np.linalg.norm(plate_points[:,:2]-plate_xy,axis=1)
    radius=float(np.percentile(radii,97))
    if not .06<radius<.16:raise ValueError('plate_radius_outside_baseline_range')
    inner=plate_points[radii<radius*.45]
    floor=float(np.median(inner[:,2]));rim=float(np.percentile(plate_points[:,2],98))
    if not 0<rim-floor<.05:raise ValueError('plate_depth_inconsistent')
    boxes=[cube]
    # Bottom square sits inside the plate; rim boxes enclose the receiving cavity.
    boxes.append(Box('plate_bottom',Pose([*plate_xy,floor-.004],[1,0,0,0],'base'),(radius*1.4,radius*1.4,.008)))
    for index,angle in enumerate(np.linspace(0,2*np.pi,24,endpoint=False)):
        pos=[*(plate_xy+(radius-.005)*np.array([np.cos(angle),np.sin(angle)])),(floor+rim)/2]
        boxes.append(Box('plate_rim_'+str(index),Pose(pos,Rotation.from_euler('z',angle).as_quat()[[3,0,1,2]],'base'),(.014,radius*.28,rim-floor+.008)))
    for index,region in enumerate(regions.get('obstacles',[])):
        selected=points[region_mask(valid.shape,region)&valid]
        raised=selected[(selected-support)@normal>.01]
        if len(raised)<100:raise ValueError('obstacle_depth_missing')
        lo,hi=np.percentile(raised,[1,99],axis=0);lo[2]=float(coef[:2]@((lo[:2]+hi[:2])/2)+coef[2])
        boxes.append(Box('obstacle_'+str(index),Pose((lo+hi)/2,[1,0,0,0],'base'),tuple(hi-lo+.01)))
    scene=Scene(tuple(boxes),tuple(support),tuple(normal),Path(folder).name.removeprefix('observation-'),binding,meta['timestamp_s'])
    measurements=dict(camera=camera_name,regions=regions,cube_visible_points=len(visible),cube_height_m=height,
        cube_top_rectangle_m=rectangle[1],cube_hidden_faces='inferred using cube assumption',
        plate_center_m=[*plate_xy,floor],plate_outer_radius_m=radius,plate_rim_top_m=rim,
        table_residual_p95_m=float(np.percentile(np.abs(residual[keep]),95)),
        limitations=['manual image regions for this snapshot','hidden object faces inferred',
                     'plate represented by approximate bottom and rim boxes','unobserved geometry and cables are not reconstructed'])
    return scene,measurements,meta,rgb
