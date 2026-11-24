"""Observed workspace geometry from full RGB-D, without task names or ROIs."""
import numpy as np
from scipy.spatial import cKDTree
from cap_harness.geometry import mask_to_point_cloud
from ..contracts import Segmentation, Pose
from ..manipulation import Box, Scene


def observed_scene(observation, observation_id, binding, *, camera_name='global', target=None,
                   lower=None, upper=None, voxel_m=.02, robot_bounds=(), robot_mask=None):
    camera=observation.cameras[camera_name]
    pixels=np.isfinite(camera.depth_m)&(camera.depth_m>0)
    if robot_mask is not None:
        robot_mask=np.asarray(robot_mask,dtype=bool)
        if robot_mask.shape!=pixels.shape:raise ValueError('robot_mask_depth_shape_mismatch')
        pixels&=~robot_mask
    mask=Segmentation(pixels,'observed workspace',1.,camera_name,camera.frame)
    points=mask_to_point_cloud(camera,mask,target_frame=observation.robot_state.base_frame).points
    if lower is not None:points=points[np.all(points>=lower,axis=1)]
    if upper is not None:points=points[np.all(points<=upper,axis=1)]
    for lo,hi in robot_bounds:
        points=points[~np.all((points>=np.asarray(lo))&(points<=np.asarray(hi)),axis=1)]
    if len(points)<3:raise ValueError('support_plane_requires_three_points')
    # RANSAC finds the dominant plane, using measured
    # depth only. The seed makes numerical checks repeatable, not scene placement.
    sample=points[np.linspace(0,len(points)-1,min(12000,len(points))).astype(int)]
    rng=np.random.default_rng(0);best=None;count=0
    for _ in range(160):
        tri=sample[rng.choice(len(sample),3,replace=False)]
        normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]);norm=np.linalg.norm(normal)
        if norm==0:continue
        normal/=norm
        if normal[2]<0:normal=-normal
        offset=float(tri[0]@normal);inliers=np.abs(sample@normal-offset)<.004
        if inliers.sum()>count:best=(normal,offset,inliers);count=int(inliers.sum())
    if best is None:raise ValueError('support_plane_not_observed')
    normal,offset,inliers=best
    support=sample[inliers].mean(axis=0)
    _,_,axes=np.linalg.svd(sample[inliers]-support,full_matrices=False)
    normal=axes[-1];normal=normal if normal[2]>0 else -normal
    height=(points-support)@normal
    raised=points[height>0]
    boxes=[]
    if target is not None:
        if target.frame!=observation.robot_state.base_frame:raise ValueError('target_frame_mismatch')
        lo,hi=target.points.min(axis=0),target.points.max(axis=0)
        # This box bounds visible samples only; hidden geometry is not reconstructed.
        boxes.append(Box('perceived_target',Pose((lo+hi)/2,[1,0,0,0],target.frame),tuple(hi-lo),True))
        raised=raised[cKDTree(target.points).query(raised)[0]>0]
    voxels,indices=np.unique(np.floor(raised/voxel_m).astype(int),axis=0,return_inverse=True)
    # Voxelization groups samples; it does not make the entire empty cell solid.
    # Bound measured points without added sensor padding.
    low=np.full((len(voxels),3),np.inf);high=np.full_like(low,-np.inf)
    np.minimum.at(low,indices,raised);np.maximum.at(high,indices,raised)
    for index,(lo,hi) in enumerate(zip(low,high)):
        boxes.append(Box(f'observed_{index}',Pose((lo+hi)/2,[1,0,0,0],'base'),tuple(hi-lo)))
    scene=Scene(tuple(boxes),tuple(support),tuple(normal),observation_id,binding,observation.timestamp_s,False)
    evidence={'source':'full_rgbd','manual_regions':False,'workspace_points':len(points),'support_inliers':count,
        'robot_exclusion':'sam3_robot_mask_and_calibrated_robot_geometry' if robot_mask is not None else 'calibrated_robot_geometry',
        'robot_mask_pixels':int(np.count_nonzero(robot_mask)) if robot_mask is not None else 0,
        'robot_mask_boundary_px':0,
        'valid_depth_boundary_px':0,
        'obstacle_voxels':len(voxels),'voxel_m':voxel_m,'obstacle_padding_m':0.,'hidden_geometry':'unobserved',
        'target_geometry':'visible-point envelope only','camera':camera_name}
    return scene,evidence
