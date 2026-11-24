"""Prepare a reviewable candidate from saved RGB-D/state; never connects to hardware."""
import argparse
import json
import os
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from franka_cap.common import config
from robol2l_deploy.adapters.fr3 import FR3Adapter
from robol2l_deploy.contracts import Pose
from robol2l_deploy.preview import preview, pose_of
from robol2l_deploy.runtime import public
from robol2l_deploy.tasks.green_cube import build_scene


def candidate_operations(scene,measurements,current):
    cube=scene.boxes[0];cube_yaw=Rotation.from_matrix(cube.pose.as_matrix()[:3,:3]).as_euler('xyz')[2]
    down=[Rotation.from_euler('z',cube_yaw+a)*Rotation.from_euler('x',np.pi) for a in np.arange(4)*np.pi/2]
    rotation=min(down,key=lambda r:Rotation.from_matrix(current.as_matrix()[:3,:3].T@r.as_matrix()).magnitude())
    quat=rotation.as_quat()[[3,0,1,2]]
    def pose(xyz):return Pose(xyz,quat,'base')
    center=cube.pose.position;plate=np.array(measurements['plate_center_m']);hover=max(.18,center[2]+.12)
    grasp_offset=min(.012,cube.size_m[2]/4)
    # Carry the observed object-to-TCP offset through the placement calculation.
    release=plate+np.array([0,0,cube.size_m[2]/2+.004+grasp_offset])
    # This is an inspectable candidate, not Runtime task logic or a claimed model output.
    operations=[dict(name='approach_above_cube',kind='motion',pose=Pose([*center[:2],hover],current.quaternion_wxyz,'base')),
                dict(name='align_above_cube',kind='motion',pose=pose([*center[:2],hover])),
                dict(name='descend_to_grasp',kind='motion',pose=pose(center+np.array([0,0,grasp_offset]))),
                dict(name='contact_grasp',kind='grasp'),
                dict(name='lift',kind='motion',pose=pose([*center[:2],hover])),
                dict(name='transfer_above_plate',kind='motion',pose=pose([*plate[:2],hover])),
                dict(name='lower_inside_plate',kind='motion',pose=pose(release)),
                dict(name='release',kind='release'),
                dict(name='retract',kind='motion',pose=pose([*plate[:2],hover]))]
    return operations


def render_preview(rgb,scene,operations,current,report,paths,directory,regions):
    os.environ.setdefault('MPLCONFIGDIR',str(directory.resolve()/'plot-cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig=plt.figure(figsize=(13,6),layout='constrained');image=fig.add_subplot(121);image.imshow(rgb)
    for name,color in [('cube','lime'),('plate','cyan')]:
        x,y,w,h=regions[name]
        image.add_patch(plt.Rectangle((x,y),w,h,fill=False,color=color,lw=2));image.text(x,y-8,name,color=color)
    image.set_title('Frozen physical RGB — no actuation');image.axis('off')
    axis=fig.add_subplot(122,projection='3d')
    for box in scene.boxes:
        corners=box.corners();faces=[[corners[i] for i in face] for face in ((0,1,3,2),(4,5,7,6),(0,1,5,4),(2,3,7,6),(0,2,6,4),(1,3,7,5))]
        color='green' if box.graspable else ('orange' if box.name.startswith('obstacle') else 'gray')
        axis.add_collection3d(Poly3DCollection(faces,facecolors=color,alpha=.25,linewidths=.3,edgecolors=color))
    proposed=np.array([current.position]+[op['pose'].position for op in operations if op['kind']=='motion'])
    axis.plot(*proposed.T,'--',color='gray',label='candidate waypoints')
    for index,path in enumerate(paths):axis.plot(*path.T,color='blue',label='checked FK path' if index==0 else None)
    axis.scatter(*current.position,color='red',label='measured TCP')
    axis.set(xlabel='base x (m)',ylabel='base y (m)',zlabel='base z (m)',xlim=(.3,.85),ylim=(-.55,.25),zlim=(-.02,.35),title='Offline path preview: '+report['status'])
    axis.legend(loc='upper left',fontsize=8);axis.view_init(elev=35,azim=135)
    fig.savefig(directory/'preview.png',dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observation',type=Path,required=True)
    parser.add_argument('--raw-state',type=Path,required=True)
    parser.add_argument('--regions',type=Path,required=True)
    parser.add_argument('--station',type=Path,default=Path('config/station.json'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    cfg=config(args.station);raw=json.loads(args.raw_state.read_text())
    adapter=FR3Adapter(SimpleNamespace(cfg=cfg),cameras={})  # binding only; no device methods
    scene,measurements,meta,rgb=build_scene(args.observation,'global',json.loads(args.regions.read_text()),adapter.device_binding())
    if abs(raw['timestamp_s']-meta['robot_state']['timestamp_s'])>.001 or not np.allclose(raw['q'],meta['robot_state']['joint_positions']['primary'],rtol=0,atol=1e-7):
        raise ValueError('raw_state_does_not_match_frozen_observation')
    (args.output/'scene.json').write_text(json.dumps(public(scene),indent=2))
    (args.output/'measurements.json').write_text(json.dumps(public(measurements),indent=2))
    (args.output/'source.json').write_text(json.dumps(dict(observation=str(args.observation.resolve()),raw_state=raw),indent=2))
    current=pose_of(raw['T_base_ee'])
    operations=candidate_operations(scene,measurements,current)
    (args.output/'candidate.json').write_text(json.dumps(public(dict(author='software-generated review candidate; not agent output',operations=operations)),indent=2))
    report,paths=preview(raw,scene,operations,cfg,json.loads(Path(cfg['wrist_calibration']).read_text()),args.output/'paths')
    render_preview(rgb,scene,operations,current,report,paths,args.output,json.loads(args.regions.read_text()))
    blocked=next((r for r in report['operations'] if r['status']=='blocked'),None)
    if blocked:blocked={key:blocked[key] for key in ('operation','status','error')}
    print(json.dumps(dict(output=str(args.output.resolve()),status=report['status'],blocked=blocked),ensure_ascii=False))


if __name__=='__main__':main()
