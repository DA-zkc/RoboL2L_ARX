"""Compare real SAM3 text localization on one archived three-camera X5 observation."""
import argparse
from dataclasses import replace
from itertools import combinations
import json
from pathlib import Path

import cv2
import numpy as np
import requests

from cap_harness.contracts import Segmentation
from cap_harness.geometry import mask_to_point_cloud
from robol2l_deploy.adapters.arx_dual import DualARXStation
from robol2l_deploy.arx_calibration import detect, make_board
from robol2l_deploy.cap.physical_task import DualJointTask
from robol2l_deploy.cap.program import ProgramExecutor
from robol2l_deploy.cap.storage import write_json
from robol2l_deploy.observations import load_observation
from robol2l_deploy.providers.perception import configuration
from robol2l_deploy.runtime import StationRuntime


def read_archive(station, directory):
    metadata = json.loads((directory/'metadata.json').read_text())
    obs = load_observation(directory)
    roles = ('global','left_wrist','right_wrist')
    if set(obs.cameras) != set(roles):
        raise ValueError('a three-camera RGB-D archive is required')
    for role in roles:
        if metadata['measurement_diagnostics'][role]['serial'] != station.cameras[role].source.serial:
            raise ValueError(f'{role}: archive serial does not match calibration')
    if metadata['device_binding'] == station.device_binding():
        if obs.robot_state.base_frame != 'left_base' or any(c.camera_pose.frame != 'left_base' for c in obs.cameras.values()):
            raise ValueError('inconsistent calibrated observation')
        return obs
    if metadata['device_binding'] != station.calibration_binding() or obs.robot_state.base_frame != 'arm_local':
        raise ValueError('archive does not match the installed X5 station')
    local = {}
    for side, q in obs.robot_state.joint_positions.items():
        local[side] = station.arms[side]._pose(q).as_matrix()
        np.testing.assert_allclose(local[side],obs.robot_state.end_effector_poses[side].as_matrix(),atol=1e-7)
    state = replace(obs.robot_state,base_frame='left_base',end_effector_poses={
        side:station.shared_flange(side,pose) for side,pose in local.items()})
    return replace(obs,robot_state=state,cameras={name:replace(cam,camera_pose=station.camera_pose(
        name,local.get(name.removesuffix('_wrist')))) for name,cam in obs.cameras.items()})


def matching_board_corners(board, camera, mask):
    """Independent check using only identified corners inside the SAM3 selection."""
    try:
        detection = detect(board, camera.rgb, camera.intrinsics)
    except ValueError as error:
        return {}, str(error)
    points = {}
    pixel_mask = np.zeros_like(mask, dtype=bool)
    for tag, (x, y) in zip(detection['ids'], np.rint(detection['pixels']).astype(int)):
        if not (0 <= y < mask.shape[0] and 0 <= x < mask.shape[1] and mask[y, x]):
            continue
        if not (np.isfinite(camera.depth_m[y, x]) and camera.depth_m[y, x] > 0):
            continue
        pixel_mask[y, x] = True
        corner = Segmentation(pixel_mask, str(tag), 1., camera.frame, camera.frame)
        cloud = mask_to_point_cloud(camera, corner, target_frame='left_base')
        points[tag] = cloud.points[0]
        pixel_mask[y, x] = False
    return points, None


def run(directory, output, prompt, config, check_board=False):
    service_url = configuration()['sam3_url']
    with requests.Session() as client:
        client.trust_env = False
        response = client.get(service_url.rstrip('/')+'/healthz',timeout=5)
        response.raise_for_status()
        health = response.json()
        if health.get('weights_loaded') is not True:
            raise RuntimeError('SAM3 weights not loaded')
    output.mkdir(parents=True,exist_ok=False)
    station = DualARXStation.from_file(config)
    rt = StationRuntime(station,output/'runtime')
    try:
        obs = read_archive(station,directory)
        policy = ProgramExecutor(rt,DualJointTask('Locate a text target on archived RGB-D').context)
        env = policy.api.environment
        env.observation = obs
        env.observation_id = directory.name.removeprefix('observation-')
        env.binding = station.device_binding()
        source = ("result = {}\nfor name in ['global', 'left_wrist', 'right_wrist']:\n"
                  f"    loc = localize_object({prompt!r}, camera_name=name)\n"
                  "    if loc.ok:\n"
                  "        result[name] = {'frame': loc.geometry.frame, 'center_m': loc.geometry.pose.position, "
                  "'score': loc.segmentation.score, 'point_count': loc.geometry.point_count, "
                  "'box_xyxy': loc.segmentation.box_xyxy}\n"
                  "    else:\n        result[name] = {'error': loc.error.message}\n")
        run = policy.execute(source,output/'policy')
        if not run['ok']:
            raise RuntimeError(str(run['error']))
        views = run['result']
        pairs = {}
        for a,b in combinations(views,2):
            if any('error' in views[n] for n in (a,b)):
                continue
            if any(views[n]['frame'] != 'left_base' for n in (a,b)):
                raise ValueError('localization frame mismatch')
            delta = 1000*(np.asarray(views[b]['center_m'])-views[a]['center_m'])
            pairs[f'{a} to {b}'] = dict(delta_mm=delta.tolist(),distance_mm=float(np.linalg.norm(delta)))
        panels = []
        corner_points = {}
        if check_board:
            extrinsics = json.loads(config.read_text())['extrinsics']
            board = make_board(json.loads((config.parent / extrinsics).read_text())['board'])
        events = [json.loads(line) for line in (rt.directory/'events.jsonl').read_text().splitlines()]
        records = [e for e in events if e['kind']=='perception' and e['operation']=='segment_text']
        for role,event in zip(('global','left_wrist','right_wrist'),records,strict=True):
            evidence = rt.directory/event['artifact']
            views[role]['mask_artifact'] = str((evidence/'masks.npz').relative_to(output))
            rgb = obs.cameras[role].rgb
            bgr = cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
            with np.load(evidence/'masks.npz',allow_pickle=False) as masks:
                if 'mask_0' in masks:
                    mask = masks['mask_0']
                    views[role]['mask_pixels'] = int(np.count_nonzero(mask))
                    views[role]['mask_touches_image_edge'] = bool(mask[0].any() or mask[-1].any() or mask[:,0].any() or mask[:,-1].any())
                    depth = obs.cameras[role].depth_m[mask]
                    depth = depth[np.isfinite(depth) & (depth > 0)]
                    if depth.size:
                        quantiles = (0, 1, 50, 99, 100)
                        views[role]['depth_percentiles_m'] = dict(zip(map(str, quantiles), np.percentile(depth, quantiles).tolist()))
                    contours,_ = cv2.findContours(mask.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(bgr,contours,-1,(0,230,255),2)
                    if check_board:
                        corner_points[role], error = matching_board_corners(board, obs.cameras[role], mask)
                        if error:
                            views[role]['corner_detection_error'] = error
            cv2.putText(bgr,role,(12,26),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,0),4)
            cv2.putText(bgr,role,(12,26),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),1,cv2.LINE_AA)
            panels.append(bgr)
        if not cv2.imwrite(str(output/'selected-masks.jpg'),np.hstack(panels)):
            raise RuntimeError('could not save selected mask image')
        report = dict(observation=directory.name,observation_time_s=obs.timestamp_s,prompt=prompt,
                      model='SAM3',service_health=health,calibration=station.calibration_session,
                      calibration_binding=station.device_binding(),
                      camera_capture_span_ms=1000*np.ptp([cam.timestamp_s for cam in obs.cameras.values()]),
                      frame='left_base',center_definition='CaP oriented bounding box of the selected visible depth surface',
                      views=views,pairwise=pairs)
        if check_board and len(corner_points) == 3:
            common = sorted(set.intersection(*(set(points) for points in corner_points.values())))
            distances = {}
            if common:
                for a, b in combinations(corner_points, 2):
                    errors = [1000*np.linalg.norm(corner_points[a][i]-corner_points[b][i]) for i in common]
                    distances[f'{a} to {b}'] = dict(median_mm=float(np.median(errors)), max_mm=float(max(errors)))
            report['matched_charuco_corners'] = dict(common_ids=common, pairwise=distances)
        write_json(output/'comparison.json',report)
        print(json.dumps(report,indent=2))
    finally:
        rt.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('observation',type=Path)
    p.add_argument('output',type=Path)
    p.add_argument('--prompt',default='black and white checkerboard calibration board')
    p.add_argument('--check-board', action='store_true', help='also compare matching ChArUco corners inside all three selected masks')
    p.add_argument('--station',type=Path,default=Path(__file__).resolve().parents[1]/'config/arx-x5-2023-dual.json')
    args=p.parse_args()
    run(args.observation,args.output,args.prompt,args.station,args.check_board)
