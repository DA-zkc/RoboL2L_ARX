"""Offline evaluation of installed X5 extrinsics against a calibration batch and saved RGB-D."""
import argparse
from collections import Counter, defaultdict
from dataclasses import replace
from itertools import combinations
import json
from pathlib import Path
import tempfile

import cv2
import numpy as np

from robol2l_deploy.adapters.arx_dual import DualARXStation
from robol2l_deploy.arx_calibration import detect, make_board, residuals, transform
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cap.physical_task import DualJointTask
from robol2l_deploy.cap.program import ProgramExecutor
from robol2l_deploy.contracts import Segmentation, SegmentationSet
from robol2l_deploy.observations import load_observation
from robol2l_deploy.runtime import StationRuntime


def check_planar_pose(board, record, directory):
    """Disambiguate the two planar fits using additional marker corners in this RGB image."""
    corners = board.getChessboardCorners()[record['ids']].astype(float)
    pixels, k = np.asarray(record['pixels']), np.asarray(record['K'])
    _, rotations, translations, _ = cv2.solvePnPGeneric(corners, pixels, k, None, flags=cv2.SOLVEPNP_IPPE)
    gray = cv2.imread(str(directory/record['image']), cv2.IMREAD_GRAYSCALE)
    enlarged = cv2.resize(gray, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC)
    options = cv2.aruco.DetectorParameters()
    options.minMarkerPerimeterRate = .012
    options.adaptiveThreshWinSizeMax = 71
    options.adaptiveThreshWinSizeStep = 4
    options.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    marks, ids, _ = cv2.aruco.ArucoDetector(board.getDictionary(), options).detectMarkers(enlarged)
    known = dict(zip(board.getIds(), board.getObjPoints()))
    pairs = [(known[i], (quad.reshape(4, 2)+.5)/4-.5) for i, quad in zip(ids.ravel(), marks) if i in known]
    obj, img = map(np.concatenate, zip(*pairs))
    candidates = []
    for rv, tv in zip(rotations, translations):
        rv, tv = cv2.solvePnPRefineLM(corners, pixels, k, None, rv, tv)
        projected = cv2.projectPoints(obj, rv, tv, k, None)[0].reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum((projected-img)**2, axis=1))))
        candidates.append((rms, transform(cv2.Rodrigues(rv)[0], tv)))
    return min(candidates, key=lambda item: item[0])[1], dict(
        decoded_markers=len(pairs), candidate_marker_rms_px=[x[0] for x in candidates])


def check_batch(station, batch, board, directory):
    poses, verified, checks = defaultdict(list), defaultdict(list), {}
    counts = Counter()
    for row in batch['samples']:
        sample = json.loads((directory/(row['id']+'.json')).read_text())
        side = sample['side']
        counts[side] += 1
        if counts[side] % 4:
            continue
        flange = station.arms[side]._pose(sample['q_rad']).as_matrix()
        np.testing.assert_allclose(flange, sample['T_base_from_flange'], atol=1e-7)
        wrist = station.camera_pose(side+'_wrist', flange).as_matrix() @ sample['wrist']['T_camera_from_board']
        global_camera = station.camera_pose('global').as_matrix()
        seen = global_camera @ sample['global']['T_camera_from_board']
        poses[side].append(np.linalg.solve(seen, wrist))
        # This specific held-out image was flagged in the original calibration review.
        if row['id'] == 'd98d8a66fd184413b5ebacb5f4dad547':
            corrected, checks[row['id']] = check_planar_pose(board, sample['global'], directory)
            seen = global_camera @ corrected
        verified[side].append(np.linalg.solve(seen, wrist))
    return dict(sample_counts=dict(counts), held_out={side:len(v) for side,v in poses.items()},
                original={s:residuals(np.eye(4),v) for s,v in poses.items()},
                reviewed={s:residuals(np.eye(4),v) for s,v in verified.items()}, planar_check=checks)


def check_depth(station, api, board, directory, binding):
    differences = defaultdict(list)
    frames = []
    policy_result = None
    for path in sorted(directory.glob('observation-*/metadata.json')):
        meta = json.loads(path.read_text())
        if meta.get('device_binding') != binding or set(meta['cameras']) != set(station.cameras):
            continue
        obs = load_observation(path.parent)
        cameras, poses = {}, {}
        for side in ('left','right'):
            local = station.arms[side]._pose(obs.robot_state.joint_positions[side])
            np.testing.assert_allclose(local.as_matrix(),obs.robot_state.end_effector_poses[side].as_matrix(),atol=1e-7)
            poses[side] = local.as_matrix()
        for name, camera in obs.cameras.items():
            if meta['measurement_diagnostics'][name]['serial'] != station.cameras[name].source.serial:
                raise ValueError('recorded camera serial mismatch')
            pose = station.camera_pose(name, poses.get(name.removesuffix('_wrist')))
            cameras[name] = replace(camera, camera_pose=pose)
        state = replace(obs.robot_state, base_frame='left_base', end_effector_poses={
            side:station.shared_flange(side,local) for side,local in poses.items()})
        api.environment.observation = replace(obs, cameras=cameras, robot_state=state)
        project = api.tools()['mask_to_point_cloud']
        points = {}
        for name, camera in cameras.items():
            points[name] = {}
            try:
                detection = detect(board, camera.rgb, camera.intrinsics)
            except ValueError:
                continue
            mask = np.zeros(camera.depth_m.shape,bool)
            for tag, (x,y) in zip(detection['ids'], np.rint(detection['pixels']).astype(int)):
                if not (0 <= x < mask.shape[1] and 0 <= y < mask.shape[0] and np.isfinite(camera.depth_m[y,x])
                        and camera.depth_m[y,x] > 0):
                    continue
                mask[y,x] = True
                cloud = project(mask, name)
                assert cloud.frame == 'left_base'
                points[name][tag] = cloud.points[0]
                mask[y,x] = False
        common = sorted(set.intersection(*(set(v) for v in points.values())))
        if common and policy_result is None:
            # Exercise localize_object with the known calibration target and measured depth.
            # This detector is local to the offline test. It does not stand in for a SAM3 benchmark.
            class CornerDetector:
                def segment_text(self, camera, query):
                    detected = detect(board,camera.rgb,camera.intrinsics)
                    x,y = detected['pixels'][detected['ids'].index(int(query))]
                    yy,xx = np.ogrid[:camera.depth_m.shape[0],:camera.depth_m.shape[1]]
                    mask = (xx-x)**2 + (yy-y)**2 <= 6**2
                    return SegmentationSet(True,(Segmentation(mask,query,1.,camera.frame,camera.frame),))
            executor = ProgramExecutor(api.runtime,api.task_context)
            executor.api = PublicAPI(api.runtime,api.task_context,providers={'segmentation':CornerDetector()})
            executor.api.environment.observation = api.environment.observation
            source = ("result = {}\nfor name in ['global','left_wrist','right_wrist']:\n"
                      f"    location = localize_object('{common[0]}', camera_name=name)\n"
                      "    result[name] = [location.geometry.pose.frame, location.geometry.pose.position]")
            run = executor.execute(source,api.runtime.directory/'board-policy')
            if not run['ok'] or any(row[0] != 'left_base' for row in run['result'].values()):
                raise ValueError(f"archived board policy failed: {run['error']}")
            policy_result = dict(observation=path.parent.name,corner_id=common[0], xyz_m=run['result'],
                                 provider='offline ChArUco corner neighbourhood')
        frames.append(dict(observation=path.parent.name, common_three_view_ids=common,
                           valid_corners={n:len(v) for n,v in points.items()}))
        for a,b in combinations(sorted(points),2):
            distances = differences[a+' / '+b]
            distances.extend(1000*np.linalg.norm(points[a][i]-points[b][i])
                             for i in points[a].keys() & points[b].keys())
    if not any(frame['common_three_view_ids'] for frame in frames):
        raise ValueError('no archived depth observation with a corner visible to all three cameras')
    return dict(observations=frames, isolated_policy=policy_result, pairs={k:dict(count=len(v), median_mm=float(np.median(v)),
                rms_mm=float(np.sqrt(np.mean(np.square(v)))), max_mm=float(max(v))) for k,v in differences.items() if v})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch',type=Path)
    parser.add_argument('--station',type=Path,default=Path(__file__).resolve().parents[1]/'config/arx-x5-2023-dual.json')
    args = parser.parse_args()
    station = DualARXStation.from_file(args.station)
    batch = json.loads((args.batch/'session.json').read_text())
    if batch['id'] != station.calibration_session or batch['binding'] != station.calibration_binding():
        raise ValueError('batch does not match installed calibration')
    board = make_board(batch['board'])
    with tempfile.TemporaryDirectory() as tmp:
        runtime = StationRuntime(station,tmp)
        try:
            api = PublicAPI(runtime,DualJointTask('Offline board localization').context,providers={})
            report = dict(session=batch['id'],frame='left_base',
                          calibration=check_batch(station,batch,board,args.batch),
                          recorded_depth=check_depth(station,api,board,args.batch.parent.parent,batch['binding']))
            print(json.dumps(report,indent=2,allow_nan=False))
        finally:
            runtime.close()


if __name__ == '__main__':
    main()
