"""Static ChArUco board. T_A_from_B maps B coordinates into A, in metres."""
import json
import time
import uuid

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .cap.storage import write_json
from .cameras import rectify

SIDES = ('left', 'right')
DICTIONARIES = tuple(f'DICT_{n}X{n}_{size}' for n in range(4, 8)
                     for size in (50, 100, 250, 1000)) + ('DICT_ARUCO_ORIGINAL',)


def make_board(config):
    cols, rows = config['cols'], config['rows']
    square, marker = config['square_mm'], config['marker_mm']
    name = config['dictionary']
    if (type(cols) != int or type(rows) != int or not (cols > 1 and rows > 1)
            or not 0 < marker < square or name not in DICTIONARIES):
        raise ValueError('请核对行列、字典、格子和标签的实测尺寸 / Invalid board parameters')
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
    if cols * rows // 2 > len(dictionary.bytesList):
        raise ValueError('标记字典容量不足 / Dictionary too small')
    board = cv2.aruco.CharucoBoard((cols, rows), square / 1000, marker / 1000, dictionary)
    board.setLegacyPattern(config.get('legacy') is True)
    return board


def transform(rotation, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.ravel(translation)
    return matrix


def average(matrices):
    values = np.stack(matrices)
    return transform(Rotation.from_matrix(values[:, :3, :3]).mean().as_matrix(), values[:, :3, 3].mean(axis=0))


def detect(board, rgb, k):
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    detector = cv2.aruco.CharucoDetector(board)
    # Keep the original image path. On small codes, upsampling and local contrast
    # reveal additional IDs even when the native pass already has six corners.
    enlarged = cv2.resize(gray, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMax = 51
    params.adaptiveThreshWinSizeStep = 4
    fine = cv2.aruco.CharucoDetector(board, cv2.aruco.CharucoParameters(), params)
    sharp = cv2.addWeighted(enlarged, 2., cv2.GaussianBlur(enlarged, (0, 0), 2.4), -1., 0)
    best, max_count, last_error = None, 0, None
    for engine, image, scale in ((detector, gray, 1), (detector, enlarged, 3), (fine, sharp, 3)):
        corners, ids, _, _ = engine.detectBoard(image)
        count = 0 if ids is None else len(ids)
        max_count = max(max_count, count)
        if not count or board.checkCharucoCornersCollinear(ids):
            continue
        if scale != 1:
            corners = (corners + .5) / scale - .5
        try:
            result = _measure_corners(board, corners, ids, k)
        except (ValueError, cv2.error) as error:
            last_error = error
            continue
        if best is None or (count, -result['rms_px']) > (len(best['ids']), -best['rms_px']):
            best = result
    if best is not None:
        return best
    if last_error is not None:
        raise last_error
    raise ValueError(f'最多识别 {max_count} 个内角点，没有可求解的非共线角点集合 / Board corners insufficient')


def _measure_corners(board, corners, ids, k):
    points, pixels = board.matchImagePoints(corners, ids)
    ok, rv, tv = cv2.solvePnP(points, pixels, k, None)
    if not ok or not np.isfinite(np.r_[rv, tv]).all() or tv[2, 0] <= 0:
        raise ValueError('标定板位姿不可用 / Invalid board pose')
    projected = cv2.projectPoints(points, rv, tv, k, None)[0]
    rms = float(np.sqrt(np.mean(np.sum((pixels.reshape(-1, 2) - projected.reshape(-1, 2))**2, axis=1))))
    return dict(T_camera_from_board=transform(cv2.Rodrigues(rv)[0], tv).tolist(),
                ids=ids.ravel().tolist(), pixels=pixels.reshape(-1, 2).tolist(), K=k.tolist(), rms_px=rms)


def residuals(reference, estimates):
    if not len(estimates):
        return dict(rms_mm=None, max_mm=None, rms_deg=None, max_deg=None)
    rel = np.linalg.inv(reference) @ np.stack(estimates)
    mm = 1000 * np.linalg.norm(rel[:, :3, 3], axis=1)
    deg = np.rad2deg(Rotation.from_matrix(rel[:, :3, :3]).magnitude())
    return dict(rms_mm=float(np.sqrt(np.mean(mm**2))), max_mm=float(max(mm)),
                rms_deg=float(np.sqrt(np.mean(deg**2))), max_deg=float(max(deg)))


def solve(samples):
    transforms, errors = {}, {}
    training_global, validation_global = [], []
    for side in SIDES:
        rows = [s for s in samples if s['side'] == side]
        if not rows:
            raise ValueError(f'{side}: no_calibration_samples')
        hand = np.array([s['T_base_from_flange'] for s in rows])
        board = np.array([s['wrist']['T_camera_from_board'] for s in rows])
        holdout = np.arange(len(rows)) % 4 == 3
        for s, withheld in zip(rows, holdout):
            (validation_global if withheld else training_global).append(s['global']['T_camera_from_board'])
        h, b = hand[~holdout], board[~holdout]
        rotations = Rotation.from_matrix(h[0, :3, :3].T @ h[:, :3, :3]).as_rotvec()
        span = np.linalg.svd(rotations, compute_uv=False)
        rot, t = cv2.calibrateHandEye(h[:, :3, :3], h[:, :3, 3], b[:, :3, :3], b[:, :3, 3],
                                      method=cv2.CALIB_HAND_EYE_PARK)
        wrist = transform(rot, t)
        if not np.isfinite(wrist).all() or not np.isclose(np.linalg.det(rot), 1):
            raise ValueError(f'{side} 手眼求解退化 / Invalid hand-eye solution')
        predictions = hand @ wrist @ board
        fixed = average(predictions[~holdout])
        errors[side] = dict(training=int(sum(~holdout)), validation=int(sum(holdout)),
                            **residuals(fixed, predictions[holdout]),
                            worst_detection_px=max(s['wrist']['rms_px'] for s in rows))
        transforms[f'T_{side}_base_from_board'] = fixed
        transforms[f'T_{side}_flange_from_{side}_wrist_optical'] = wrist
    global_pose = average(training_global)
    errors['global'] = dict(**residuals(global_pose, validation_global),
                            worst_detection_px=max(s['global']['rms_px'] for s in samples))
    left = transforms['T_left_base_from_board']
    transforms['T_left_base_from_right_base'] = left @ np.linalg.inv(transforms['T_right_base_from_board'])
    transforms['T_left_base_from_global_optical'] = left @ np.linalg.inv(global_pose)
    # Compare each held-out wrist against the simultaneous global view in left_base.
    cross = {}
    for side in SIDES:
        deltas = []
        for s in [s for s in samples if s['side'] == side][3::4]:
            base = np.eye(4) if side == 'left' else transforms['T_left_base_from_right_base']
            predicted = (base @ s['T_base_from_flange'] @ transforms[f'T_{side}_flange_from_{side}_wrist_optical']
                         @ s['wrist']['T_camera_from_board'])
            seen = transforms['T_left_base_from_global_optical'] @ s['global']['T_camera_from_board']
            deltas.append(np.linalg.inv(seen) @ predicted)
        cross[side] = residuals(np.eye(4), deltas)
    return dict(convention='T_parent_from_child, translations in metres; optical frames use RGB axes',
                transforms={k: v.tolist() for k, v in transforms.items()}, errors=errors, cross_view=cross)


class CalibrationSession:
    def __init__(self, runtime):
        self.runtime = runtime
        self.root = runtime.directory / 'arx-calibration'
        pointer = self.root / 'current.json'
        self.folder = self.root / json.loads(pointer.read_text())['id'] if pointer.exists() else None

    def status(self):
        if self.folder is None:
            return dict(id=None)
        return json.loads((self.folder / 'session.json').read_text())

    def _binding(self):
        adapter = self.runtime.adapter
        return getattr(adapter, 'calibration_binding', adapter.device_binding)()

    def start(self, board):
        make_board(board)
        key = uuid.uuid4().hex
        folder = self.root / key
        folder.mkdir(parents=True)
        state = dict(id=key, board=board, binding=self._binding(), samples=[], report=None)
        write_json(folder / 'session.json', state)
        write_json(self.root / 'current.json', dict(id=key))
        self.folder = folder
        return state

    def checked(self, session_id):
        state = self.status()
        if not session_id or session_id != state['id'] or state['binding'] != self._binding():
            raise ValueError('工作站或采集批次已改变 / Station or session changed')
        return state

    def capture(self, side, session_id):
        state = self.checked(session_id)
        if side not in SIDES:
            raise ValueError('invalid_arm')
        adapter = self.runtime.adapter
        roles = {'wrist': side + '_wrist', 'global': 'global'}
        owners = adapter.cameras
        for role in roles.values():
            owners[role].latest()
        board = make_board(state['board'])
        unit = adapter.arms[side]
        # Bracket acquisition by measured encoder states, using the host camera timestamp
        # as well as arrival time to reject queued frames from before the button press.
        before = unit.get_robot_state()
        start = time.monotonic()
        frames = {}
        for key, role in roles.items():
            cursor = start
            while True:
                frame = owners[role].latest(after=cursor)
                if frame.metadata['captured_s'] >= before.timestamp_s:
                    frames[key] = frame
                    break
                cursor = frame.monotonic_s
        after = unit.get_robot_state()
        q0, q1 = (r.joint_positions['primary'] for r in (before, after))
        dq = np.stack([r.joint_velocities['primary'] for r in (before, after)])
        if not np.isfinite(np.r_[q0, q1, dq.ravel()]).all():
            raise ValueError('invalid_joint_feedback')
        sample = dict(id=uuid.uuid4().hex, side=side, q_rad=q1.tolist(), q_before_rad=q0.tolist(),
                      state_times_s=[before.timestamp_s, after.timestamp_s],
                      joint_velocity_rad_s=dq.tolist(), joint_delta_rad=(q1-q0).tolist(),
                      stationary_verified=False,
                      T_base_from_flange=after.end_effector_poses['primary'].as_matrix().tolist())
        images = {}
        for key, frame in frames.items():
            rgb, _, k = rectify(frame)
            try:
                measurement = detect(board, rgb, k)
            except (ValueError, cv2.error) as error:
                raise ValueError(f'{roles[key]}: {error}') from error
            filename = f"{sample['id']}-{key}.png"
            sample[key] = dict(**measurement, image=filename, serial=frame.metadata['serial'],
                               captured_s=frame.metadata['captured_s'])
            images[filename] = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        for filename, image in images.items():
            if not cv2.imwrite(str(self.folder / filename), image):
                raise OSError('image_write_failed')
        write_json(self.folder / (sample['id'] + '.json'), sample)
        state['samples'].append(dict(id=sample['id'], side=side))
        state['report'] = None
        write_json(self.folder / 'session.json', state)
        return state

    def calculate(self, session_id):
        state = self.checked(session_id)
        samples = [json.loads((self.folder / (s['id']+'.json')).read_text()) for s in state['samples']]
        report = dict(**solve(samples), session=state['id'], board=state['board'], binding=state['binding'],
                      sample_ids=[s['id'] for s in samples], ports=dict(self.runtime.adapter.ports),
                      cameras={role: owner.source.serial for role, owner in self.runtime.adapter.cameras.items()})
        write_json(self.folder / 'extrinsics.json', report)
        state['report'] = report
        write_json(self.folder / 'session.json', state)
        return state
