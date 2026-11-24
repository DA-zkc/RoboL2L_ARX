import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from robol2l_deploy.arx_calibration import CalibrationSession, detect, make_board, solve, transform
from robol2l_deploy.arx_teach import TeachKey
from robol2l_deploy.cameras import Frame
from robol2l_deploy.runtime import StationRuntime

BOARD = dict(cols=6, rows=5, square_mm=40, marker_mm=30, dictionary='DICT_4X4_50')


def rigid(rot, pos):
    return transform(Rotation.from_rotvec(rot).as_matrix(), pos)


class CalibrationTests(unittest.TestCase):
    def test_known_geometry_and_holdout(self):
        rng = np.random.default_rng(28)
        rows = []
        board_global = rigid([.1,-.4,.2], [.2,0,.8])
        bases = [rigid([.3,.1,.4], [.3,.18,.2]), rigid([-.1,.15,2.1], [.09,-.2,.26])]
        wrists = [rigid([.1,-.23,.13], [.01,.052,.08]), rigid([-.2,.3,-.1], [.05,.07,.1])]
        for side, w, b in zip(('left','right'), wrists, bases):
            for i in range(16):
                hand = rigid(rng.uniform(-.7,.7,3), rng.uniform(-.5,.5,3))
                cam_board = np.linalg.inv(hand @ w) @ b
                rows.append(dict(side=side,T_base_from_flange=hand.tolist(),
                            wrist=dict(T_camera_from_board=cam_board.tolist(),rms_px=.2),
                            **{'global':dict(T_camera_from_board=board_global.tolist(),rms_px=.1)}))
        result = solve(rows)
        expect = [wrists[0], wrists[1], bases[0] @ np.linalg.inv(bases[1]), bases[0] @ np.linalg.inv(board_global)]
        for key, truth in zip(('T_left_flange_from_left_wrist_optical','T_right_flange_from_right_wrist_optical',
                               'T_left_base_from_right_base','T_left_base_from_global_optical'),expect):
            np.testing.assert_allclose(result['transforms'][key], truth, atol=1e-9)
        rows[3]['wrist']['T_camera_from_board'][0][3] += .02
        self.assertAlmostEqual(solve(rows)['errors']['left']['max_mm'], 20, places=5)
        with self.assertRaisesRegex(ValueError, 'no_calibration_samples'):
            solve(rows[:8])
        with self.assertRaisesRegex(ValueError, 'Invalid hand-eye solution'):
            solve([rows[0]]*12 + rows[16:])

    def test_click_captures_real_detector_pixels_and_paired_joints(self):
        board = make_board(BOARD)
        gray = cv2.copyMakeBorder(board.generateImage((480,400)),40,40,80,80,cv2.BORDER_CONSTANT,value=255)
        rgb = cv2.cvtColor(gray,cv2.COLOR_GRAY2RGB)
        dq = np.zeros(6)
        def read_state():
            return SimpleNamespace(timestamp_s=time.time(),joint_positions={'primary':np.zeros(6)},
                   joint_velocities={'primary':dq},end_effector_poses={'primary':SimpleNamespace(as_matrix=lambda:np.eye(4))})
        class Camera:
            def latest(self, **_):
                return Frame(rgb,np.ones_like(gray),dict(fx=600,fy=600,ppx=320,ppy=240,serial='fixture',
                             captured_s=time.time()),time.monotonic())
        with tempfile.TemporaryDirectory() as tmp:
            adapter = SimpleNamespace(device_binding=lambda:'bound', arms={'left':SimpleNamespace(get_robot_state=read_state)},
                                      cameras={'left_wrist':Camera(),'global':Camera()})
            runtime = SimpleNamespace(directory=Path(tmp),adapter=adapter)
            c = CalibrationSession(runtime)
            key = c.start(BOARD)['id']
            state = c.capture('left',key)
            record = json.loads((c.folder / (state['samples'][0]['id']+'.json')).read_text())
            self.assertEqual(len(record['q_rad']), 6)
            for view in ('wrist','global'):
                self.assertLess(record[view]['rms_px'], 1)
                self.assertTrue((c.folder/record[view]['image']).is_file())
            self.assertEqual(CalibrationSession(runtime).status(),state)
            dq[:] = .2
            captured = c.capture('left',key)
            self.assertEqual(len(captured['samples']), 2)
            adapter.device_binding=lambda:'different'
            with self.assertRaisesRegex(ValueError,'changed'):
                c.calculate(key)
        with self.assertRaisesRegex(ValueError,'insufficient'):
            detect(board,np.zeros_like(rgb),np.eye(3))


class TeachTests(unittest.TestCase):
    def make_runtime(self, path):
        armed, held = threading.Event(), threading.Event()
        def hold(side):
            held.set()
            return dict(arm=side)
        adapter = SimpleNamespace(describe=lambda:{}, binding=lambda:'mock', check_teach_start=lambda side:None,
                                  teach_drag=lambda side:armed.set(),teach_hold=hold)
        return StationRuntime(adapter,path), armed, held

    def test_adapter_holds_only_selected_measured_pose(self):
        from test_arx_dual import fixture_station
        station = fixture_station()
        self.addCleanup(station.close)
        left = station.arms['left'].driver.arm
        right = station.arms['right'].driver.arm
        left.q[-1] = .3
        with patch.object(left, 'gravity_compensation', return_value=True, create=True) as gravity:
            station.teach_drag('left')
            receipt = station.teach_hold('left')
        gravity.assert_called_once_with()
        self.assertEqual(receipt['target_rad'], left.q.tolist())
        self.assertEqual(receipt['measured_rad'], left.q.tolist())
        self.assertFalse(right.commands)
        station.allow_motion = False
        with self.assertRaisesRegex(ValueError, 'disabled'):
            station.teach_drag('right')

    def test_rejected_teach_start_sends_neither_drag_nor_hold(self):
        with tempfile.TemporaryDirectory() as tmp, patch('robol2l_deploy.arx_teach.authorize'), patch('robol2l_deploy.arx_teach.revoke'):
            rt, dragged, held = self.make_runtime(tmp)
            with patch.object(rt.adapter,'check_teach_start',side_effect=ValueError('support_arm_and_start_from_rest')):
                key = TeachKey(rt)
                key.signal(key.prepare('left')['token'],False)
                with key.lock:
                    self.assertTrue(key.lock.wait_for(lambda:key.token is None, 2))
                self.assertFalse(dragged.is_set() or held.is_set() or rt.faulted)
                self.assertEqual(key.error,'support_arm_and_start_from_rest')

    def test_feedback_fault_requires_verified_single_arm_recheck(self):
        from test_arx_dual import fixture_station
        for verified in (True, False):
            with self.subTest(verified=verified), tempfile.TemporaryDirectory() as tmp:
                station = fixture_station()
                self.addCleanup(station.close)
                station.allow_motion = False
                rt = StationRuntime(station,tmp)
                rt.event('teach_dispatch', arm='left')
                rt.event('outcome_unknown', operation='teach_hold', arm='left', error='hold_feedback_not_verified')
                rt = StationRuntime(station,tmp)
                key = TeachKey(rt)
                self.assertFalse(key.status()['can_acknowledge'])
                self.assertTrue(key.status()['can_recheck_hold'])
                with self.assertRaises(ValueError):
                    key.acknowledge_parked('right',recheck_hold=True)
                left = station.arms['left'].driver.arm
                with patch.object(station,'realtime_ready',return_value=True), patch.object(left,'set_joint_positions',wraps=left.set_joint_positions) as command:
                    if verified:
                        self.assertEqual(key.acknowledge_parked('left',recheck_hold=True)['phase'],'held')
                    else:
                        command.return_value=False
                        with self.assertRaisesRegex(RuntimeError,'hold_command_not_confirmed'):
                            key.acknowledge_parked('left',recheck_hold=True)
                    self.assertEqual(command.call_count,1)
                self.assertFalse(station.arms['left'].driver.allow_motion or station.arms['right'].driver.arm.commands)
                self.assertEqual(rt.faulted, not verified)
                restarted = StationRuntime(station,tmp)
                self.assertEqual(restarted.faulted,not verified)
                self.assertFalse(TeachKey(restarted).status()['can_recheck_hold'])

    def test_only_parked_limit_fault_can_be_acknowledged_after_restart(self):
        from test_arx_dual import fixture_station
        with tempfile.TemporaryDirectory() as tmp:
            station = fixture_station()
            self.addCleanup(station.close)
            rt = StationRuntime(station, tmp)
            rt.event('teach_dispatch', arm='right')
            rt.event('outcome_unknown', operation='teach_hold', arm='right',
                     error='arx_joint_target_outside_limits: joint5')
            rt = StationRuntime(station, tmp)
            key = TeachKey(rt)
            self.assertTrue(key.status()['can_acknowledge'])
            with self.assertRaisesRegex(ValueError, 'no_matching'):
                key.acknowledge_parked('left')
            arm = station.arms['right'].driver.arm
            arm.q[4] = 0
            with patch.object(arm, 'get_joint_velocities', return_value=np.full(6,.1)):
                self.assertEqual(key.acknowledge_parked('right')['phase'], 'idle')
            self.assertFalse(arm.commands)
            self.assertFalse(rt.faulted)
            self.assertFalse(StationRuntime(station, tmp).faulted)
            rt.event('teach_dispatch', arm='right')
            rt.event('outcome_unknown', operation='teach_hold', arm='right', error='hold_feedback_not_verified')
            self.assertFalse(TeachKey(StationRuntime(station, tmp)).status()['can_acknowledge'])

    def test_saved_hold_receipt_only_allows_readback_after_restart(self):
        from test_arx_dual import fixture_station
        with tempfile.TemporaryDirectory() as tmp:
            station = fixture_station()
            self.addCleanup(station.close)
            rt = StationRuntime(station,tmp)
            rt.event('teach_dispatch',arm='left')
            rt.event('outcome_unknown',operation='teach_hold',arm='left',error='hold_feedback_not_verified')
            rt.event('teach_recheck_dispatch',arm='left')
            rt.event('teach_recheck_held',arm='left',receipt={'target_rad':[0.]*6})
            key = TeachKey(StationRuntime(station,tmp))
            self.assertTrue(key.status()['can_acknowledge'])
            with self.assertRaises(ValueError):
                key.acknowledge_parked('left',recheck_hold=True)
            left = station.arms['left'].driver.arm
            left.q[2] = .011
            self.assertEqual(key.acknowledge_parked('left')['phase'],'idle')
            self.assertFalse(left.commands or station.arms['right'].driver.arm.commands)

    def test_release_timeout_and_unknown_hold(self):
        with tempfile.TemporaryDirectory() as tmp, patch('robol2l_deploy.arx_teach.authorize'), patch('robol2l_deploy.arx_teach.revoke'):
            rt, started, held = self.make_runtime(tmp)
            key = TeachKey(rt)
            def finish():
                with key.lock:
                    self.assertTrue(key.lock.wait_for(lambda:key.token is None, 2))
            for action in ('release','lost_client','failure'):
                token=key.prepare('left')['token']
                key.signal(token,False)
                self.assertTrue(started.wait(1))
                with self.assertRaisesRegex(Exception,'busy'):
                    with rt.lease():
                        pass
                if action=='failure':
                    rt.adapter.teach_hold=lambda s: (_ for _ in ()).throw(RuntimeError('hold failed'))
                if action=='lost_client':
                    time.sleep(.7)
                    self.assertIsNotNone(key.token)
                key.signal(token,True)
                finish()
                if action=='failure':
                    self.assertTrue(rt.faulted)
                    self.assertTrue(StationRuntime(rt.adapter,tmp).faulted)
                else:
                    self.assertTrue(held.is_set())
                    self.assertEqual(key.phase,'held')
                started.clear()
                held.clear()
            token=key.prepare('right')['token']
            key.signal(token,True)
            finish()
            self.assertFalse(started.is_set())

if __name__=='__main__':
    unittest.main()
