"""Empty-hand commissioning and two-arm handover ordering on a simulated X5."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import numpy as np

from robol2l_deploy.arx_gripper_calibration import GripperCalibration
from robol2l_deploy.adapters.arx_dual import DualARXStation
from robol2l_deploy.cap.physical_task import DualJointTask
from robol2l_deploy.cap.program import ProgramExecutor
from robol2l_deploy.contracts import CameraObservation, Observation
from robol2l_deploy.runtime import StationRuntime
from test_arx_dual import CONFIG, fixture_station


class CommissioningTests(unittest.TestCase):
    def setUp(self):
        tmp = TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.station = fixture_station()
        self.rt = StationRuntime(self.station, self.root/'session', allow_motion=True)
        self.addCleanup(self.rt.close)
        self.arms = {s:u.driver.arm for s,u in self.station.arms.items()}

    def test_station_home_survives_new_session_and_is_bound_to_the_two_ports(self):
        cfg = json.loads(CONFIG.read_text())
        cfg.update(cameras={}, extrinsics=None, payload=None, grasp_geometry=None)
        saved = json.loads((CONFIG.parent/cfg['commissioning']).read_text())
        local = self.root / cfg['commissioning']
        local.write_text(json.dumps(saved))
        for session in ('first', 'second'):
            station = DualARXStation(cfg, directory=self.root)
            backend = StationRuntime(station, self.root / session)
            try:
                self.assertEqual(backend.home, saved['home'])
                self.assertEqual(json.loads(backend.home_path.read_text()), saved['home'])
            finally:
                backend.close()
        cfg['arms'] = {'left':'can3','right':'can1'}
        with self.assertRaisesRegex(ValueError,'commissioning_can_binding_changed'):
            DualARXStation(cfg,directory=self.root)

    def test_repeated_empty_measurements_enable_only_selected_side(self):
        a = self.station
        a.commissioning_path = self.root/'commissioning.json'
        a.commissioning = dict(ports=a.ports, grippers={})
        h = GripperCalibration(self.rt)
        h.record('left', 'open_rad')
        with self.assertRaisesRegex(ValueError, 'revisit'):
            h.record('left','open_rad')
        def permit(rt, *args):
            rt.allow_motion = True
            rt.adapter.allow_motion = True
        with patch('robol2l_deploy.arx_gripper_calibration.authorize', side_effect=permit):
            for target, tag in ((.02,'closed_rad'),(1.,'open_rad'),(.02,'closed_rad')):
                self.assertTrue(h.jog('left',target_rad=target).ok)
                record = h.record('left', tag)
        self.assertTrue(record['calibrated'])
        self.assertEqual(a.arms['left'].gripper,(.02,1.))
        self.assertEqual(self.arms['right'].commands,[])
        self.assertFalse(self.rt.allow_motion)
        self.assertEqual(json.loads(a.commissioning_path.read_text())['grippers'], {'left':record['endpoints']})

    def test_review_only_matched_gripper_fault_does_not_dispatch(self):
        rt = self.rt
        rt.faulted, rt.allow_motion = True, False
        rt.jobs['jog'] = dict(status='outcome_unknown', operation='calibrate_gripper_jog', dispatch='sent')
        rt.event('dispatch', operation='gripper_calibration_left', job_id='jog')
        rt.event('outcome_unknown', operation='gripper_calibration_left',
                 diagnostics=dict(cause_message='gripper_calibration_increment_unverified'))
        hands = GripperCalibration(rt)
        with self.assertRaisesRegex(ValueError, 'no_matching'):
            hands.acknowledge('right')
        self.assertTrue(rt.faulted)
        self.assertTrue(hands.acknowledge('left')['acknowledged'])
        self.assertFalse(rt.faulted)
        self.assertFalse(rt.allow_motion)
        self.assertFalse(any(arm.commands for arm in self.arms.values()))

    def test_custom_increment_uses_one_selected_gripper_command(self):
        self.station.commissioning_path = self.root/'commissioning.json'
        hands = GripperCalibration(self.rt)
        with patch('robol2l_deploy.arx_gripper_calibration.authorize'):
            result = hands.jog('left', .6)
        self.assertTrue(result.ok)
        self.assertAlmostEqual(result.diagnostics['target_rad'], 1.6)
        self.assertEqual(len(self.arms['left'].commands), 1)
        self.assertEqual(self.arms['right'].commands, [])
        self.assertFalse(self.rt.allow_motion)

    def test_invalid_or_out_of_range_increment_never_dispatches(self):
        self.station.commissioning_path = self.root/'commissioning.json'
        hands = GripperCalibration(self.rt)
        for delta in (0, True, '0.5', None, float('nan'), float('inf'), -float('inf'), -1.1, 4.1):
            with self.subTest(delta=delta), self.assertRaises(ValueError):
                hands.jog('left', delta)
        self.assertFalse(any(arm.commands for arm in self.arms.values()))

    def test_closed_remeasurement_saves_command_and_both_measured_values(self):
        station = self.station
        station.commissioning_path = self.root/'commissioning.json'
        right = dict(open_rad=4.8, closed_rad=.07)
        station.commissioning = dict(ports=station.ports, grippers={
            'left':dict(open_rad=4.8, closed_rad=.07), 'right':right})
        hands = GripperCalibration(self.rt)
        arm = self.arms['left']
        original = arm.set_gripper_pos
        def offset_feedback(target, duration):
            result = original(target, duration)
            if target <= .1:
                arm.gripper = target + .06
            return result
        def permit(rt, *args):
            rt.allow_motion = True
            rt.adapter.allow_motion = True
        with patch.object(arm, 'set_gripper_pos', side_effect=offset_feedback), \
                patch('robol2l_deploy.arx_gripper_calibration.authorize', side_effect=permit):
            self.assertTrue(hands.jog('left', target_rad=.02).ok)
            self.assertFalse(hands.record('left', 'closed_rad')['calibrated'])
            with self.assertRaisesRegex(ValueError, 'reopen'):
                hands.record('left', 'closed_rad')
            self.assertTrue(hands.jog('left', target_rad=1.).ok)
            self.assertTrue(hands.jog('left', target_rad=.02).ok)
            result = hands.record('left', 'closed_rad')
        self.assertTrue(result['calibrated'])
        self.assertEqual(result['endpoints'], dict(open_rad=4.8, closed_rad=.02))
        self.assertEqual(result['validation']['measured_rad'], [.08, .08])
        self.assertEqual(station.commissioning['grippers']['right'], right)
        self.assertEqual(self.arms['right'].commands, [])
        self.assertEqual(json.loads(station.commissioning_path.read_text())['grippers']['left']['closed_rad'], .02)

    def test_closed_record_rejects_unverified_or_insufficient_closure(self):
        self.station.commissioning_path = self.root/'commissioning.json'
        hands = GripperCalibration(self.rt)
        with self.assertRaisesRegex(ValueError, 'test_selected'):
            hands.record('left', 'closed_rad')
        self.rt.last_execution = dict(result_ok=True, diagnostics=dict(side='left', target_rad=.02))
        self.arms['left'].gripper = .12
        with self.assertRaisesRegex(ValueError, 'at_most'):
            hands.record('left', 'closed_rad')
        self.assertEqual(hands.samples, {})

    def test_absolute_target_validation_never_dispatches(self):
        self.station.commissioning_path = self.root/'commissioning.json'
        hands = GripperCalibration(self.rt)
        for target in (-.01, 5.01, True, '0.02', float('nan'), float('inf')):
            with self.subTest(target=target), self.assertRaises(ValueError):
                hands.jog('left', target_rad=target)
        with self.assertRaises(ValueError):
            hands.jog('left', delta_rad=.1, target_rad=.02)
        self.assertFalse(any(arm.commands for arm in self.arms.values()))

    def test_isolated_handover_feedback_revision_and_unknown(self):
        for unit in self.station.arms.values():
            unit.gripper = (.2, 3.)
        self.station.unsupported_operations.difference_update({'close_gripper','open_gripper','set_gripper'})
        executor = ProgramExecutor(self.rt, DualJointTask('Software handover').context)
        # A simulated receiving-side witness independent of both finger encoders.
        witness = [False]
        def observed(*args):
            state = self.station.get_robot_state()
            rgb = np.zeros((4,4,3),np.uint8)
            rgb[0,0,1] = 255 if witness[0] else 0
            cam = CameraObservation(rgb,np.ones((4,4)),np.eye(3),'witness',
                                    state.end_effector_poses['right'],state.timestamp_s)
            return Observation({'witness':cam},state,timestamp_s=state.timestamp_s)
        def run(program, label):
            res = executor.execute(program,self.root/label)
            self.assertTrue(res['ok'],res['error'])
            return res['result']
        check = ("evidence = get_observation().cameras['witness'].rgb[0,0,1]\n"
                 "result = 'waiting_for_receiver'\n"
                 "if evidence == 255:\n    result = open_gripper(arm='left').ok")
        with patch.object(self.station,'check_hands',side_effect=ValueError('stale target; read feedback')):
            self.assertFalse(run("result = close_gripper(arm='right').ok",'v0'))
        self.assertFalse(self.rt.faulted)
        self.assertEqual(self.arms['right'].commands, [])
        with patch.object(self.station, 'get_observation', side_effect=observed):
            self.assertEqual(run("state = get_robot_state()\nr = close_gripper(arm='right')\nif not r.ok:\n    raise ValueError(r.error.message)\n"+check,'v1'),
                             'waiting_for_receiver')
            self.assertEqual(self.arms['left'].commands, [])
            witness[0] = True
            self.assertTrue(run('state = get_robot_state()\n'+check,'v2'))
        sent = len(self.arms['left'].commands)
        with patch.object(self.arms['right'],'set_gripper_pos',side_effect=RuntimeError('lost feedback')):
            self.assertFalse(run("result = close_gripper(arm='right').ok",'v3'))
        self.assertTrue(self.rt.faulted)
        blocked = executor.execute("result = open_gripper(arm='left').ok",self.root/'v4')
        self.assertFalse(blocked['ok'])
        self.assertEqual(blocked['error']['message'], 'runtime_fault_latched')
        self.assertEqual(len(self.arms['left'].commands),sent)
