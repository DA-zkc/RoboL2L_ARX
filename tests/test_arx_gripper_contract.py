"""Completion semantics reach real CaP request construction without hardware I/O."""
import json
from dataclasses import replace
from tempfile import TemporaryDirectory
import unittest

from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cap.experiment import ExperimentController
from robol2l_deploy.cap.tasks import MockReachTask
from robol2l_deploy.runtime import StationRuntime
import test_arx_adapter
import test_arx_dual


class GripperContractTests(unittest.TestCase):
    def check_requests(self, runtime, observation=None):
        controller = ExperimentController(runtime, lambda: None)
        controller.state = dict(session_id='contract-test', goal='fixture', criteria='fixture',
                                history=[], attempts=1, initial={}, speed_factor=.1)
        if observation is None:
            observation = runtime.get_observation()
        for phase in ('generate', 'review'):
            with self.subTest(phase=phase):
                request, _ = controller.build_request(observation, phase)
                sent = json.loads(request[1]['content'][0]['text'])
                self.assertEqual(sent['experiment']['phase'], phase)
                api = sent['api']
                timing = api['capabilities']['trajectory_gripper_timing']
                for text in (timing, api['grasp_semantics']):
                    self.assertIn('duration=None', text)
                    self.assertIn('no active gripper interpolation trajectory', text)
                    self.assertIn('may return immediately', text)
                    self.assertIn('fingers are still moving', text)
                self.assertIn('target_reached=None', api['grasp_semantics'])
                self.assertIn('single opening', api['grasp_semantics'])
                self.assertNotIn('blocking dwell', json.dumps(api))
                self.assertNotIn('adds dwell', json.dumps(api))

    def test_single_arx_calibrated_and_unavailable_gripper(self):
        fixture = test_arx_adapter.ARXAdapterTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.assertEqual(fixture.runtime.describe()['trajectory_gripper_timing'], 'unavailable')
        fixture.adapter.gripper = (.02, 4.87)
        fixture.adapter.unsupported_operations.difference_update(
            {'set_gripper', 'set_grippers', 'open_gripper', 'close_gripper'})
        observation = replace(MockAdapter().get_observation(),
                              robot_state=fixture.runtime.get_robot_state())
        self.check_requests(fixture.runtime, observation)
        self.assertFalse(fixture.arm.commands)

    def test_dual_arx_model_generate_and_review_get_direct_target_contract(self):
        with TemporaryDirectory() as tmp:
            station = test_arx_dual.fixture_station()
            for unit in station.arms.values():
                unit.gripper = (.02, 4.87)
            station.unsupported_operations.difference_update(
                {'set_gripper', 'set_grippers', 'open_gripper', 'close_gripper'})
            runtime = StationRuntime(station, tmp)
            self.addCleanup(runtime.close)
            self.check_requests(runtime)
            self.assertTrue(all(not unit.driver.arm.commands for unit in station.arms.values()))

    def test_other_backends_keep_their_existing_timing_description(self):
        with TemporaryDirectory() as tmp:
            runtime = StationRuntime(MockAdapter(), tmp)
            self.addCleanup(runtime.close)
            api = PublicAPI(runtime, MockReachTask.context).describe()
            self.assertIn('blocking hand commands', api['capabilities']['trajectory_gripper_timing'])
            self.assertNotIn('duration=None', api['grasp_semantics'])
