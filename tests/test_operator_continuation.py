"""Offline human continuation intent, reset readiness and attempt boundaries."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.runtime import StationRuntime
from robol2l_deploy.cap.experiment import ExperimentController
from robol2l_deploy.contracts import ExecutionResult
from robol2l_deploy.errors import ApiError, ErrorCode


PROGRAM = dict(kind='program', reason='Measure the scene', source='result = get_robot_state()')
ASSIST = dict(kind='assist', reason='Prior cross-view center difference needs investigation')


class OperatorContinuationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.runtime = StationRuntime(MockAdapter(), Path(self.tmp.name))
        self.model = Mock(label='offline_continuation_model', settings={'model': 'mock'})
        self.model.decide.side_effect = [PROGRAM, ASSIST]
        self.controller = ExperimentController(self.runtime, lambda: self.model)
        self.controller.is_arx = True  # ARX orchestration; all devices remain Mock.
        self.controller.start('Move object', 'Object reaches target', True)
        self.wait()
        self.assertEqual(self.controller.state['status'], 'waiting_operator')
        self.assertEqual(self.controller.state['attempts'], 1)

    def tearDown(self):
        self.controller.stop.set()
        if self.controller.thread:
            self.controller.thread.join(15)
        self.runtime.close()
        self.tmp.cleanup()

    def wait(self):
        self.controller.thread.join(15)
        self.assertFalse(self.controller.busy())

    def authorize(self, note='I inspected the scene; continue with a revised approach'):
        self.controller.authorize_reset(self.controller.state['session_id'], True,
                                        note, self.controller.state['attempts'])
        self.wait()

    def test_explicit_continue_reaches_next_generation_with_old_evidence_and_note(self):
        requests = []
        def decide(request):
            payload = json.loads(request[1]['content'][0]['text'])
            requests.append(payload)
            exp = payload['experiment']
            self.assertFalse(self.runtime.allow_motion)
            continuation = exp['operator_continuation']
            if exp['phase'] == 'generate':
                self.assertTrue(continuation['applies_to_this_generation'])
                self.assertEqual(continuation['readiness'], 'return_command_completed')
                self.assertEqual(continuation['prior_assistance']['reason'], ASSIST['reason'])
                self.assertIn('revised approach', continuation['note'])
                self.assertFalse(continuation['evidence_resolved'])
                self.assertFalse(continuation['physical_arrival_verified'])
                self.assertTrue(any(h.get('decision', {}).get('kind') == 'assist'
                                    for h in payload['history']))
                self.assertIn('do not repeat the same', request[0]['content'])
                return PROGRAM
            self.assertFalse(continuation['applies_to_this_generation'])
            return dict(kind='finish', reason='Mock completion')
        self.model.decide.side_effect = decide
        self.authorize()
        self.assertEqual(self.controller.state['attempts'], 2)
        self.assertEqual(self.controller.state['status'], 'completed')
        self.assertEqual([r['experiment']['phase'] for r in requests], ['generate', 'review'])
        stored = json.loads((self.controller.folder / 'experiment.json').read_text())
        self.assertEqual(stored['operator_continuation']['for_attempt'], 1)
        _, context = self.controller.build_request(self.runtime.get_observation(), 'generate', history=[])
        self.assertFalse(context['experiment']['operator_continuation']['applies_to_this_generation'])
        self.assertEqual(context['experiment']['operator_continuation']['note'], stored['operator_continuation']['note'])

    def test_failed_return_never_activates_continuation_or_calls_model(self):
        count = self.model.decide.call_count
        failure = ExecutionResult(False, 0, error=ApiError(ErrorCode.CONTROLLER_FAILED, 'Mock return failed'))
        with patch.object(self.runtime.adapter, 'plan_recorded_return', create=True), \
                patch.object(self.runtime, 'return_recorded_joints', return_value=failure):
            self.authorize()
        self.assertEqual(self.model.decide.call_count, count)
        self.assertEqual(self.controller.state['status'], 'waiting_operator')
        _, context = self.controller.build_request(self.runtime.get_observation(), 'generate')
        self.assertFalse(context['experiment']['operator_continuation']['applies_to_this_generation'])

    def test_stale_confirmation_does_not_record_a_continuation(self):
        with self.assertRaisesRegex(ValueError, 'stale_reset'):
            self.controller.authorize_reset(self.controller.state['session_id'], True, '', 0)
        self.assertNotIn('operator_continuation', self.controller.state)

    def test_new_evidence_can_still_request_assistance_without_dispatch(self):
        self.model.decide.side_effect = None
        self.model.decide.return_value = dict(kind='assist', reason='New observation: object fell outside workspace')
        self.authorize()
        self.assertEqual(self.controller.state['status'], 'waiting_operator')
        self.assertEqual(self.controller.state['attempts'], 1)
        self.assertIn('New observation', self.controller.state['reason'])
        self.assertFalse(self.runtime.allow_motion)

    def test_legacy_confirm_reset_also_records_explicit_continue(self):
        self.model.decide.side_effect = [PROGRAM, dict(kind='finish', reason='Mock completion')]
        self.controller.confirm_reset(self.controller.state['session_id'], True, 'Continue', 1)
        self.wait()
        payload = json.loads(self.model.decide.call_args_list[-2].args[0][1]['content'][0]['text'])
        self.assertTrue(payload['experiment']['operator_continuation']['applies_to_this_generation'])
        self.assertEqual(payload['experiment']['operator_continuation']['readiness'], 'confirmed')

    def test_automatic_return_does_not_invent_operator_decision(self):
        self.assertNotIn('operator_continuation', self.controller.state)
        self.assertTrue(self.controller._restore_initial(automatic=True))
        self.assertNotIn('operator_continuation', self.controller.state)

