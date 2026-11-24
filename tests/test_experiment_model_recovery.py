"""Bounded review evidence and recovery without physical dispatch."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.runtime import StationRuntime
from robol2l_deploy.cap.experiment import ExperimentController
from robol2l_deploy.cap.experiment_models import ExperimentAPIModel, ModelRequestError
from robol2l_deploy.cap.program import ProgramExecutor
from robol2l_deploy.cap.prompt_history import HISTORY_BYTES, prompt_history, size


class PromptHistoryTests(unittest.TestCase):
    def test_poll_storm_keeps_failures_and_outcome_without_mutating_evidence(self):
        call = dict(tool='get_robot_state', ok=True, output={'joint_positions': {'left': [0.] * 6},
                                                           'padding': 'x' * 2000})
        calls = [copy.deepcopy(call) for _ in range(3977)]
        calls.insert(1800, dict(tool='execute_trajectory', ok=False,
                               output={'ok': False, 'error': {'code': 'outcome_unknown', 'recoverable': False}}))
        history = [dict(version='v07', source='result = get_robot_state()', feedback=dict(
            ok=True, result={'status': '初始状态未稳定'}, calls=calls, physical_task_success=False))]
        before = json.dumps(history)
        result = prompt_history(history)
        self.assertEqual(json.dumps(history), before)
        self.assertLess(size(result), 24_000)
        feedback = result[0]['feedback']
        self.assertEqual(feedback['result']['status'], '初始状态未稳定')
        self.assertFalse(feedback['physical_task_success'])
        self.assertEqual(feedback['call_summary']['total'], 3978)
        self.assertEqual(feedback['call_summary']['error_counts'], {'outcome_unknown': 1})
        self.assertIn('outcome_unknown', json.dumps(feedback['calls']))

    def test_large_nested_outputs_and_unicode_respect_total_budget(self):
        history = [dict(version=f'v{i}', feedback=dict(ok=False,
            error={'code': 'controller_failed', 'message': '故障' * 10_000},
            result=[{'data': '中文\\\n' * 5000}] * 20, calls=[])) for i in range(80)]
        result = prompt_history(history)
        self.assertLessEqual(size(result), HISTORY_BYTES)
        self.assertEqual(result[-1]['version'], 'v79')
        self.assertIn('controller_failed', json.dumps(result[-1]))
        self.assertIn('_omitted_older_entries', result[0])


class ModelErrorTests(unittest.TestCase):
    def model(self):
        return ExperimentAPIModel(dict(provider='openai', reasoning='none', profiles={
            'openai': dict(base_url='https://example.test/v1', model='test', api_key='private-test-key')}))

    def test_context_rejection_is_classified_without_storing_provider_secrets(self):
        response = Mock(ok=False, is_redirect=False, status_code=400)
        response.json.return_value = {'error': {'code': 'context_length_exceeded',
            'message': 'maximum context length; private-test-key https://secret.invalid?token=private-test-key'}}
        with patch('requests.post', return_value=response) as post:
            with self.assertRaises(ModelRequestError) as caught:
                self.model().decide([{'role': 'user', 'content': 'test'}])
        error = caught.exception
        self.assertEqual(error.diagnostics['category'], 'context_limit')
        self.assertGreater(error.diagnostics['request_bytes'], 0)
        self.assertFalse(error.diagnostics['retryable'])
        self.assertNotIn('private-test-key', str(error) + json.dumps(error.diagnostics))
        self.assertEqual(post.call_count, 1)  # Never repeat a rejected request blindly.

    def test_non_json_error_is_safe(self):
        response = Mock(ok=False, is_redirect=False, status_code=502)
        response.json.side_effect = ValueError('secret html')
        with patch('requests.post', return_value=response), self.assertRaises(ModelRequestError) as caught:
            self.model().decide([])
        self.assertEqual(caught.exception.diagnostics['category'], 'provider_unavailable')
        self.assertTrue(caught.exception.diagnostics['retryable'])


class ReviewRetryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.runtime = StationRuntime(MockAdapter(), Path(self.tmp.name) / 'runtime')
        self.fail_review = True
        owner = self
        class Model:
            label = 'test_review_failure'
            def decide(self, messages):
                context = json.loads(messages[-1]['content'][0]['text'])
                if context['experiment']['phase'] == 'review':
                    if owner.fail_review: raise ModelRequestError(400, 'context_limit', 123456)
                    return dict(kind='program', source='raise ValueError("MUST NOT EXECUTE")',
                                reason='incomplete', post='Recorded incomplete attempt reviewed.')
                return dict(kind='program', source='result = get_robot_state()', reason='read only')
        self.controller = ExperimentController(self.runtime, Model)
        self.controller.start('test', 'test criterion', True)
        self.wait()

    def wait(self):
        self.controller.thread.join(15)
        self.assertFalse(self.controller.busy())

    def tearDown(self):
        self.controller.stop.set()
        if self.controller.thread: self.controller.thread.join(15)
        self.runtime.close()
        self.tmp.cleanup()

    def test_retry_review_keeps_attempt_and_never_authorizes_or_returns(self):
        self.assertTrue(self.controller.snapshot()['review_retry_supported'])
        self.assertEqual(self.controller.state['model_error']['category'], 'context_limit')
        self.fail_review = False
        with patch.object(self.controller, '_authorize', side_effect=AssertionError('motion authorization')):
            self.controller.retry_review(self.controller.state['session_id'], 1)
            self.wait()
        state = self.controller.snapshot()
        self.assertEqual(state['status'], 'waiting_reset')
        self.assertEqual(state['attempts'], 1)
        self.assertEqual(state['versions'][-1]['post'], 'Recorded incomplete attempt reviewed.')
        self.assertIsNone(state['model_error'])
        self.assertEqual(self.runtime.adapter.executions, 0)
        self.assertFalse(self.runtime.allow_motion)

    def test_last_review_exhausts_budget_without_ninth_program(self):
        self.controller.state['attempts'] = 8
        self.fail_review = False
        self.controller.retry_review(self.controller.state['session_id'], 8)
        self.wait()
        self.assertEqual(self.controller.state['status'], 'exhausted')
        self.assertEqual(self.controller.state['attempts'], 8)
        self.assertEqual(self.runtime.adapter.executions, 0)

    def test_stale_fault_pause_and_device_change_reject_without_clearing(self):
        session = self.controller.state['session_id']
        with self.assertRaises(ValueError): self.controller.retry_review(session, 0)
        for field in ('faulted', 'paused', 'closed'):
            setattr(self.runtime, field, True)
            with self.assertRaises(ValueError): self.controller.retry_review(session, 1)
            self.assertTrue(getattr(self.runtime, field))
            setattr(self.runtime, field, False)
        self.controller.state['initial']['device_binding'] = 'different-device'
        with self.assertRaisesRegex(ValueError, 'device_changed'):
            self.controller.retry_review(session, 1)
        self.assertEqual(self.runtime.adapter.executions, 0)

    def test_state_polling_reads_fresh_states_without_project_pacing(self):
        report = ProgramExecutor(self.runtime, self.controller.context()).execute(
            'for i in range(5):\n    state = get_robot_state()\nresult = state',
            Path(self.tmp.name) / 'polls')
        self.assertTrue(report['ok'], report['error'])
        calls = report['calls']
        self.assertEqual(len(calls), 5)
        stamps = [call['output']['timestamp_s'] for call in calls]
        self.assertEqual(stamps, sorted(stamps))
        self.assertEqual(len(set(stamps)), 5)

    def test_live_snapshot_is_small_while_record_keeps_all_calls(self):
        feedback = self.controller.state['versions'][-1]['feedback']
        feedback['calls'] = feedback['calls'] * 3977
        self.controller.save()
        snapshot = self.controller.snapshot(summary=True)
        self.assertEqual(snapshot['versions'][-1]['feedback']['call_summary']['total'], 3977)
        self.assertLess(size(snapshot), 200_000)
        record = self.controller.record(self.controller.state['session_id'])
        self.assertEqual(len(record['versions'][-1]['feedback']['calls']), 3977)


if __name__ == '__main__': unittest.main()
