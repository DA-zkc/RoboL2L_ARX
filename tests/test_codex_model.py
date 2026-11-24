import json
import os
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from robol2l_deploy.cap.codex_model import CodexModel, cli_environment, request_failure
from robol2l_deploy.cap.model import FeedbackFixture
from robol2l_deploy.cap.validation import run_validation


class CodexModelTests(unittest.TestCase):
    def test_existing_chatgpt_login_and_isolated_generation(self):
        calls = []

        def invoke(command, **kwargs):
            calls.append((command, kwargs))
            if command[1:3] == ['login', 'status']:
                return SimpleNamespace(returncode=0, stdout='', stderr='Logged in using ChatGPT')
            if command[1:] == ['--version']:
                return SimpleNamespace(returncode=0, stdout='codex-cli test', stderr='')
            response = Path(command[command.index('--output-last-message') + 1])
            response.write_text(json.dumps({'kind': 'assist', 'reason': 'test', 'source': ''}))
            event = {'type': 'turn.completed', 'usage': {'input_tokens': 10, 'output_tokens': 5}}
            return SimpleNamespace(returncode=0, stdout=json.dumps(event), stderr='')

        with patch('shutil.which', return_value='/installed/codex'), patch('subprocess.run', side_effect=invoke):
            model = CodexModel('gpt-6-astra')
            result = model.decide([{'role': 'system', 'content': 'Test only.'}])
        self.assertEqual(result, {'kind': 'assist', 'reason': 'test'})
        command, arguments = calls[-1]
        self.assertEqual(command[command.index('--sandbox') + 1], 'read-only')
        self.assertIn('--ignore-user-config', command)
        self.assertIn('--ephemeral', command)
        self.assertNotIn('shell', arguments)
        self.assertIn('shell_tool', command)
        self.assertIn('multi_agent', command)
        self.assertEqual(model.requests[0]['usage']['output_tokens'], 5)
        self.assertFalse(model.settings['api_key_used'])
        self.assertFalse(Path(arguments['cwd']).exists())

    def _run_decide(self, extra_fields, answer):
        """Run one decide() against a fake CLI; return (result, schema the CLI was given)."""
        seen = {}
        def invoke(command, **kwargs):
            if command[1:3] == ['login', 'status']:
                return SimpleNamespace(returncode=0, stdout='', stderr='Logged in using ChatGPT')
            if command[1:] == ['--version']:
                return SimpleNamespace(returncode=0, stdout='codex-cli test', stderr='')
            seen['schema'] = json.loads(Path(command[command.index('--output-schema') + 1]).read_text())
            Path(command[command.index('--output-last-message') + 1]).write_text(json.dumps(answer))
            return SimpleNamespace(returncode=0, stdout=json.dumps({'type': 'turn.completed', 'usage': {}}), stderr='')
        with patch('shutil.which', return_value='/installed/codex'), patch('subprocess.run', side_effect=invoke):
            model = CodexModel('gpt-6-astra', extra_fields=extra_fields)
            return model.decide([{'role': 'system', 'content': 'Test only.'}]), seen['schema']

    def test_experiment_record_fields_reach_the_model_and_come_back(self):
        # The output schema forbids extra properties, so PRE-A/PRE-B/POST must be
        # declared on it or Codex can never return them. "" means "not provided".
        answer = {'kind': 'program', 'reason': 'r', 'source': 'x = 1', 'pre_a': 'root cause', 'pre_b': '', 'post': ''}
        result, schema = self._run_decide(('pre_a', 'pre_b', 'post'), answer)
        self.assertEqual(result, {'kind': 'program', 'reason': 'r', 'source': 'x = 1', 'pre_a': 'root cause'})
        self.assertFalse(schema['additionalProperties'])
        self.assertEqual(set(schema['properties']), {'kind', 'reason', 'source', 'pre_a', 'pre_b', 'post'})
        self.assertEqual(set(schema['required']), set(schema['properties']))

    def test_default_schema_is_unchanged_so_other_callers_are_unaffected(self):
        answer = {'kind': 'program', 'reason': 'r', 'source': 'x = 1'}
        result, schema = self._run_decide((), answer)
        self.assertEqual(result, answer)
        self.assertEqual(set(schema['properties']), {'kind', 'reason', 'source'})
        self.assertEqual(set(schema['required']), {'kind', 'reason', 'source'})

    def test_invalid_record_field_names_are_rejected(self):
        with patch('shutil.which', return_value='/installed/codex'):
            with self.assertRaisesRegex(ValueError, 'invalid_codex_schema_fields'):
                CodexModel('gpt-6-astra', extra_fields=('not valid!',))

    def test_timeout_is_configurable_and_bounded(self):
        from robol2l_deploy.cap.codex_model import configuration
        with patch.dict(os.environ, {}, clear=False):
            os.environ.pop('ROBOL2L_CODEX_TIMEOUT_S', None)
            self.assertEqual(configuration()['timeout_s'], 600)
        for given, expected in (('900', 900), ('5', 30), ('999999', 3600), ('not-a-number', 600)):
            with patch.dict(os.environ, {'ROBOL2L_CODEX_TIMEOUT_S': given}):
                self.assertEqual(configuration()['timeout_s'], expected, given)

    def test_child_does_not_inherit_api_secrets_or_provider_overrides(self):
        with patch.dict(os.environ, {
            'OPENAI_API_KEY': 'secret', 'CODEX_API_KEY': 'secret',
            'OPENAI_BASE_URL': 'https://unrelated.example', 'HOME': '/home/user',
        }):
            environment = cli_environment()
        self.assertNotIn('OPENAI_API_KEY', environment)
        self.assertNotIn('CODEX_API_KEY', environment)
        self.assertNotIn('OPENAI_BASE_URL', environment)
        self.assertEqual(environment['HOME'], '/home/user')

    def test_api_key_login_is_not_treated_as_codex_plan_usage(self):
        status = SimpleNamespace(returncode=0, stdout='Logged in using an API key', stderr='')
        with patch('shutil.which', return_value='/installed/codex'), patch('subprocess.run', return_value=status):
            with self.assertRaisesRegex(ValueError, 'chatgpt_login_required'):
                CodexModel('gpt-6-astra')

    def test_cli_errors_expose_only_classified_codes(self):
        for message, expected in (
            ('You have hit your usage limit. Authorization: secret', 'codex_usage_limit_reached'),
            ('401 Unauthorized; token=secret', 'codex_chatgpt_login_required'),
            ('This model requires a newer version of Codex', 'codex_upgrade_required_for_model'),
            ('unexpected diagnostic containing secret', 'codex_request_failed'),
        ):
            self.assertEqual(request_failure(message), expected)

    def test_quota_error_survives_session_feedback_without_dispatch(self):
        from robol2l_deploy.adapters.mock import MockAdapter
        from robol2l_deploy.runtime import StationRuntime
        from robol2l_deploy.cap.session import CapSession
        from robol2l_deploy.cap.tasks import MockReachTask
        model = SimpleNamespace(label='codex_cli_chatgpt')
        def decide(messages):raise RuntimeError('codex_usage_limit_reached')
        model.decide = decide
        with tempfile.TemporaryDirectory() as tmp:
            runtime = StationRuntime(MockAdapter(), Path(tmp) / 'runtime', allow_motion=True)
            try:
                state = CapSession(runtime, MockReachTask(), model, Path(tmp) / 'session').run()
                self.assertEqual(state['reason'], 'codex_usage_limit_reached')
                self.assertIn('额度', state['message_zh'])
                self.assertEqual(runtime.adapter.executions, 0)
            finally:runtime.close()

    def test_validation_separates_injected_program_from_model_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = run_validation(FeedbackFixture(), Path(tmp) / 'validation')
        self.assertTrue(report['passed'], report)
        self.assertNotIn('injected_test_program_not_model_generated', report['cases']['generation']['decision_origins'])
        self.assertEqual(report['cases']['repair']['decision_origins'][0], 'injected_test_program_not_model_generated')
        self.assertEqual(report['cases']['repair']['decision_origins'][1:], ['model_request', 'model_request'])
        self.assertFalse(report['physical_task_success'])


if __name__ == '__main__':
    unittest.main()
