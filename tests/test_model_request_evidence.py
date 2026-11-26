"""The recorded boundary is the actual model input, including correction retries."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from robol2l_deploy.cap.experiment_models import ExperimentAPIModel
from robol2l_deploy.cap.request_evidence import record_model_call, request_digest
import test_experiment_pipeline as pipeline


class RequestEvidenceTests(unittest.TestCase):
    def test_api_input_matches_intercepted_transport_without_credentials_or_image_bytes(self):
        messages=[dict(role='system',content='工具说明'),dict(role='user',content=[
            dict(type='text',text='{"history":[],"observation_id":"frozen-frame"}'),
            dict(type='image_url',image_url=dict(url='data:image/jpeg;base64,fixture',detail='high'))])]
        before=deepcopy(messages)
        model=ExperimentAPIModel(dict(provider='openai',reasoning='none',profiles={'openai':dict(
            base_url='https://example.test/v1',model='fixture',api_key='private-test-key')}))
        response=Mock(ok=True,is_redirect=False,status_code=200)
        response.json.return_value={'choices':[{'message':{'content':'{"kind":"finish","reason":"test"}'}}]}
        with tempfile.TemporaryDirectory() as tmp, patch('requests.post',return_value=response) as post:
            result=record_model_call(tmp,model,messages,phase='review',attempts=2,retry=1)
            record=json.loads(next(Path(tmp).glob('model-requests/*.json')).read_text())
            self.assertEqual(record['messages_sha256'],request_digest(post.call_args.kwargs['json']['messages']))
            self.assertEqual(record['messages'][0],messages[0])
            self.assertEqual(record['messages'][1]['content'][0],messages[1]['content'][0])
            image=record['messages'][1]['content'][1]['image_url']
            self.assertEqual(image['url_sha256'],hashlib.sha256(messages[1]['content'][1]['image_url']['url'].encode()).hexdigest())
            self.assertEqual(image['detail'],'high')
            self.assertNotIn('private-test-key',json.dumps(record))
            self.assertNotIn('base64,fixture',json.dumps(record))
            self.assertEqual(record['response'],result)
            self.assertEqual(record['status'],'returned')
        self.assertEqual(messages,before)

    def test_failure_keeps_input_without_logging_exception_secret(self):
        model=Mock(settings={'api_key':'private-key'})
        model.decide.side_effect=RuntimeError('private-key provider text')
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(RuntimeError):
                record_model_call(tmp,model,[],phase='generate',attempts=0,retry=1)
            record=json.loads(next(Path(tmp).glob('model-requests/*.json')).read_text())
            self.assertEqual(record['status'],'error')
            self.assertEqual(record['error_type'],'RuntimeError')
            self.assertNotIn('private-key',json.dumps(record))


class RequestRetryEvidenceTests(unittest.TestCase):
    def test_each_static_correction_request_is_recorded_before_call_without_dispatch(self):
        fixture=pipeline.ExperimentTests();fixture.setUp()
        try:
            fixture.start()
            fixture.model.decide=Mock(side_effect=[
                dict(kind='program',reason='bad syntax',source='if :'),
                dict(kind='finish',reason='done')])
            result=fixture.controller._decide(fixture.model,fixture.runtime.get_observation(),'generate')
            records=[json.loads(p.read_text()) for p in fixture.controller.folder.glob('model-requests/*.json')]
            records.sort(key=lambda r:r['retry'])
            self.assertEqual([r['retry'] for r in records],[1,2])
            self.assertNotEqual(records[0]['messages_sha256'],records[1]['messages_sha256'])
            self.assertIn('validation_error',records[1]['messages'][-1]['content'][-1]['text'])
            self.assertEqual(records[1]['response'],result)
            self.assertEqual(fixture.runtime.adapter.executions,0)
        finally:fixture.tearDown()
