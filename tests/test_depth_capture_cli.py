"""Camera trace CLI uses only get_observation and refuses legacy processes."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from tools import capture_rgbd_stages as cli


class CaptureCLITests(unittest.TestCase):
    def fixture(self, root, *, supported):
        metadata=root/'.franka-player/panel/sessions/fixture/hardware/observation-test/metadata.json'
        request=root/'.franka-player/rgbd-depth-traces/fixture/request.json'
        metadata.parent.mkdir(parents=True)
        metadata.write_text(json.dumps(dict(measurement_diagnostics={'right_wrist':dict(
            serial='fixture',depth_capture=dict(supported=supported,request_file=str(request)))})))
        client=Mock();session=dict(session_id='fixture',busy=False,cap_running=False,closed=False)
        def get(url, **kwargs):
            if url.endswith('/session'):
                if request.exists():
                    value=json.loads(request.read_text())
                    for i in range(value['frames']):
                        output=request.parent/value['id']/str(i)
                        output.mkdir(parents=True,exist_ok=True)
                        (output/'metadata.json').write_text('{}')
                return Mock(json=lambda:session)
            if url.endswith('/state'):
                job_id=client.post.call_args.kwargs['json']['request_id']
                return Mock(json=lambda:dict(session_id='fixture',jobs={job_id:dict(
                    status='completed',observation_id='test')}))
            raise AssertionError(url)
        client.get.side_effect=get
        return client,request

    def test_legacy_owner_fails_without_request_file(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);client,request=self.fixture(root,supported=False)
            with patch.object(cli,'ROOT',root),patch.object(cli.requests,'Session',return_value=client):
                with self.assertRaisesRegex(RuntimeError,'no raw-depth capture hook'):
                    cli.capture('right_wrist',2)
            self.assertFalse(request.exists())
            self.assertEqual(client.post.call_count,1)
            self.assertEqual(client.post.call_args.kwargs['json']['operation'],'get_observation')

    def test_loaded_owner_bounded_request_is_removed_after_completion(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);client,request=self.fixture(root,supported=True)
            with patch.object(cli,'ROOT',root),patch.object(cli.requests,'Session',return_value=client):
                result=cli.capture('right_wrist',2)
            self.assertEqual(result['frames'],2)
            self.assertFalse(request.exists())
            self.assertEqual(client.post.call_count,1)
            self.assertEqual(client.post.call_args.kwargs['json']['operation'],'get_observation')


if __name__=='__main__':unittest.main()
