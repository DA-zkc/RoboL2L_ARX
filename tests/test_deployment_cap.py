import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.runtime import StationRuntime
from robol2l_deploy.cap.program import ProgramExecutor
from robol2l_deploy.cap.tasks import MockReachTask,prepare_pick_place
from robol2l_deploy.cap.model import FeedbackFixture,ConfiguredModel
from robol2l_deploy.cap.session import CapSession


class CapTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.adapter=MockAdapter();self.runtime=StationRuntime(self.adapter,self.root/'runtime',allow_motion=True)
        self.task=MockReachTask();self.executor=ProgramExecutor(self.runtime,self.task.context)
        self.index=0
    def tearDown(self):self.runtime.close();self.temp.cleanup()
    def execute(self,source,**kwargs):
        self.index+=1;return self.executor.execute(source,self.root/f'program-{self.index}',**kwargs)
    def test_robol2l_format_functions_math_types_print(self):
        result=self.execute('''import math
def distance(v):
    return math.sqrt(sum(x*x for x in v))
state = get_robot_state()
p = Pose([0, 0, 0.31], state.end_effector_poses["primary"].quaternion_wxyz, state.base_frame)
print("CHECKPOINT norm", distance(p.position))
result = {"arms": state.arms, "embodiment": state.embodiment, "norm": distance(p.position)}''')
        self.assertTrue(result['ok'],result);self.assertEqual(result['result']['arms'],['primary'])
        self.assertIn('CHECKPOINT norm 0.31',result['stdout'])

    def test_public_array_reductions_in_isolated_shared_executor(self):
        result=self.execute('q=get_robot_state().joint_positions["primary"]\nresult=[float(q.mean()),float(q.min()),float(q.max()),float(q.sum())]')
        self.assertTrue(result['ok'],result);self.assertEqual(result['result'],[0.,0.,0.,0.])
        self.assertFalse(self.execute('import numpy\nresult=0')['ok'])
    def test_validation_rejects_before_any_call(self):
        for tail in ['import os','open("secret")','s.__class__','s.joint_positions["primary"][0] = 2','return 3']:
            source='s = get_robot_state()\n'+tail
            result=self.execute(source)
            self.assertFalse(result['ok'])
            # Undefined open is a runtime name error, never an actual file operation.
            if 'open(' not in tail and '[0] = 2' not in tail:self.assertEqual(result['calls'],[])
        self.assertEqual(self.adapter.executions,0)
    def test_public_arrays_cannot_call_host_methods(self):
        result=self.execute('s = get_robot_state()\ns.joint_positions["primary"].tofile("bad")\nresult = True')
        self.assertFalse(result['ok']);self.assertEqual(result['error']['line'],2)
        self.assertFalse(Path('bad').exists())
    def test_partial_stdout_line_and_call_failure_survive(self):
        result=self.execute('print("before")\ns = get_robot_state()\nresult = s["q"]')
        self.assertFalse(result['ok']);self.assertEqual(result['error']['type'],'TypeError')
        self.assertEqual(result['error']['line'],3);self.assertEqual(result['stdout'],'before\n')
        self.assertEqual(result['calls'][0]['line'],2)
    def test_typed_planner_failure_feedback(self):
        result=self.execute('p = plan_motion(Pose([0,0,.31],[1,0,0,0],"base"), strategy=MotionStrategy())\nresult = {"ok": p.ok, "code": p.error.code}')
        self.assertTrue(result['ok']);self.assertFalse(result['calls'][0]['ok']);self.assertEqual(result['result']['code'],'unsupported')
    def test_program_resource_limits(self):
        for source in ['while True:\n    pass','result = "x" * 100000000','result = list(range(100000000))','def f():\n    return f()\nresult = f()']:
            result=self.execute(source);self.assertFalse(result['ok']);self.assertEqual(result['error']['code'],'host_stopped')
        self.assertEqual(self.adapter.executions,0)
    def test_try_except_cannot_swallow_host_stop(self):
        self.executor.max_calls=1
        result=self.execute('try:\n    a=get_robot_state()\n    b=get_robot_state()\nexcept Exception:\n    result=True')
        self.assertFalse(result['ok']);self.assertEqual(len(result['calls']),1)
    def test_pause_is_dispatch_boundary(self):
        result=self.execute('set_gripper(.5)\nset_gripper(.7)\nresult=True',should_stop=lambda:self.adapter.executions>=1)
        self.assertFalse(result['ok']);self.assertEqual(self.adapter.executions,1);self.assertEqual(self.adapter.gripper,.5)
    def test_physical_calls_block_even_if_runtime_motion_enabled(self):
        self.adapter.synthetic=False
        result=self.execute('set_gripper(.5)\nresult=True')
        self.assertFalse(result['ok']);self.assertEqual(self.adapter.executions,0)
        self.assertEqual(result['error']['message'],'physical_motion_not_enabled_for_cap')
    def test_program_owns_station_for_all_calls(self):
        entered=threading.Event();release=threading.Event();original=self.adapter.get_robot_state
        def blocked():entered.set();release.wait(2);return original()
        self.adapter.get_robot_state=blocked
        thread=threading.Thread(target=lambda:self.execute('result=get_robot_state()'));thread.start()
        self.assertTrue(entered.wait(1))
        with self.assertRaises(Exception):self.runtime.submit('manual','set_gripper',{'position':.5})
        release.set();thread.join(2);self.assertFalse(thread.is_alive())
    def test_feedback_driven_repair_and_independent_verification(self):
        session=CapSession(self.runtime,self.task,FeedbackFixture(),self.root/'session')
        state=session.run();self.assertEqual(state['status'],'completed')
        self.assertEqual([v['program_ok'] for v in state['versions']],[False,True])
        self.assertEqual(session.history[0]['feedback']['error']['line'],2)
        self.assertTrue(state['verification']['passed']);self.assertFalse(state['physical_task_success'])
        self.assertEqual(self.adapter.executions,2)
        with self.assertRaises(ValueError):session.run()
        self.assertTrue((self.root/'session/v01/source.py').exists())
    def test_false_finish_claim_and_malformed_response_bounded(self):
        class Model:
            label='fixture'
            def decide(self,messages):return {'kind':'finish','reason':'I say success'}
        state=CapSession(self.runtime,self.task,Model(),self.root/'session',max_turns=2).run()
        self.assertEqual(state['status'],'waiting_operator');self.assertEqual(state['turns'],1)
        self.assertEqual(self.adapter.executions,0)
    def test_independent_check_rejects_changed_orientation(self):
        from robol2l_deploy.contracts import Pose
        self.adapter.pose=Pose([0,0,.31],[0,1,0,0],'base')
        self.adapter.gripper=.5
        self.assertFalse(self.task.verify(self.runtime)['passed'])
    def test_feedback_references_frozen_observation(self):
        self.runtime.get_observation()
        observation_id=next(reversed(self.runtime.observations))
        result=self.execute('state=get_robot_state()\nresult=(state.base_frame, True)')
        self.assertTrue(result['ok'],result)
        self.assertEqual(result['observation_id'],observation_id)
        self.assertEqual(result['calls'][0]['observation_id'],observation_id)
    def test_provider_errors_do_not_leak_secrets(self):
        class Model:
            label='fixture'
            def decide(self,messages):raise RuntimeError('SECRET_VALUE')
        session=CapSession(self.runtime,self.task,Model(),self.root/'session');state=session.run()
        self.assertEqual(state['status'],'error')
        self.assertNotIn('SECRET_VALUE',(self.root/'session/session.json').read_text())
    def test_openai_credential_cannot_go_to_old_proxy(self):
        with self.assertRaisesRegex(ValueError,'official'):
            ConfiguredModel(dict(base_url='https://example.com/v1',model='gpt-6-astra',reasoning='medium',api_key_env='OPENAI_API_KEY'))
    def test_real_model_transport_shape_uses_only_environment_secret(self):
        import io
        from franka_player.runtime import request_decision
        captured=[]
        def urlopen(req,timeout):
            captured.append(req)
            return io.BytesIO(json.dumps({'choices':[{'message':{'content':json.dumps({'kind':'assist','reason':'test'})}}]}).encode())
        with patch.dict('os.environ',{'OPENAI_API_KEY':'test-secret'}),patch('urllib.request.urlopen',urlopen):
            config=dict(base_url='https://api.openai.com/v1',model='gpt-6-astra',reasoning='medium',api_key_env='OPENAI_API_KEY')
            self.assertEqual(ConfiguredModel(config).decide([])['kind'],'assist')
        self.assertEqual(captured[0].full_url,'https://api.openai.com/v1/chat/completions')
        self.assertNotIn(b'test-secret',captured[0].data)
    def test_preparation_has_no_coordinates_or_motion(self):
        draft=prepare_pick_place();self.assertFalse(draft['motion_enabled'])
        self.assertIn('not_ready',draft['status']);self.assertEqual(self.adapter.executions,0)

class CapIntegrationTests(unittest.TestCase):
    def test_configured_transport_runs_feedback_loop_against_local_fixture(self):
        from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
        from robol2l_deploy.cap.model import FeedbackFixture
        fixture=FeedbackFixture();calls=[]
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_POST(self):
                payload=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                calls.append({'model':payload['model'],'authenticated':self.headers.get('Authorization')=='Bearer fixture-secret'})
                response={'choices':[{'message':{'content':json.dumps(fixture.decide(payload['messages']))}}]}
                body=json.dumps(response).encode();self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler);worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        try:
            with tempfile.TemporaryDirectory() as tmp,patch.dict('os.environ',{'TEST_CAP_KEY':'fixture-secret'}):
                root=Path(tmp);runtime=StationRuntime(MockAdapter(),root/'runtime',allow_motion=True)
                model=ConfiguredModel(dict(base_url=f'http://127.0.0.1:{server.server_port}/v1',model='test-fixture',reasoning='none',api_key_env='TEST_CAP_KEY'))
                try:
                    state=CapSession(runtime,MockReachTask(),model,root/'session').run()
                    self.assertEqual(state['status'],'completed');self.assertEqual(len(calls),3);self.assertTrue(all(c['authenticated'] for c in calls))
                    for path in (root/'session').rglob('*'):
                        if path.is_file():self.assertNotIn('fixture-secret',path.read_text())
                finally:runtime.close()
        finally:server.shutdown();server.server_close();worker.join(2)

    def test_web_start_feedback_and_physical_rejection(self):
        from urllib.request import Request,urlopen
        from urllib.error import HTTPError
        from franka_player.server import Server,handler
        from franka_player.core import Console
        from robol2l_deploy.web import hardware_handler
        import time
        with tempfile.TemporaryDirectory() as tmp:
            runtime=StationRuntime(MockAdapter(),Path(tmp)/'runtime',allow_motion=True)
            server=Server(('127.0.0.1',0),hardware_handler(handler(Console(Path(tmp)/'console')),runtime))
            worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
            def request(route,body=None):
                req=Request(f'http://127.0.0.1:{server.server_port}/api/hardware/cap/'+route,
                            data=None if body is None else json.dumps(body).encode(),headers={'X-Console-Request':'1'})
                with urlopen(req,timeout=5) as response:return json.load(response)
            try:
                self.assertEqual(request('start',{'model_mode':'fixture'})['status'],'accepted')
                for _ in range(100):
                    state=request('state')
                    if state['status'] not in ('ready','running'):break
                    time.sleep(.02)
                self.assertEqual(state['status'],'completed');self.assertEqual(len(state['recent_feedback']),2)
                self.assertEqual(state['recent_feedback'][0]['feedback']['error']['line'],2)
                self.assertFalse(request('prepare',{})['motion_enabled'])
                runtime.adapter.synthetic=False
                with self.assertRaises(HTTPError) as error:request('start',{'model_mode':'fixture'})
                self.assertEqual(error.exception.code,409)
                with self.assertRaises(HTTPError) as error:request('start',{'model_mode':'fixture','task':'visual'})
                self.assertEqual(error.exception.code,409)
            finally:server.shutdown();server.server_close();runtime.close();worker.join(2)

    def test_restart_marks_interruption_without_replay(self):
        from robol2l_deploy.cap.session import CapController
        from robol2l_deploy.cap.storage import write_json
        with tempfile.TemporaryDirectory() as tmp:
            runtime=StationRuntime(MockAdapter(),Path(tmp)/'runtime',allow_motion=True)
            folder=runtime.directory/'cap'/'old';folder.mkdir(parents=True)
            write_json(folder/'session.json',{'status':'running','versions':[]})
            try:
                controller=CapController(runtime)
                self.assertEqual(json.loads((folder/'session.json').read_text())['status'],'interrupted')
                self.assertEqual(controller.snapshot()['status'],'idle');self.assertEqual(runtime.adapter.executions,0)
            finally:runtime.close()
