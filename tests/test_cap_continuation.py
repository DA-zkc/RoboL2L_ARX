"""Explicit continuation and recoverable feedback, using only mock hardware."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.runtime import StationRuntime
from robol2l_deploy.cap.session import CapController
from robol2l_deploy.cap.tasks import MockReachTask
from robol2l_deploy.cap.presentation import turns
from robol2l_deploy.errors import ApiError, ErrorCode

REACH = '''state = get_robot_state()
goal = Pose([0, 0, .31], state.end_effector_poses['primary'].quaternion_wxyz, 'base')
plan = plan_motion(goal)
if not plan.ok:
    result = plan
else:
    result = execute_trajectory(plan.trajectory)
'''
CLOSE = 'result = set_gripper(.5)'


def model(*decisions):
    remaining=iter(decisions)
    return SimpleNamespace(label='software_test',decide=lambda messages:next(remaining))


def program(source):return dict(kind='program',reason='Mock test',source=source)


class ContinuationTests(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory()
        self.runtime=StationRuntime(MockAdapter(),Path(self.temp.name)/'runtime',allow_motion=True)
        self.controller=CapController(self.runtime)
        self.task=MockReachTask()
    def tearDown(self):
        self.runtime.close();self.temp.cleanup()
    def join(self):
        self.controller.thread.join(10)
        self.assertFalse(self.controller.thread.is_alive())
        return self.controller.snapshot()

    def test_continue_after_reload_keeps_session_versions_and_context(self):
        accepted=self.controller.start(model(program(REACH),dict(kind='assist',reason='Mock assistance')),self.task)
        state=self.join();self.assertEqual(state['status'],'waiting_operator')
        directory=Path(state['session_directory'])
        originals={p:p.read_bytes() for p in directory.rglob('*') if p.is_file() and p.name not in ('session.json','history.json')}
        # Also exercises migration of sessions created before history.json existed.
        (directory/'history.json').unlink()
        self.controller=CapController(self.runtime)
        self.assertEqual(self.runtime.adapter.executions,1)  # No replay on reload.
        seen=[];decisions=iter([program(CLOSE),dict(kind='finish',reason='Mock done')])
        def decide(messages):
            seen.append(json.loads(messages[1]['content'][0]['text']))
            return next(decisions)
        result=self.controller.continue_session(SimpleNamespace(label='software_test',decide=decide),self.task,accepted['session'],'现场已确认，继续')
        final=self.join()
        self.assertEqual(final['status'],'completed')
        self.assertEqual(result['session'],accepted['session'])
        self.assertEqual([v['version'] for v in final['versions']],['v00','v01'])
        self.assertEqual(final['turns'],4)
        self.assertEqual(self.runtime.adapter.executions,2)
        self.assertTrue(any(item.get('source')==REACH for item in seen[0]['history']))
        self.assertEqual(seen[0]['history'][-1]['human_intervention']['message'],'现场已确认，继续')
        self.assertEqual(seen[0]['observation']['robot_state']['end_effector_poses']['primary']['position'],[0.,0.,.31])
        for path,original in originals.items():self.assertEqual(path.read_bytes(),original)
        self.assertEqual(len(final['continuations']),1)

    def test_paused_model_decision_is_not_replayed_or_mislabelled(self):
        def pause(messages):
            self.runtime.pause(True)
            return program(REACH)
        accepted=self.controller.start(SimpleNamespace(label='software_test',decide=pause),self.task)
        state=self.join();self.assertEqual(state['status'],'paused')
        self.assertEqual(self.runtime.adapter.executions,0)
        self.runtime.pause(False)
        self.controller.continue_session(model(program(REACH),dict(kind='assist',reason='Mock stop')),self.task,accepted['session'])
        state=self.join();rows=turns(Path(state['session_directory']),state)
        self.assertEqual(self.runtime.adapter.executions,1)
        self.assertIsNone(rows[0]['version']);self.assertEqual(rows[1]['version'],'v00')
        self.assertEqual(state['versions'][0]['turn'],1)

    def test_controller_fault_is_not_cleared_by_continue(self):
        accepted=self.controller.start(model(dict(kind='assist',reason='Mock stop')),self.task)
        self.join();self.runtime.faulted=True
        with self.assertRaisesRegex(ValueError,'runtime_fault_latched'):
            self.controller.continue_session(model(program(CLOSE)),self.task,accepted['session'])
        self.assertTrue(self.runtime.faulted);self.assertEqual(self.runtime.adapter.executions,0)

    def test_recoverable_collision_rejection_returns_to_model(self):
        original=self.runtime.adapter.plan_motion
        calls=[]
        def plan(goal):
            calls.append(goal)
            if len(calls)==1:raise ApiError(ErrorCode.PLANNING_FAILED,'mock collision')
            return original(goal)
        self.runtime.adapter.plan_motion=plan
        contexts=[];decisions=iter([program(REACH),program(REACH+'\nresult=set_gripper(.5)'),dict(kind='finish',reason='Mock done')])
        def decide(messages):
            contexts.append(json.loads(messages[1]['content'][0]['text']))
            return next(decisions)
        self.controller.start(SimpleNamespace(label='software_test',decide=decide),self.task)
        state=self.join();self.assertEqual(state['status'],'completed')
        feedback=contexts[1]['history'][-1]['feedback']
        self.assertEqual(feedback['calls'][-1]['output']['error']['message'],'mock collision')
        self.assertTrue(feedback['calls'][-1]['output']['error']['recoverable'])
        self.assertFalse(self.runtime.faulted)
        self.assertNotEqual(contexts[0]['observation_id'],contexts[1]['observation_id'])

    def test_http_continue_unpauses_and_keeps_original_task(self):
        import requests,threading,time
        from franka_player.core import Console
        from franka_player.server import Server,handler
        from robol2l_deploy.web import hardware_handler
        accepted=self.controller.start(model(dict(kind='assist',reason='Mock checkpoint')),self.task)
        self.join();self.runtime.pause(True)
        console=Console(Path(self.temp.name)/'console')
        server=Server(('127.0.0.1',0),hardware_handler(handler(console),self.runtime,cap_controller=self.controller))
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        client=requests.Session();client.trust_env=False
        base=f'http://127.0.0.1:{server.server_port}/api/hardware/cap/'
        headers={'X-Console-Request':'1'}
        try:
            response=client.post(base+'continue',json={'session':accepted['session'],'model_mode':'fixture','message':''},headers=headers,timeout=5)
            self.assertEqual(response.status_code,202,response.text)
            self.assertTrue(response.json()['continued'])
            final=self.join();self.assertEqual(final['status'],'completed')
            self.assertEqual(final['session_id'],accepted['session']);self.assertFalse(self.runtime.paused)
            self.assertEqual(client.post(base+'continue',json={'session':accepted['session'],'model_mode':'fixture','message':''},headers=headers,timeout=5).status_code,409)
        finally:
            server.shutdown();server.server_close();worker.join(3);console.shutdown();client.close()


if __name__=='__main__':unittest.main()
