import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from franka_player.core import Console
from franka_player.cap_pipeline import model_messages
from franka_cap.program import Program
from franka_cap.runtime import FrankaRuntime, READ_ONLY_TOOLS


class FakeSDK:
    def __init__(self, directory):
        self.run_dir=Path(directory);self.cfg={'station':'fr3-single','motion_enabled':False};self.events=[];self.reads=0
    def describe(self):return {'methods':{'get_robot_state()':'read'},'motion_enabled':False}
    def get_robot_state(self):self.reads+=1;return {'q':[0]*7}
    def close(self):pass


class FixtureRuntime(FrankaRuntime):
    def observe(self):return {'fresh':True,'source':'software_fixture','images':[]}
    def validation_evidence(self):
        return {'passed':self.sdk.reads>0,'checks':{'fixture_read':self.sdk.reads>0},'physical_task_success':False,'motion_sent':False}


class CapLoopTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.sdk=FakeSDK(self.tmp.name);self.runtime=FixtureRuntime(self.sdk)
        self.console=Console(Path(self.tmp.name)/'console',lambda cfg:self.runtime)
        self.console.create({'name':'green cube','goal':'put green cube in plate','success_criteria':'lift/place/release','mode':'live'})
        self.console.configure({'base_url':'http://127.0.0.1:9999/v1','model':'fixture','station':'fr3-single','reasoning':'none'})
        self.env=patch.dict('os.environ',{'ROBOT_MODEL_API_KEY':'fixture-not-secret'});self.env.start()
    def tearDown(self):
        self.console.shutdown();self.env.stop();self.tmp.cleanup()
    def run_loop(self,decisions):
        with patch('franka_player.core.decide_cap',side_effect=decisions) as model:
            self.console.start();self.console.worker.join(5)
            self.assertFalse(self.console.worker.is_alive())
        return model
    def test_error_feedback_corrected_then_offline_finish(self):
        seen=[]
        def model(config,exp,obs,history,contract,evidence):
            seen.append(json.loads(json.dumps(history)))
            if len(seen)==1:return {'kind':'program','reason':'bad syntax','source':'def run(api):\n    import os'}
            if len(seen)==2:
                self.assertFalse(history[-2]['value']['ok'])
                return {'kind':'program','reason':'read','source':'def run(api):\n    return api.get_robot_state()'}
            return {'kind':'finish','reason':'software loop only'}
        self.run_loop(model);exp=self.console._selected()
        self.assertEqual(exp['status'],'validated');self.assertEqual(exp['actions'],2)
        self.assertEqual(self.sdk.reads,1);self.assertFalse(exp['validation']['physical_task_success'])
        self.assertEqual(len(list(Path(self.tmp.name).glob('program-*/source.py'))),2)
        restored=Console(Path(self.tmp.name)/'console');self.assertEqual(restored._selected()['status'],'validated')
    def test_premature_completion_and_malformed_response_are_bounded_feedback(self):
        self.console.configure({'max_actions':3})
        self.run_loop([{'kind':'finish','reason':'unproven'}, {'kind':'complete','reason':'physical'}, {'kind':'finish','reason':'still unproven'}])
        exp=self.console._selected();self.assertEqual(exp['status'],'waiting_operator')
        self.assertEqual(exp['detail'],'cap_round_budget_reached');self.assertEqual(self.sdk.reads,0)
    def test_motion_rejected_before_any_read(self):
        result=self.runtime.execute_program('def run(api):\n api.get_robot_state()\n api.set_gripper(0)')
        self.assertFalse(result['ok']);self.assertEqual(self.sdk.reads,0)
    def test_pause_stops_between_program_calls(self):
        calls=[]
        result=self.runtime.execute_program('def run(api):\n api.get_robot_state()\n api.get_robot_state()',should_pause=lambda:self.sdk.reads>0)
        self.assertEqual(result['status'],'paused');self.assertEqual(self.sdk.reads,1)
    def test_pause_during_model_never_dispatches_and_resume_reobserves(self):
        entered=threading.Event();release=threading.Event()
        def model(*args):
            entered.set();release.wait(3)
            return {'kind':'program','reason':'read','source':'def run(api):\n return api.get_robot_state()'}
        with patch('franka_player.core.decide_cap',side_effect=model):
            self.console.start();self.assertTrue(entered.wait(3));self.console.pause();release.set();self.console.worker.join(5)
        self.assertEqual(self.console._selected()['status'],'paused');self.assertEqual(self.sdk.reads,0)
        with patch('franka_player.core.decide_cap',return_value={'kind':'assist','reason':'operator'}) as model:
            self.console.start(resume=True);self.console.worker.join(5)
        self.assertEqual(model.call_args.args[2]['source'],'software_fixture')
    def test_context_keeps_feedback_but_not_historical_base64(self):
        exp=self.console._selected();obs={'id':'new','images':[{'name':'global','data_url':'data:image/jpeg;base64,CURRENT'}]}
        history=[{'kind':'observation','value':{'images':[{'data_url':'OLD_SECRET_BLOB'}]}},{'kind':'program_result','value':{'error':'depth unavailable'}}]
        msg=model_messages(exp,obs,history,{},{});text=json.dumps(msg)
        self.assertIn('CURRENT',text);self.assertNotIn('OLD_SECRET_BLOB',text);self.assertIn('depth unavailable',text)

if __name__=='__main__':unittest.main()

class EvidenceTests(unittest.TestCase):
    def test_model_return_value_cannot_forge_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            sdk=FakeSDK(tmp);runtime=FrankaRuntime(sdk)
            result=runtime.execute_program('def run(api):\n return {"passed": True, "physical_task_success": True}')
            self.assertTrue(result['ok']);self.assertFalse(runtime.validation_evidence()['passed'])
            self.assertFalse(runtime.validation_evidence()['physical_task_success'])
    def test_nonfinite_constants_never_reach_sdk(self):
        with tempfile.TemporaryDirectory() as tmp:
            sdk=FakeSDK(tmp);runtime=FrankaRuntime(sdk)
            result=runtime.execute_program('def run(api):\n api.get_robot_state()\n return 1e999')
            self.assertFalse(result['ok']);self.assertEqual(sdk.reads,0)
