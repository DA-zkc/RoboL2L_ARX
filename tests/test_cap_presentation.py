"""Status explanations and pre-command rejection; no hardware or model calls."""
import unittest
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from pathlib import Path
import json
from robol2l_deploy.cap.presentation import summary, failure, turns, turn_detail
from robol2l_deploy.execution_feedback import motion_not_started
from robol2l_deploy.runtime import StationRuntime, OutcomeUnknown
from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.errors import ErrorCode

REJECTED = dict(driver='franky', cause_type='RuntimeError',
                cause_message='franky_trajectory_start_state_changed', control_trace={'samples':0})

class CapPresentationTest(unittest.TestCase):
    def test_zero_commands_required_for_retryable_rejection(self):
        self.assertTrue(motion_not_started(REJECTED))
        self.assertFalse(motion_not_started(dict(REJECTED,control_trace={'samples':1})))
        self.assertFalse(motion_not_started(dict(REJECTED,cause_type='ControlException')))
        with TemporaryDirectory() as directory:
            runtime = StationRuntime(MockAdapter(),directory,allow_motion=True)
            def reject(): raise OutcomeUnknown('native init failed',diagnostics=REJECTED)
            result = runtime._dispatch('execute_trajectory',reject)
            self.assertEqual(result.error.code,ErrorCode.STALE_PLAN)
            self.assertEqual(result.steps_executed,0)
            self.assertFalse(runtime.faulted)
            runtime.close()

    def test_old_rejection_keeps_evidence_without_false_fault(self):
        with TemporaryDirectory() as directory:
            path=Path(directory)/'events.jsonl'
            events=[{'kind':'dispatch'},{'kind':'outcome_unknown','diagnostics':REJECTED}]
            path.write_text('\n'.join(json.dumps(e) for e in events)+'\n')
            runtime=StationRuntime(MockAdapter(),directory)
            self.assertFalse(runtime.faulted)
            self.assertTrue(runtime.last_execution['can_continue'])
            runtime.close()
            # An earlier true fault must survive a later harmless rejection.
            events.insert(0,{'kind':'outcome_unknown','diagnostics':dict(REJECTED,cause_type='ControlException')})
            path.write_text('\n'.join(json.dumps(e) for e in events)+'\n')
            runtime=StationRuntime(MockAdapter(),directory)
            self.assertTrue(runtime.faulted)
            runtime.close()

    def test_assist_uses_model_reason_not_previous_corrected_error(self):
        state={'status':'waiting_operator','reason':'请重新调整观察视角'}
        view=summary(state,[{'failure':{'message_zh':'旧错误'}},{'kind':'assist'}],SimpleNamespace(faulted=False,paused=False))
        self.assertEqual(view['title'],'AI 请求协助')
        self.assertEqual(view['reason'],state['reason'])
        self.assertFalse(view['restart_service'])

    def test_host_stop_shows_underlying_call_and_restart_advice(self):
        output={'error':{'code':'outcome_unknown','message':REJECTED['cause_message'],'details':REJECTED}}
        failed=failure({'error':{'message':'runtime_fault_latched'},'calls':[{'tool':'execute_trajectory','line':32,'ok':False,'output':output}]})
        view=summary({'status':'waiting_operator','reason':'runtime_fault_latched'},[{'kind':'program','failure':failed}],SimpleNamespace(faulted=False,paused=False))
        self.assertIn('execute_trajectory',view['reason'])
        self.assertIn('32',view['reason'])
        self.assertIn('继续当前任务',view['next_step'])
        self.assertTrue(view['can_continue'])
        self.assertTrue(view['can_start_new'])

    def test_completion_does_not_claim_physical_success(self):
        view=summary({'status':'completed','reason':'探测已完成','physical_task_success':False},[{'kind':'finish'}],SimpleNamespace(faulted=False,paused=False))
        self.assertEqual(view['title'],'本轮完成')
        self.assertIn('不代表已完成真机抓放',view['reason'])

    def test_invalid_model_turn_does_not_shift_program_version(self):
        with TemporaryDirectory() as directory:
            p=Path(directory)
            for i in range(2):
                (p/f'turn-{i:02d}-context.json').write_text(json.dumps({'system':'API','context':'{}'}))
            (p/'turn-00-feedback.json').write_text(json.dumps({'error':{'message':'bad model JSON'}}))
            (p/'turn-01-decision.json').write_text(json.dumps({'kind':'program','source':'result=1','reason':'inspect'}))
            (p/'v00').mkdir();(p/'v00/result.json').write_text(json.dumps({'ok':True,'calls':[]}))
            state={'versions':[{'version':'v00'}]};rows=turns(p,state)
            self.assertIsNone(rows[0]['version']);self.assertEqual(rows[1]['version'],'v00')
            self.assertEqual(turn_detail(p,state,1)['decision']['source'],'result=1')
            with self.assertRaises(ValueError):turn_detail(p,state,999)

if __name__=='__main__':unittest.main()
