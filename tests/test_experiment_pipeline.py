"""Software-only experiment lifecycle, reset boundaries and model transports."""
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.runtime import StationRuntime
from robol2l_deploy.cap.experiment import ExperimentController
from robol2l_deploy.cap.experiment_models import ModelProfiles, ExperimentAPIModel, ModelConfigurationError
from robol2l_deploy.cap.model import FeedbackFixture


class RepeatModel:
    label = 'mock_repeat_model'
    def __init__(self): self.requests = []
    def decide(self, messages):
        self.requests.append(messages)
        return {'kind':'program','reason':'Test revised program with feedback', 'source':'result = get_robot_state()'}


class PhaseModel:
    """Answers differently per phase, so a review draft is distinguishable from
    the program actually written after the human confirmed the reset."""
    label = 'mock_phase_model'
    def __init__(self): self.phases = []
    def decide(self, messages):
        phase = json.loads(messages[-1]['content'][0]['text'])['experiment']['phase']
        self.phases.append(phase)
        if phase == 'review':
            return {'kind':'program','reason':'Ask for another attempt',
                    'source':'result = "REVIEW_DRAFT"','post':'Mock arm did not reach the target.'}
        return {'kind':'program','reason':'Written against the confirmed scene',
                'source':'result = "GENERATED"','pre_a':'Root cause: stale pose assumption.',
                'pre_b':'Expect the TCP to reach the target; risk is a blocked path.'}


class ExperimentTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.runtime = StationRuntime(MockAdapter(), Path(self.tmp.name)/'runtime')
        self.model = RepeatModel()
        self.controller = ExperimentController(self.runtime, lambda:self.model)

    def tearDown(self):
        if self.controller.busy():
            self.controller.stop.set(); self.controller.thread.join(10)
        self.runtime.close();self.tmp.cleanup()

    def wait(self):
        self.controller.thread.join(15)
        self.assertFalse(self.controller.busy())
        return self.controller.snapshot()

    def start(self):
        # Seed an existing waiting-reset record for legacy/manual-gate coverage.
        # New-run startup is covered without this stub in FirstAttemptTests and
        # the HTTP lifecycle tests below.
        with patch.object(self.controller, '_cycle',
                          side_effect=lambda: self.controller.save('waiting_reset')):
            self.controller.start('Move mock object','Object reaches target', True)
            return self.wait()

    def confirm(self):
        self.controller.confirm_reset(self.controller.state['session_id'], True, 'Objects and gripper restored', self.controller.state['attempts'])
        return self.wait()

    def test_existing_initial_reset_gate_still_requires_confirmation(self):
        state=self.start()
        self.assertEqual(state['status'],'waiting_reset')
        self.assertEqual(state['attempts'],0)
        self.assertEqual(self.runtime.adapter.executions,0)
        self.assertTrue((self.controller.folder/'initial-0.jpg').is_file())
        self.assertEqual(state['initial']['observation']['robot_state']['joint_positions']['primary'],[0]*7)
        with self.assertRaises(ValueError):self.controller.confirm_reset(state['session_id'],False)
        with self.assertRaises(ValueError):self.controller.confirm_reset('stale-session',True)
        with self.assertRaises(ValueError):self.controller.start('Other','Other',True)

    def test_eight_attempts_no_ninth_and_reset_between_every_program(self):
        self.start()
        for n in range(1,9):
            state=self.confirm()
            self.assertEqual(state['attempts'],n)
            self.assertEqual(state['status'],'exhausted' if n==8 else 'waiting_reset',state)
        self.assertEqual([v['version'] for v in state['versions']],[f'v{i:02d}' for i in range(8)])
        self.assertEqual(len(state['interventions']),8)
        with self.assertRaises(ValueError):self.confirm()
        self.assertFalse(self.runtime.allow_motion)
        context=json.loads(self.model.requests[-1][-1]['content'][0]['text'])
        self.assertEqual(context['experiment']['remaining'],0)
        self.assertTrue(any('feedback' in h for h in context['history']))

    def test_fixture_repairs_actual_executor_error_then_finishes(self):
        self.model=FeedbackFixture();self.start()
        first=self.confirm()
        self.assertEqual(first['status'],'waiting_reset',first)
        self.assertEqual(first['versions'][0]['feedback']['error']['type'],'TypeError')
        self.assertEqual(self.runtime.adapter.executions,0)
        second=self.confirm()
        self.assertEqual(second['status'],'completed',second)
        self.assertEqual(second['attempts'],2)
        self.assertTrue(second['synthetic'])
        self.assertFalse(second['physical_task_success'])
        self.assertGreater(self.runtime.adapter.executions,0)

    def test_restart_preserves_budget_but_never_executes(self):
        self.start();self.confirm()
        before=self.runtime.adapter.executions
        restored=ExperimentController(self.runtime,lambda:self.model)
        self.assertEqual(restored.snapshot()['attempts'],1)
        self.assertEqual(restored.snapshot()['status'],'interrupted')
        self.assertIsNone(restored.state['pending'])
        self.assertIsNone(restored.thread)
        self.assertEqual(self.runtime.adapter.executions,before)
        self.controller=restored
        self.assertEqual(self.confirm()['attempts'],2)

    def test_human_return_uses_runtime_and_does_not_restore_gripper_or_spend_attempt(self):
        state=self.start()
        self.runtime.adapter.q[:]=.01;self.runtime.adapter.gripper=.4
        self.controller.return_to_initial(state['session_id'],True)
        state=self.wait()
        self.assertEqual(state['status'],'waiting_reset',state)
        self.assertEqual(self.runtime.adapter.q.tolist(),[0]*7)
        self.assertEqual(self.runtime.adapter.gripper,.4)
        self.assertEqual(state['attempts'],0)
        self.assertEqual(self.runtime.adapter.executions,1)

    def test_human_return_acknowledges_a_cleared_fault_but_never_a_live_device_error(self):
        from robol2l_deploy.contracts import ExecutionResult
        from robol2l_deploy.errors import ApiError, ErrorCode
        state=self.start()
        # The journal a red reflex leaves behind, and the latch it sets.
        self.runtime.event('execution',operation='execute_trajectory',physical_evidence=False,
            result=ExecutionResult(False,0,error=ApiError(ErrorCode.CONTROLLER_FAILED,
                'libfranka: Move command aborted: motion aborted by reflex!')))
        self.runtime.faulted=True
        latched=StationRuntime(MockAdapter(),self.runtime.directory)
        try:self.assertTrue(latched.faulted)
        finally:latched.close()
        # Still red on the device: the button refuses and dispatches nothing.
        reading=self.runtime.adapter.get_robot_state
        def not_idle():raise ApiError(ErrorCode.SAFETY_INTERLOCK,'robot_not_idle')
        self.runtime.adapter.get_robot_state=not_idle
        with self.assertRaises(ApiError):self.controller.return_to_initial(state['session_id'],True)
        self.runtime.adapter.get_robot_state=reading
        self.assertTrue(self.runtime.faulted)
        self.assertEqual(self.runtime.adapter.executions,0)
        # Cleared in Franka Desk: the same click acknowledges the latch and moves.
        self.runtime.adapter.q[:]=.01;self.runtime.pause(True)
        self.controller.return_to_initial(state['session_id'],True)
        state=self.wait()
        self.assertEqual(state['status'],'waiting_reset',state)
        self.assertFalse(self.runtime.faulted);self.assertFalse(self.runtime.paused)
        self.assertEqual(self.runtime.adapter.q.tolist(),[0]*7)
        self.assertEqual(self.runtime.adapter.executions,1)
        self.assertEqual(state['attempts'],0)
        kinds=[i['kind'] for i in state['interventions']]
        self.assertEqual(kinds[-2:],['human_requested_arm_return','arm_return_result'])
        self.assertEqual(len(state['interventions'][-2]['acknowledged_robot_state']['joint_positions']['primary']),7)
        cleared=StationRuntime(MockAdapter(),self.runtime.directory)
        try:self.assertFalse(cleared.faulted)
        finally:cleared.close()

    def test_operator_unlock_is_recorded_in_the_live_run_only(self):
        self.controller.record_unlock({'has_errors': False})  # No run yet: nothing to record.
        self.start()
        self.controller.record_unlock(self.runtime.unlock())
        entry=self.controller.snapshot()['interventions'][-1]
        self.assertEqual(entry['kind'],'human_unlocked_controller')
        self.assertFalse(entry['receipt']['has_errors'])
        self.controller.save('stopped')
        self.controller.record_unlock({'has_errors': False})
        kinds=[i['kind'] for i in self.controller.snapshot()['interventions']]
        self.assertEqual(kinds.count('human_unlocked_controller'),1)

    def test_starting_the_next_experiment_acknowledges_a_latched_fault(self):
        from robol2l_deploy.contracts import ExecutionResult
        from robol2l_deploy.errors import ApiError, ErrorCode
        self.runtime.event('execution',operation='execute_trajectory',physical_evidence=False,
            result=ExecutionResult(False,0,error=ApiError(ErrorCode.CONTROLLER_FAILED,
                'libfranka: Move command aborted: motion aborted by reflex!')))
        self.runtime.faulted=True;self.runtime.pause(True)
        reading=self.runtime.adapter.get_robot_state
        def not_idle():raise ApiError(ErrorCode.SAFETY_INTERLOCK,'robot_not_idle')
        self.runtime.adapter.get_robot_state=not_idle
        with self.assertRaises(ApiError):self.controller.start('Move mock object','Object reaches target',True)
        self.runtime.adapter.get_robot_state=reading
        self.assertTrue(self.runtime.faulted);self.assertIsNone(self.controller.state)
        # A rejected form or model must not clear the station either.
        with self.assertRaises(ValueError):self.controller.start('   ','Object reaches target',True)
        self.assertTrue(self.runtime.faulted)
        state=self.start()
        self.assertEqual(state['status'],'waiting_reset',state)
        self.assertFalse(self.runtime.faulted);self.assertFalse(self.runtime.paused)
        self.assertEqual(state['interventions'][0]['kind'],'human_started_experiment')
        self.assertEqual(len(state['interventions'][0]['acknowledged_robot_state']['joint_positions']['primary']),7)
        cleared=StationRuntime(MockAdapter(),self.runtime.directory)
        try:self.assertFalse(cleared.faulted)
        finally:cleared.close()

    def test_pause_during_model_generation_never_dispatches(self):
        self.start()
        entered=threading.Event();release=threading.Event()
        def decide(messages):
            entered.set();release.wait(5)
            return {'kind':'program','reason':'mock','source':'result=set_gripper(.2)'}
        self.model.decide=decide
        self.controller.confirm_reset(self.controller.state['session_id'],True,expected_attempt=0)
        self.assertTrue(entered.wait(5))
        self.controller.pause(self.controller.state['session_id']);release.set()
        state=self.wait()
        self.assertEqual(state['status'],'paused')
        self.assertEqual(state['attempts'],0)
        self.assertEqual(self.runtime.adapter.executions,0)

    def test_invalid_programs_are_bounded_without_spending_attempt(self):
        self.model.decide=lambda _:dict(kind='program',reason='bad',source='import os')
        self.start();state=self.confirm()
        self.assertEqual(state['attempts'],0)
        self.assertEqual(state['status'],'waiting_operator')
        self.assertEqual(state['reason'],'three_invalid_model_responses')

    def test_authorization_only_after_valid_program_and_revoked_before_review(self):
        self.start()
        # Simulate a preceding operator action leaving mock motion enabled.
        self.runtime.allow_motion = True
        events = []
        def decide(request):
            context = json.loads(request[1]['content'][0]['text'])
            experiment = context['experiment']
            policy = experiment['execution_authorization']
            self.assertTrue(policy['task_authorization_recorded'])
            self.assertFalse(policy['human_code_approval_required'])
            self.assertTrue(policy['disabled_during_model_decision_is_expected'])
            self.assertFalse(policy['motion_enabled_now'])
            self.assertFalse(self.runtime.allow_motion)
            self.assertFalse(context['api']['physical_motion_for_generated_programs'])
            self.assertIn('host AST validation and compilation', request[0]['content'])
            events.append(experiment['phase'])
            if experiment['phase'] == 'review':
                return dict(kind='finish', reason='Mock done')
            # First rejected code must never reach authorization or execution.
            return dict(kind='program', reason='mock',
                        source='import os' if events.count('generate') == 1 else 'result = 1')
        original_authorize = self.controller._authorize
        def authorize():
            self.assertEqual(events, ['generate', 'generate'])
            self.assertEqual(self.controller.state['gate']['rejected'], 1)
            self.assertEqual(self.controller.state['history'][-1]['decision']['source'], 'result = 1')
            events.append('authorize')
            original_authorize()
        def execute(source, *args, **kwargs):
            self.assertEqual(source, 'result = 1')
            self.assertTrue(self.runtime.allow_motion)
            events.append('execute')
            return dict(ok=True, calls=[])
        self.model.decide = decide
        with patch.object(self.controller, '_authorize', side_effect=authorize), patch(
                'robol2l_deploy.cap.experiment.ProgramExecutor.execute', side_effect=execute):
            state = self.confirm()
        self.assertEqual(events, ['generate', 'generate', 'authorize', 'execute', 'review'])
        self.assertEqual(state['status'], 'completed')
        self.assertEqual(state['attempts'], 1)
        self.assertFalse(self.runtime.allow_motion)

    def test_three_rejections_never_authorize(self):
        self.start()
        self.model.decide = lambda _: dict(kind='program', reason='bad', source='import os')
        with patch.object(self.controller, '_authorize') as authorize:
            state = self.confirm()
            authorize.assert_not_called()
        self.assertEqual(state['attempts'], 0)
        self.assertFalse(self.runtime.allow_motion)

    def test_review_permission_misdiagnosis_is_returned_to_agent_not_operator(self):
        reviews = []
        def decide(request):
            phase = json.loads(request[1]['content'][0]['text'])['experiment']['phase']
            if phase == 'generate':
                return dict(kind='program', reason='mock attribute error', source='result = get_robot_state().missing_pose')
            reviews.append(request)
            if len(reviews) == 1:
                return dict(kind='assist', reason='当前物理运动未启用，需要操作人员启用',
                            post='AttributeError; fix the program.')
            self.assertTrue(any('assistance_validation_error' in b.get('text', '')
                                for b in request[-1]['content']))
            return dict(kind='program', reason='Repair attribute access after reset',
                        source='result = get_robot_state()', post='Fix attribute access, no hardware fault.')
        self.model.decide = decide
        self.start()
        with patch.object(self.controller, '_authorize', wraps=self.controller._authorize) as authorize:
            state = self.confirm()
            self.assertEqual(authorize.call_count, 1)
        self.assertEqual(state['status'], 'waiting_reset')
        self.assertEqual(state['attempts'], 1)
        self.assertEqual(state['gate']['rejected'], 0)
        self.assertEqual(state['assistance_validation']['rejected'], 1)
        self.assertEqual(state['versions'][0]['feedback']['error']['type'], 'AttributeError')
        self.assertIn('Fix attribute', state['versions'][0]['post'])
        self.assertFalse(self.runtime.allow_motion)
        self.assertIsNone(state['pending'])

    def test_repeated_authorization_assist_is_model_protocol_error(self):
        self.start()
        self.model.decide = lambda _: dict(kind='assist', reason='Enable motion',
                                           assist_type='motion_authorization', assist_evidence='active=false')
        with patch.object(self.controller, '_authorize') as authorize:
            state = self.confirm()
            authorize.assert_not_called()
        self.assertEqual(state['status'], 'error')
        self.assertEqual(state['reason'], 'model_assistance_protocol_error')
        self.assertEqual(state['error_category'], 'model_protocol')
        self.assertEqual(state['gate']['rejected'], 0)
        self.assertEqual(state['attempts'], 0)

    def test_evidenced_external_assistance_remains_blocked(self):
        self.start()
        self.model.decide = lambda _: dict(kind='assist', reason='Object is trapped; inspect the scene',
            assist_type='physical_intervention', assist_evidence='Mock image shows trapped object against fixture')
        with patch.object(self.controller, '_authorize') as authorize:
            state = self.confirm()
            authorize.assert_not_called()
        self.assertEqual(state['status'], 'waiting_operator')
        self.assertEqual(state['attempts'], 0)

    def test_fault_or_pause_arriving_during_review_cannot_be_overridden(self):
        for fault in (True, False):
            self.runtime.faulted = False
            self.runtime.paused = False
            self.controller.stop.clear()
            self.start()
            if fault: self.runtime.faulted = True
            else: self.runtime.paused = True
            self.controller._handle(dict(kind='finish', reason='done'), 'review')
            self.assertEqual(self.controller.state['status'], 'waiting_operator' if fault else 'paused')
            self.controller.save('stopped')
        self.runtime.paused = False

    def test_duplicate_confirmation_cannot_start_next_version(self):
        self.start();self.confirm()
        with self.assertRaisesRegex(ValueError,'stale_reset_confirmation'):
            self.controller.confirm_reset(self.controller.state['session_id'],True,expected_attempt=0)
        self.assertEqual(self.controller.state['attempts'],1)

    def test_controller_fault_stops_review_and_is_cleared_only_by_a_human_gate(self):
        from robol2l_deploy.errors import ApiError, ErrorCode
        self.start()
        def failed(*args,**kwargs):
            self.runtime.faulted=True
            return dict(ok=False,error=dict(code='outcome_unknown',message='mock unknown outcome'),calls=[])
        with patch('robol2l_deploy.cap.experiment.ProgramExecutor.execute',side_effect=failed):
            state=self.confirm()
        self.assertEqual(state['status'],'waiting_operator')
        self.assertEqual(state['attempts'],1)
        self.assertEqual(len(self.model.requests),1)
        # Nothing clears the latch on its own: a device still in error refuses.
        reading=self.runtime.adapter.get_robot_state
        def not_idle():raise ApiError(ErrorCode.SAFETY_INTERLOCK,'robot_not_idle')
        self.runtime.adapter.get_robot_state=not_idle
        with self.assertRaises(ApiError):self.confirm()
        self.runtime.adapter.get_robot_state=reading
        self.assertTrue(self.runtime.faulted)
        self.assertEqual(self.controller.state['attempts'],1)
        self.assertEqual(len(self.model.requests),1)
        # Inspected and cleared in Desk: the human's confirmation acknowledges it.
        state=self.confirm()
        self.assertFalse(self.runtime.faulted)
        self.assertEqual(state['attempts'],2)
        self.assertEqual(state['status'],'waiting_reset',state)
        self.assertTrue(state['interventions'][-1]['fault_acknowledged'])
        self.assertEqual(len(state['interventions'][-1]['acknowledged_robot_state']['joint_positions']['primary']),7)
        self.assertTrue(any(h.get('human_intervention',{}).get('fault_acknowledged') for h in state['history']))
        self.assertNotIn('acknowledged_robot_state',
            next(h['human_intervention'] for h in reversed(state['history']) if 'human_intervention' in h))

    def test_review_draft_is_never_dispatched_and_records_are_kept(self):
        # The human gate is physical reset only: every dispatched program is written
        # after that confirmation, against the fresh observation, never before it.
        self.model=PhaseModel();self.start()
        first=self.confirm()
        entry=first['versions'][0]
        self.assertEqual(entry['source'],'result = "GENERATED"')
        self.assertEqual(entry['pre_a'],'Root cause: stale pose assumption.')
        self.assertEqual(entry['pre_b'],'Expect the TCP to reach the target; risk is a blocked path.')
        self.assertEqual(entry['post'],'Mock arm did not reach the target.')
        self.assertIsNone(first['pending'])
        self.assertEqual(first['status'],'waiting_reset')
        second=self.confirm()
        self.assertEqual([v['source'] for v in second['versions']],['result = "GENERATED"']*2)
        self.assertEqual(self.model.phases,['generate','review','generate','review'])

    def test_experiment_records_are_optional(self):
        # A model that omits PRE/POST must not stall the run or fake a record.
        state=self.start();state=self.confirm()
        self.assertEqual(state['attempts'],1)
        for name in ('pre_a','pre_b','post'):
            self.assertNotIn(name,state['versions'][0])

    def test_rejected_programs_are_counted_on_the_version(self):
        answers=[dict(kind='program',reason='rejected by the machine check',source='import os')]
        def flaky(messages):
            return answers.pop(0) if answers else dict(kind='program',reason='clean',source='result = get_robot_state()')
        self.model.decide=flaky
        self.start();state=self.confirm()
        self.assertEqual(state['attempts'],1)
        self.assertEqual(state['versions'][0]['rejected'],1)
        self.assertEqual(state['versions'][0]['source'],'result = get_robot_state()')

    def test_version_outcome_separates_task_success_from_program_success(self):
        # A program can run cleanly and still not do the task. Only the version the
        # review judged complete may be reported as a success.
        answers=[dict(kind='program',reason='v00',source='result = get_robot_state()'),
                 dict(kind='program',reason='ran but did not move',source='result = get_robot_state()'),
                 dict(kind='program',reason='v01',source='result = get_robot_state()'),
                 dict(kind='finish',reason='object is in the plate')]
        self.model.decide=lambda messages:answers.pop(0)
        self.start()
        first=self.confirm()
        self.assertEqual(first['versions'][0]['outcome'],'task_incomplete')
        self.assertTrue(first['versions'][0]['feedback']['ok'])  # program fine, task not
        second=self.confirm()
        self.assertEqual(second['status'],'completed')
        self.assertEqual([v['outcome'] for v in second['versions']],['task_incomplete','task_complete'])

    def test_assist_review_marks_the_version_blocked(self):
        answers=[dict(kind='program',reason='v00',source='result = get_robot_state()'),
                 dict(kind='assist',reason='object is off the table',assist_type='physical_intervention',
                      assist_evidence='Both camera images show the cube outside the workspace.')]
        self.model.decide=lambda messages:answers.pop(0)
        self.start();state=self.confirm()
        self.assertEqual(state['status'],'waiting_operator')
        self.assertEqual(state['versions'][0]['outcome'],'blocked')

    def test_skill_library_reaches_the_model_request(self):
        import io,zipfile
        from robol2l_deploy.cap.skills import SkillLibrary
        buffer=io.BytesIO()
        with zipfile.ZipFile(buffer,'w') as bundle:
            bundle.writestr('tips.md','# Sampling tip\nSample inside the mask.')
        library=SkillLibrary(Path(self.tmp.name)/'skills')
        library.install('kit',buffer.getvalue())
        self.controller=ExperimentController(self.runtime,lambda:self.model,skills=library)
        self.start();self.confirm()
        sent=json.loads(self.model.requests[0][-1]['content'][0]['text'])
        self.assertEqual(sent['skills']['name'],'kit')
        self.assertIn('Sample inside the mask.',sent['skills']['files'][0]['content'])
        self.assertIn('DATA',sent['skills']['how_to_use'])
        # Without a library the key is simply absent from the request.
        self.model.requests.clear()
        self.controller=ExperimentController(self.runtime,lambda:self.model,Path(self.tmp.name)/'plain')
        self.start();self.confirm()
        self.assertNotIn('skills',json.loads(self.model.requests[0][-1]['content'][0]['text']))

    def test_records_list_and_archive_round_trip(self):
        import io,zipfile
        self.start();self.confirm()
        rows=self.controller.records()
        self.assertEqual(len(rows),1)
        row=rows[0]
        self.assertEqual(row['session_id'],self.controller.state['session_id'])
        self.assertEqual(row['versions'],1)
        self.assertTrue(row['active'])
        self.assertFalse(row['succeeded'])
        full=self.controller.record(row['session_id'])
        self.assertEqual(full['session_id'],row['session_id'])
        with self.assertRaisesRegex(ValueError,'invalid_experiment_id'):
            self.controller.record('../escape')
        with self.assertRaisesRegex(ValueError,'not_found'):
            self.controller.record('0'*32)
        bundle=self.controller.archive(row['session_id'])
        names=zipfile.ZipFile(io.BytesIO(bundle)).namelist()
        self.assertIn(row['session_id']+'/experiment.json',names)
        self.assertTrue(any(n.endswith('v00/source.py') for n in names),names)

    def test_records_can_be_deleted_but_not_while_live(self):
        self.start();self.confirm()
        session=self.controller.state['session_id']
        folder=self.controller.directory/session
        self.assertTrue((folder/'experiment.json').is_file())
        with self.assertRaisesRegex(ValueError,'explicit_record_delete_confirmation_required'):
            self.controller.delete_record(session,False)
        with self.assertRaisesRegex(ValueError,'invalid_experiment_id'):
            self.controller.delete_record('../escape',True)
        with self.assertRaisesRegex(ValueError,'not_found'):
            self.controller.delete_record('0'*32,True)
        # The run is still live, so its evidence is protected.
        with self.assertRaisesRegex(ValueError,'end_the_experiment_before_deleting'):
            self.controller.delete_record(session,True)
        self.controller.end(session)
        value=self.controller.delete_record(session,True)
        self.assertEqual(value['records'],[])
        self.assertFalse(folder.exists())
        self.assertIsNone(self.controller.state)

    def test_each_experiment_starts_with_no_prior_knowledge(self):
        # Two runs in a row: the second must not see the first one's history.
        self.start();self.confirm()
        first=self.controller.state['session_id']
        self.controller.end(first)
        self.model.requests.clear()
        self.start();self.confirm()
        self.assertNotEqual(self.controller.state['session_id'],first)
        for key in ('history','versions','steps','interventions'):
            self.assertTrue(len(self.controller.state[key])>0 or key=='interventions')
        sent=json.loads(self.model.requests[0][-1]['content'][0]['text'])
        # Only this run's own reset is in history; nothing from the previous one.
        self.assertEqual([list(h)[0] for h in sent['history']],['operator_continuation', 'human_intervention'])
        self.assertIsNone(sent['experiment']['operator_continuation']['prior_assistance'])
        self.assertEqual(sent['experiment']['attempts'],0)
        self.assertNotIn(first,json.dumps(sent))
        self.assertEqual(self.controller.state['versions'][0]['version'],'v00')

    def test_recording_is_optional_and_never_blocks_the_attempt(self):
        state=self.controller.start('Move mock object','Object reaches target',True,
                                    record_cameras=['wrist'])
        state=self.wait()
        self.assertEqual(self.controller.state['record_cameras'],['wrist'])
        videos=state['versions'][0].get('videos') or {}
        self.assertEqual(list(videos),['wrist'])
        receipt=videos['wrist']
        self.assertEqual(receipt['file'],'camera-v00-wrist.mp4')
        self.assertIsNone(receipt['error'])
        self.assertTrue(receipt['written'],receipt)
        # The file lands beside the stills, not in the version folder the
        # executor creates for itself.
        clip=self.controller.folder/receipt['file']
        self.assertTrue(clip.is_file() and clip.stat().st_size>0)
        self.assertEqual(state['attempts'],1)
        with self.assertRaisesRegex(ValueError,'unknown_camera_for_recording'):
            self.controller.end(state['session_id'])
            self.controller.start('g','c',True,record_cameras=['nope'])

    def test_recording_defaults_to_off(self):
        state=self.start()
        self.assertEqual(state['record_cameras'],[])
        state=self.confirm()
        self.assertIsNone(state['versions'][0].get('videos'))

    def test_records_can_be_tagged_and_deleted_in_bulk(self):
        made=[]
        for n in range(3):
            self.start();self.confirm()
            made.append(self.controller.state['session_id'])
            self.controller.end(made[-1])
        self.assertEqual(len(self.controller.records()),3)
        tagged=self.controller.tag_records(made[:2],['夜间批次','needs-review'])
        by_id={r['session_id']:r for r in tagged['records']}
        self.assertEqual(by_id[made[0]]['tags'],['夜间批次','needs-review'])
        self.assertEqual(by_id[made[2]]['tags'],[])
        # Tags replace rather than accumulate, and duplicates collapse.
        self.controller.tag_records([made[0]],['x','x','  '])
        self.assertEqual(self.controller.record(made[0])['tags'],['x'])
        for bad in ([str(n) for n in range(13)],'notalist',[1]):
            with self.assertRaises(ValueError):self.controller.tag_records([made[0]],bad)
        self.assertEqual(self.controller.record(made[0])['tags'],['x'])  # unchanged
        with self.assertRaisesRegex(ValueError,'select_at_least_one_record'):
            self.controller.delete_records([],True)
        with self.assertRaisesRegex(ValueError,'invalid_experiment_id'):
            self.controller.delete_records(['../escape'],True)
        with self.assertRaisesRegex(ValueError,'not_found'):
            self.controller.delete_records([made[0],'0'*32],True)
        self.assertEqual(len(self.controller.records()),3)   # nothing partially deleted
        left=self.controller.delete_records(made[:2],True)
        self.assertEqual([r['session_id'] for r in left['records']],[made[2]])

    def test_tagging_does_not_change_which_run_is_restored(self):
        self.start();self.confirm()
        first=self.controller.state['session_id']
        self.controller.end(first)
        self.start();self.confirm()
        newest=self.controller.state['session_id']
        self.controller.end(newest)
        before=(self.controller.directory/first/'experiment.json').stat().st_mtime_ns
        self.controller.tag_records([first],['后补标签'])
        self.assertEqual((self.controller.directory/first/'experiment.json').stat().st_mtime_ns,before)
        self.assertEqual(self.controller.record(first)['tags'],['后补标签'])
        restored=ExperimentController(self.runtime,lambda:self.model,self.controller.directory)
        self.assertEqual(restored.state['session_id'],newest)

    def test_bulk_delete_refuses_while_a_selected_run_is_live(self):
        self.start();self.confirm()
        live=self.controller.state['session_id']
        with self.assertRaisesRegex(ValueError,'end_the_experiment_before_deleting'):
            self.controller.delete_records([live],True)
        self.assertEqual(len(self.controller.records()),1)

    def test_operator_speed_recommendation_reaches_the_model(self):
        self.controller.start('Move mock object','Object reaches target',True,speed_factor=0.02)
        self.wait();self.confirm()
        sent=json.loads(self.model.requests[0][-1]['content'][0]['text'])
        advice=sent['experiment']['operator_speed_recommendation']
        self.assertEqual(advice['time_dilation_factor'],0.02)
        # It is advice, not a host-enforced clamp, and must say so.
        self.assertFalse(advice['enforced_by_host'])
        self.assertIn('contact',advice['go_lower_for'])
        self.assertIn('not a host-enforced',self.model.requests[0][0]['content'])

    def test_speed_defaults_to_the_documented_value_and_is_bounded(self):
        from robol2l_deploy.cap.experiment import DEFAULT_SPEED
        self.assertEqual(DEFAULT_SPEED,0.05)
        self.start()
        self.assertEqual(self.controller.state['speed_factor'],0.05)
        self.controller.end(self.controller.state['session_id'])
        for bad in (0,-0.1,1.5,'fast',True):
            with self.assertRaisesRegex(ValueError,'speed_factor_must_be'):
                self.controller.start('g','c',True,speed_factor=bad)

    def test_every_graph_node_is_recorded_in_order(self):
        # The console draws the flow from these steps. Nodes with no status of
        # their own (PLAN, GATE, SUCCESS, BUDGET, NEXT, DONE) must appear too,
        # or the diagram looks like it skipped them.
        self.model=FeedbackFixture();self.start()
        first=self.confirm()
        trace=[s['node'] for s in first['steps']]
        self.assertEqual(trace,['START','SETUP','CLICK','SNAP','RESET','CONFIRM','OBSERVE',
                                'PLAN','CODE','GATE','RUN','RESULT','POST','SUCCESS','BUDGET','NEXT'])
        second=self.confirm()
        tail=[s['node'] for s in second['steps']][len(trace):]
        self.assertEqual(tail,['RESET','CONFIRM','OBSERVE','PLAN','CODE','GATE','RUN','RESULT',
                               'POST','SUCCESS','DONE','END'])
        # Every step carries a real timestamp and the attempt it belonged to.
        self.assertTrue(all(s['time'] > 0 for s in second['steps']))
        self.assertEqual(second['steps'][-1]['attempt'],2)

    def test_rejected_program_returns_to_the_code_node(self):
        answers=[dict(kind='program',reason='bad',source='import os'),
                 dict(kind='program',reason='ok',source='result = get_robot_state()'),
                 dict(kind='finish',reason='done')]
        self.model.decide=lambda messages:answers.pop(0)
        self.start();state=self.confirm()
        trace=[s['node'] for s in state['steps']]
        # CODE appears twice: once for the rejected draft, once for the accepted one.
        self.assertEqual(trace.count('CODE'),2)
        self.assertEqual(trace.index('GATE'),trace.index('RUN')-1)

    def test_first_message_can_be_previewed_before_starting(self):
        # Nothing has been started: the operator still wants to see message one.
        self.assertIsNone(self.controller.state)
        preview=self.controller.preview(goal='把绿色方块放进盘子',criteria='方块完全落在盘内')
        self.assertIn('EXPERIMENT PROTOCOL',preview['system'])
        self.assertIn('把绿色方块放进盘子',preview['context']['task']['language'])
        self.assertIn('方块完全落在盘内',preview['context']['task']['language'])
        self.assertEqual(preview['context']['experiment']['attempts'],0)
        self.assertTrue(any(b['type']=='image' for b in preview['blocks']))
        # Previewing must not create or disturb an experiment.
        self.assertIsNone(self.controller.state)
        self.assertEqual(self.controller.records(),[])
        with self.assertRaisesRegex(ValueError,'fill_in_the_task_goal'):
            self.controller.preview(goal='   ')

    def test_prompt_preview_matches_what_would_be_sent(self):
        self.start()
        preview=self.controller.preview()
        self.assertIn('EXPERIMENT PROTOCOL',preview['system'])
        self.assertEqual(preview['context']['experiment']['phase'],'generate')
        self.assertIn('Move mock object',preview['context']['task']['language'])
        self.assertEqual(preview['system_chars'],len(preview['system']))
        self.assertEqual([b['type'] for b in preview['blocks']][0],'text')
        self.assertTrue(any(b['type']=='image' for b in preview['blocks']))
        # Images are reported by size only, never inlined into the preview.
        self.assertNotIn('base64',json.dumps(preview['blocks']))

    def test_eighth_attempt_can_finish_without_ninth_execution(self):
        original=self.model.decide
        def final(messages):
            context=json.loads(messages[-1]['content'][0]['text'])
            if context['experiment']['attempts']==8:
                return dict(kind='finish',reason='Mock final assessment')
            return original(messages)
        self.model.decide=final
        self.start()
        for _ in range(8):state=self.confirm()
        self.assertEqual(state['status'],'completed')
        self.assertEqual(len(state['versions']),8)


class ModelSettingsTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.path=Path(self.tmp.name)/'experiment-model-settings.json'
        self.profiles=ModelProfiles(self.path,lambda:dict(base_url='https://example.test/v1',model='existing-model'))
    def tearDown(self):self.tmp.cleanup()

    def configure(self,provider='openai'):
        return self.profiles.save(dict(mode='api',provider=provider,reasoning='none',
            profile=dict(base_url='https://example.test/v1',model='test-model',api_key='private-test-key')))

    def test_persist_and_redact_separate_profiles(self):
        self.assertEqual(self.profiles.public()['profiles']['openai']['model'],'existing-model')
        visible=self.configure()
        self.assertNotIn('private-test-key',json.dumps(visible))
        self.assertEqual(self.path.stat().st_mode&0o777,0o600)
        restored=ModelProfiles(self.path)
        self.assertEqual(restored.read()['profiles']['openai']['api_key'],'private-test-key')
        restored.save(dict(profile={'api_key':''}))
        self.assertTrue(restored.public()['profiles']['openai']['key_saved'])
        self.assertFalse(restored.public()['profiles']['claude']['key_saved'])
        restored.save(dict(clear_key=True))
        self.assertFalse(restored.public()['profiles']['openai']['key_saved'])

    def test_key_round_trip_and_rejected_input_preserves_saved_key(self):
        self.configure()
        token = 'sk-test_AZaz09-._+/='  # synthetic credential, never sent
        body = json.loads(json.dumps({'profile': {'api_key': token}}))
        self.profiles.save(body)
        model = ExperimentAPIModel(ModelProfiles(self.path).read())
        self.assertEqual(model.key, token)
        for bad in ('中文测试凭据', 'sk-test\nheader', 'sk-test space', 'sk-test\x7f'):
            with self.assertRaisesRegex(ModelConfigurationError, '^model_configuration_error_key_format$'):
                self.profiles.save({'profile': {'api_key': bad}})
            self.assertEqual(self.profiles.read()['profiles']['openai']['api_key'], token)
        # Older saved configurations are validated before any request too.
        cfg = self.profiles.read()
        cfg['profiles']['openai']['api_key'] = '中文测试凭据'
        with patch('requests.post') as post:
            with self.assertRaisesRegex(ModelConfigurationError, 'key_format'):
                ExperimentAPIModel(cfg)
            post.assert_not_called()

    def test_named_environment_key_survives_restart_without_persisting_secret(self):
        with patch.dict(os.environ, {'PLAYER_TEST_API_KEY': 'environment-test-secret'}):
            self.profiles.save(dict(mode='api', profile={'api_key_env': 'PLAYER_TEST_API_KEY'}))
            restored = ModelProfiles(self.path)
            self.assertTrue(restored.public()['profiles']['openai']['environment_key_loaded'])
            model = restored.model(False)
            self.assertEqual(model.key, 'environment-test-secret')
            self.assertNotIn('environment-test-secret', self.path.read_text())
            self.assertNotIn('environment-test-secret', json.dumps(model.settings))
            response = Mock(ok=True, is_redirect=False)
            response.json.return_value = {'choices': [{'message': {'content': '{"kind":"assist","reason":"test"}'}}]}
            with patch('requests.post', return_value=response) as post:
                model.decide([])
            self.assertEqual(post.call_args.kwargs['headers']['Authorization'], 'Bearer environment-test-secret')
            for invalid in ('1KEY', 'KEY-NAME', 'KEY\nX', '中文'):
                with self.assertRaisesRegex(ValueError, 'invalid_api_key_env'):
                    restored.save({'profile': {'api_key_env': invalid}})
                self.assertEqual(restored.read()['profiles']['openai']['api_key_env'], 'PLAYER_TEST_API_KEY')
            restored.save({'profile': {'api_key_env': ''}})
            with self.assertRaisesRegex(ModelConfigurationError, 'key_missing'):
                restored.model(False)

    def test_request_header_error_is_configuration_error_without_secret(self):
        import requests
        self.configure()
        model = ExperimentAPIModel(self.profiles.read())
        for error in (UnicodeEncodeError('latin-1', '秘密', 0, 2, 'bad'),
                      requests.exceptions.InvalidHeader('private-test-key')):
            with patch('requests.post', side_effect=error), self.assertRaisesRegex(
                    ModelConfigurationError, '^model_configuration_error_request_header$'):
                model.decide([])

    def test_protocols_and_no_secret_in_settings(self):
        messages=[{'role':'system','content':'system'},{'role':'user','content':[
            {'type':'text','text':'task'}, {'type':'image_url','image_url':{'url':'data:image/jpeg;base64,YWJj'}}]}]
        for provider in ('openai','claude'):
            self.configure(provider)
            model=ExperimentAPIModel(self.profiles.read())
            self.assertNotIn('private-test-key',json.dumps(model.settings))
            answer=json.dumps({'kind':'assist','reason':'mock'})
            raw={'choices':[{'message':{'content':answer}}]} if provider=='openai' else {'content':[{'type':'text','text':answer}]}
            response=Mock(ok=True,is_redirect=False);response.json.return_value=raw
            with patch('requests.post',return_value=response) as post:
                self.assertEqual(model.decide(messages)['kind'],'assist')
            sent=post.call_args.kwargs
            self.assertFalse(sent['allow_redirects'])
            if provider=='claude':
                self.assertEqual(sent['json']['messages'][0]['content'][1]['source']['data'],'YWJj')
                self.assertEqual(sent['headers']['x-api-key'],'private-test-key')
            else:self.assertEqual(sent['headers']['Authorization'],'Bearer private-test-key')

    def test_provider_errors_never_echo_key(self):
        self.configure();model=ExperimentAPIModel(self.profiles.read())
        response=Mock(ok=False,is_redirect=False,status_code=401,text='private-test-key')
        with patch('requests.post',return_value=response), self.assertRaisesRegex(RuntimeError,'http_401') as error:
            model.decide([])
        self.assertNotIn('private-test-key',str(error.exception))

    def test_api_timeout_is_a_named_error_and_configurable(self):
        import requests
        self.configure();model=ExperimentAPIModel(self.profiles.read())
        self.assertEqual(model.timeout_s,600)
        with patch('requests.post',side_effect=requests.Timeout()) as post, self.assertRaisesRegex(RuntimeError,'^model_request_timeout$'):
            model.decide([])
        self.assertEqual(post.call_args.kwargs['timeout'],600)
        with patch.dict(os.environ,{'ROBOL2L_MODEL_TIMEOUT_S':'1200'}):
            self.assertEqual(ExperimentAPIModel(self.profiles.read()).timeout_s,1200)

    def test_api_reply_parsing_tolerates_fences_and_prose_but_not_garbage(self):
        from robol2l_deploy.cap.experiment_models import decision_json
        want={'kind':'finish','reason':'done'}
        for text in ('{"kind":"finish","reason":"done"}',
                     '```json\n{"kind":"finish","reason":"done"}\n```',
                     'Here is my decision:\n{"kind":"finish","reason":"done"}\nThanks.'):
            self.assertEqual(decision_json(text),want,text)
        for text in ('no json here','[1, 2]','{broken'):
            with self.assertRaises(ValueError):decision_json(text)

    def test_api_profile_reasoning_is_saved_per_profile(self):
        self.configure()
        made=self.profiles.save(dict(create_profile=True,provider='claude',profile_name='Claude 深度',
            profile_reasoning='high',profile=dict(model='m',api_key='k')))
        self.assertEqual(made['profiles'][made['profile_id']]['reasoning'],'high')
        self.assertEqual(made['profiles']['openai']['reasoning'],'none')
        self.assertEqual(made['reasoning'],'none')  # the Codex/global value is untouched
        with self.assertRaisesRegex(ValueError,'invalid_model_settings'):
            self.profiles.save(dict(profile_id=made['profile_id'],profile_reasoning='extreme'))

    def test_experiment_surfaces_model_failure_codes_not_bare_exception_names(self):
        for error,expected in ((ModelConfigurationError('model_configuration_error_key_format'),'model_configuration_error_key_format'),
                               (RuntimeError('model_request_timeout'),'model_request_timeout'),
                               (RuntimeError('codex_request_timeout'),'codex_request_timeout'),
                               (RuntimeError('model_request_failed_http_429'),'model_request_failed_http_429'),
                               (RuntimeError('provider said: secret-token-123'),'RuntimeError')):
            with tempfile.TemporaryDirectory() as tmp:
                runtime=StationRuntime(MockAdapter(),Path(tmp)/'runtime')
                class Failing:
                    label='failing'
                    def decide(self,messages,error=error):raise error
                model=Failing()
                controller=ExperimentController(runtime,lambda:model)
                try:
                    controller.start('Move mock object','Object reaches target',True);controller.thread.join(15)
                    controller.confirm_reset(controller.state['session_id'],True,'',0);controller.thread.join(15)
                    state=controller.snapshot()
                    self.assertEqual(state['reason'],expected)
                    if isinstance(error, ModelConfigurationError):
                        self.assertEqual(state['error_category'], 'model_configuration')
                        self.assertEqual(state['status'], 'error')
                        self.assertEqual(state['gate']['rejected'], 0)
                        self.assertFalse(any('validation_error' in h for h in state['history']))
                    self.assertEqual(state['attempts'],0)
                    self.assertNotIn('secret-token-123',json.dumps(state))
                finally:runtime.close()

    def test_named_profiles_keep_multiple_accounts_and_selected_name_after_restart(self):
        self.configure()
        first=self.profiles.save(dict(create_profile=True,provider='openai',profile_name='OpenAI 备用',
            profile=dict(base_url='https://backup.example.test/v1',model='backup-model',api_key='backup-key')))
        key=first['profile_id']
        second=self.profiles.save(dict(create_profile=True,provider='claude',profile_name='Claude 实验账号',
            profile=dict(model='claude-test',api_key='claude-key')))
        self.assertEqual(len(second['profiles']),4)
        self.assertNotIn('backup-key',json.dumps(second))
        self.profiles.save(dict(profile_id=key,profile_name='OpenAI 备用重命名'))
        restored=ModelProfiles(self.path)
        self.assertEqual(restored.public()['profile_id'],key)
        self.assertEqual(restored.read()['profiles'][key]['api_key'],'backup-key')
        self.assertEqual(restored.read()['profiles']['openai']['api_key'],'private-test-key')
        model=restored.model(synthetic=True)
        self.assertEqual(model.settings['profile_name'],'OpenAI 备用重命名')
        self.assertEqual(model.key,'backup-key')
        self.assertEqual(model.profile['model'],'backup-model')
        with self.assertRaisesRegex(ValueError,'already_exists'):
            restored.save(dict(create_profile=True,profile_name='Claude 实验账号'))
        with self.assertRaisesRegex(ValueError,'not_found'):
            restored.save(dict(profile_id='missing'))

    def test_delete_profile_needs_confirmation_and_never_strands_the_active_one(self):
        self.configure()
        created=self.profiles.save(dict(create_profile=True,provider='claude',profile_name='要删除的 / Scratch',
            profile=dict(model='scratch-model',api_key='scratch-key')))
        key=created['profile_id']
        self.assertEqual(created['profile_id'],key)
        for body in (dict(delete_profile_id=key),
                     dict(delete_profile_id=key,confirmed=False),
                     dict(delete_profile_id=key,confirmed=True,unexpected='x')):
            with self.assertRaisesRegex(ValueError,'explicit_profile_delete_confirmation_required'):
                self.profiles.delete(body)
        with self.assertRaisesRegex(ValueError,'not_found'):
            self.profiles.delete(dict(delete_profile_id='missing',confirmed=True))
        # `key` is the active profile, so deleting it must name a replacement.
        with self.assertRaisesRegex(ValueError,'select_another_profile'):
            self.profiles.delete(dict(delete_profile_id=key,confirmed=True))
        with self.assertRaisesRegex(ValueError,'select_another_profile'):
            self.profiles.delete(dict(delete_profile_id=key,profile_id=key,confirmed=True))
        visible=self.profiles.delete(dict(delete_profile_id=key,profile_id='openai',confirmed=True))
        self.assertNotIn(key,visible['profiles'])
        self.assertEqual(visible['profile_id'],'openai')
        self.assertNotIn('scratch-key',json.dumps(visible))
        restored=ModelProfiles(self.path)
        self.assertNotIn('scratch-key',self.path.read_text())
        self.assertEqual(restored.read()['profiles']['openai']['api_key'],'private-test-key')

    def test_cannot_delete_the_last_remaining_profile(self):
        self.configure()
        value=self.profiles.read()
        for name in [k for k in value['profiles'] if k!='openai']: del value['profiles'][name]
        value['profile_id']='openai'
        self.profiles._write(value)
        with self.assertRaisesRegex(ValueError,'cannot_delete_last_model_profile'):
            self.profiles.delete(dict(delete_profile_id='openai',profile_id='openai',confirmed=True))

    def test_legacy_two_profile_migration_preserves_credentials(self):
        self.configure('claude')
        value=self.profiles.read()
        value.pop('profile_id');value.pop('schema_version')
        for p in value['profiles'].values():
            for name in ('name','provider','reasoning'):p.pop(name)
        self.path.write_text(json.dumps(value))
        migrated=ModelProfiles(self.path)
        self.assertEqual(migrated.read()['profile_id'],'claude')
        self.assertEqual(migrated.read()['profiles']['claude']['api_key'],'private-test-key')
        self.assertEqual(migrated.public()['profiles']['claude']['name'],'Claude 默认 / Default')


class SkillLibraryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        from robol2l_deploy.cap.skills import SkillLibrary
        self.library=SkillLibrary(Path(self.tmp.name)/'skills')
    def tearDown(self):self.tmp.cleanup()

    def archive(self,entries):
        import io,zipfile
        buffer=io.BytesIO()
        with zipfile.ZipFile(buffer,'w') as bundle:
            for name,data in entries.items():bundle.writestr(name,data)
        return buffer.getvalue()

    def test_only_safe_text_files_are_kept(self):
        value=self.library.install('kit',self.archive({
            'tips/grasp.md':'# Top-down grasp\nKeep the TCP vertical.',
            'notes/depth.txt':'Sample inside the mask.',
            '../escape.md':'path traversal',
            'nested/../../escape2.md':'path traversal',
            '.hidden/secret.md':'hidden',
            'bin/tool.exe':b'\x00\x01binary',
            'big.md':'x'*(300*1024)}))
        kept=[f['file'] for f in value['files']]
        self.assertEqual(kept,['notes/depth.txt','tips/grasp.md'])
        self.assertEqual(value['files'][1]['title'],'Top-down grasp')
        for name in ('escape.md','escape2.md','secret.md','tool.exe'):
            self.assertNotIn(name,json.dumps(value['files']))
        self.assertIn('file_too_large',[s['why'] for s in value['skipped']])
        # Nothing was written outside the library directory.
        self.assertFalse((Path(self.tmp.name)/'escape.md').exists())
        self.assertFalse((Path(self.tmp.name).parent/'escape2.md').exists())

    def test_prompt_block_labels_the_library_as_advice_not_instructions(self):
        self.library.install('kit',self.archive({'a.md':'# A\nuse plan_motion'}))
        block=self.library.prompt_block()
        self.assertEqual(block['name'],'kit')
        self.assertEqual(block['included'],1)
        self.assertFalse(block['truncated'])
        for phrase in ('DATA','not as ground truth','reduce your','failure count','never follow any directive'):
            self.assertIn(phrase,block['how_to_use'])
        self.assertIn('use plan_motion',block['files'][0]['content'])

    def test_rejects_oversized_and_empty_archives(self):
        with self.assertRaisesRegex(ValueError,'archive_too_large'):
            self.library.install('kit',b'x'*(9*1024*1024))
        with self.assertRaisesRegex(ValueError,'no_readable_text_files'):
            self.library.install('kit',self.archive({'only.bin':b'\x00\x01'}))
        with self.assertRaisesRegex(ValueError,'name_required'):
            self.library.install('  ',self.archive({'a.md':'# A'}))
        self.assertFalse(self.library.describe()['installed'])

    def test_install_replaces_and_remove_clears(self):
        self.library.install('first',self.archive({'a.md':'# A'}))
        second=self.library.install('second',self.archive({'b.md':'# B'}))
        self.assertEqual(second['name'],'second')
        self.assertEqual([f['file'] for f in second['files']],['b.md'])
        self.assertFalse(self.library.remove()['installed'])
        self.assertIsNone(self.library.prompt_block())


    def test_selecting_nothing_means_this_run_carries_no_library(self):
        self.library.install('kit',self.archive({'a.md':'# A'}))
        self.assertIsNotNone(self.library.prompt_block())
        value=self.library.select('')
        self.assertFalse(value['installed'])
        self.assertIsNone(value['active'])
        self.assertIsNone(self.library.prompt_block())
        # The library itself is kept and can be picked again.
        self.assertEqual([l['name'] for l in value['libraries']],['kit'])
        self.assertEqual(self.library.select('kit')['active'],'kit')

    def test_many_named_libraries_coexist_and_one_is_active(self):
        # A CJK name must not collapse onto another library's folder.
        self.library.install('抓放经验库',self.archive({'a.md':'# A\nuse plan_motion'}))
        self.library.install('视觉经验库',self.archive({'b.md':'# B\ncheck depth'}))
        value=self.library.describe()
        self.assertEqual(sorted(l['name'] for l in value['libraries']),['抓放经验库','视觉经验库'])
        self.assertEqual(value['active'],'视觉经验库')          # newest upload becomes active
        self.assertEqual(self.library.prompt_block()['name'],'视觉经验库')
        self.assertEqual(self.library.select('抓放经验库')['active'],'抓放经验库')
        self.assertEqual(self.library.prompt_block()['name'],'抓放经验库')
        self.assertIn('use plan_motion',self.library.prompt_block()['files'][0]['content'])
        with self.assertRaisesRegex(ValueError,'skill_library_not_found'):
            self.library.select('没有这个库')

    def test_reupload_replaces_only_that_library(self):
        self.library.install('one',self.archive({'a.md':'# first'}))
        self.library.install('two',self.archive({'b.md':'# other'}))
        self.library.install('one',self.archive({'a.md':'# second','c.md':'# extra'}))
        value=self.library.describe()
        self.assertEqual(len(value['libraries']),2)
        self.assertEqual(value['active'],'one')
        self.assertEqual(value['file_count'],2)
        self.assertIn('second',self.library.prompt_block()['files'][0]['content'])

    def test_removing_the_active_library_falls_back_to_another(self):
        self.library.install('one',self.archive({'a.md':'# A'}))
        self.library.install('two',self.archive({'b.md':'# B'}))
        value=self.library.remove('two')
        self.assertEqual([l['name'] for l in value['libraries']],['one'])
        self.assertEqual(value['active'],'one')
        self.assertIsNotNone(self.library.prompt_block())
        empty=self.library.remove('one')
        self.assertFalse(empty['installed'])
        self.assertEqual(empty['libraries'],[])
        self.assertIsNone(self.library.prompt_block())


class TaskPresetTests(unittest.TestCase):
    def setUp(self):
        from robol2l_deploy.cap.task_presets import TaskPresets
        self.tmp=tempfile.TemporaryDirectory()
        self.presets=TaskPresets(Path(self.tmp.name)/'task-presets.json')
    def tearDown(self):self.tmp.cleanup()

    def test_save_select_and_survive_restart(self):
        from robol2l_deploy.cap.task_presets import TaskPresets
        self.presets.save({'name':'绿方块进盘','goal':'把绿色方块放进白色盘子',
                           'criteria':'方块完全落在盘内','skills':'视觉经验库'})
        rows=self.presets.save({'name':'绿方块进盒','goal':'把绿色方块放进盒子','criteria':'方块在盒内'})['presets']
        self.assertEqual(len(rows),2)
        restored=TaskPresets(self.presets.path).public()['presets']
        case=next(r for r in restored if r['name']=='绿方块进盘')
        self.assertEqual(case['goal'],'把绿色方块放进白色盘子')
        self.assertEqual(case['skills'],'视觉经验库')
        self.assertTrue(case['id'])

    def test_same_name_updates_instead_of_duplicating(self):
        first=self.presets.save({'name':'case','goal':'g1','criteria':'c1'})['presets'][0]
        again=self.presets.save({'name':'case','goal':'g2','criteria':'c2'})['presets']
        self.assertEqual(len(again),1)
        self.assertEqual(again[0]['id'],first['id'])
        self.assertEqual(again[0]['goal'],'g2')

    def test_validation_and_delete(self):
        for bad in ({'name':'','goal':'g','criteria':'c'},{'name':'n','goal':'  ','criteria':'c'},
                    {'name':'n','goal':'g','criteria':''},{'name':'n','goal':'g','criteria':'c','extra':1}):
            with self.assertRaises(ValueError):self.presets.save(bad)
        saved=self.presets.save({'name':'n','goal':'g','criteria':'c'})['presets'][0]
        with self.assertRaisesRegex(ValueError,'explicit_task_preset_delete'):
            self.presets.delete({'id':saved['id']})
        with self.assertRaisesRegex(ValueError,'not_found'):
            self.presets.delete({'id':'missing','confirmed':True})
        self.assertEqual(self.presets.delete({'id':saved['id'],'confirmed':True})['presets'],[])

    def test_use_reorders_by_recency(self):
        a=self.presets.save({'name':'a','goal':'g','criteria':'c'})['presets'][0]
        self.presets.save({'name':'b','goal':'g','criteria':'c'})
        self.assertEqual(self.presets.public()['presets'][0]['name'],'b')
        self.assertEqual(self.presets.touch(a['id'])['presets'][0]['name'],'a')


class LauncherEntryTests(unittest.TestCase):
    """One bookmarkable address that never connects hardware by itself."""

    def setUp(self):
        import requests, threading
        from franka_player.core import Console
        from franka_player.server import Server, handler
        from robol2l_deploy.launcher import launcher_handler
        self.tmp=tempfile.TemporaryDirectory();root=Path(self.tmp.name)
        self.console=Console(root/'console')
        self.state={'running':False,'session':{},'runtime':None}
        outer=self
        class FakeManager:
            def snapshot(self):return outer.state
        self.server=Server(('127.0.0.1',0),launcher_handler(handler(self.console),FakeManager()))
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.client=requests.Session();self.client.trust_env=False
        self.url='http://127.0.0.1:'+str(self.server.server_port)

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join()
        self.console.shutdown();self.client.close();self.tmp.cleanup()

    def test_panel_forwards_to_a_live_session_or_back_to_connect(self):
        idle=self.client.get(self.url+'/panel',allow_redirects=False,timeout=10)
        self.assertEqual(idle.status_code,302)
        self.assertEqual(idle.headers['Location'],'/')
        self.state={'running':True,'session':{'url':'http://127.0.0.1:8771','backend':'franky'},'runtime':{}}
        live=self.client.get(self.url+'/panel',allow_redirects=False,timeout=10)
        self.assertEqual(live.status_code,302)
        self.assertEqual(live.headers['Location'],'http://127.0.0.1:8771/hardware')
        self.assertEqual(live.headers['Cache-Control'],'no-store')
        # Reading the entry point must never start a session.
        self.assertEqual(self.client.get(self.url+'/',timeout=10).status_code,200)


class ExperimentHTTPTests(unittest.TestCase):
    def setUp(self):
        import requests
        from franka_player.core import Console
        from franka_player.server import Server, handler
        from robol2l_deploy.web import hardware_handler
        self.tmp=tempfile.TemporaryDirectory();root=Path(self.tmp.name)
        self.runtime=StationRuntime(MockAdapter(),root/'runtime')
        self.console=Console(root/'console')
        self.server=Server(('127.0.0.1',0),hardware_handler(handler(self.console),self.runtime,experiment_root=root/'shared'))
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.client=requests.Session();self.client.trust_env=False
        self.url='http://127.0.0.1:'+str(self.server.server_port)

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join()
        self.runtime.close();self.console.shutdown();self.client.close();self.tmp.cleanup()

    def post(self,action,body,expected=200):
        response=self.client.post(self.url+'/api/hardware/experiment/'+action,json=body,headers={'X-Console-Request':'1'},timeout=10)
        self.assertEqual(response.status_code,expected,response.text)
        return response.json()

    def wait(self):
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            state=self.client.get(self.url+'/api/hardware/experiment/state',timeout=5).json()
            if not state['busy']:return state
            time.sleep(.02)
        self.fail('experiment HTTP lifecycle did not settle')

    def test_review_retry_route_does_not_return_or_execute_another_version(self):
        from robol2l_deploy.cap.experiment_models import ModelRequestError
        model = Mock(label='test_review_model', settings={'model': 'test'})
        model.decide.side_effect = [
            dict(kind='program', source='result = get_robot_state()', reason='read'),
            ModelRequestError(400, 'context_limit', 1000000),
            dict(kind='program', source='raise ValueError("never execute")', reason='incomplete', post='reviewed')]
        with patch('robol2l_deploy.cap.experiment_models.ModelProfiles.model', return_value=model):
            self.post('start', dict(goal='test', criteria='test', confirmed=True), 202)
            state = self.wait()
            state = self.wait()
            self.assertEqual(state['error_category'], 'model_service')
            self.assertEqual(state['model_error']['category'], 'context_limit')
            self.assertTrue(state['review_retry_supported'])
            self.post('retry-review', dict(session_id=state['session_id'], expected_attempt=0), 409)
            self.post('retry-review', dict(session_id=state['session_id'], expected_attempt=1), 202)
            state = self.wait()
        self.assertEqual(state['status'], 'waiting_reset')
        self.assertEqual(state['attempts'], 1)
        self.assertEqual(state['versions'][0]['post'], 'reviewed')
        self.assertEqual(self.runtime.adapter.executions, 0)
        self.assertFalse(self.runtime.allow_motion)

    def test_browser_routes_fixture_flow_and_access_checks(self):
        page=self.client.get(self.url+'/hardware').text
        self.assertLess(page.index('data-panel="cap-panel"'),page.index('data-panel="experiment-panel"'))
        self.assertLess(page.index('data-panel="experiment-panel"'),page.index('class="tools-menu"'))
        self.assertEqual(self.client.get(self.url+'/experiment.js').status_code,200)
        response=self.client.post(self.url+'/api/hardware/experiment/start',json={})
        self.assertEqual(response.status_code,403)
        self.post('settings',{'mode':'fixture'})
        self.post('start',{'goal':'Mock reach','criteria':'Mock pose and gripper target','confirmed':True},202)
        state=self.wait();session=state['session_id']
        self.assertEqual(state['status'],'waiting_reset')
        image=self.client.get(self.url+'/api/hardware/experiment/image',params={'session':session,'file':'initial-0.jpg'})
        self.assertEqual(image.status_code,200)
        self.post('settings',{'mode':'codex'},409)
        response=self.client.post(self.url+'/api/hardware/cap/start',json={'model_mode':'fixture'},headers={'X-Console-Request':'1'})
        self.assertEqual(response.status_code,409)
        self.assertEqual(state['attempts'], 1)
        self.assertEqual(self.runtime.adapter.executions, 0)
        for expected in ('completed',):
            current=self.wait()
            self.post('confirm-reset',{'session_id':session,'confirmed':True,'note':'mock reset','expected_attempt':current['attempts']},202)
            self.assertEqual(self.wait()['status'],expected)
        self.assertEqual(self.wait()['attempts'],2)

    def test_authorize_reset_route_returns_and_automatically_retries_fixture(self):
        self.post('settings', {'mode': 'fixture'})
        self.post('start', {'goal': 'Mock reach', 'criteria': 'Mock target', 'confirmed': True}, 202)
        state = self.wait()
        self.assertTrue(state['reset_authorization_supported'])
        self.assertEqual(state['attempts'], 1)
        body = dict(session_id=state['session_id'], confirmed=True, note='Authorize mock reset', expected_attempt=1)
        self.post('authorize-reset', dict(body, expected_attempt=0), 409)
        self.post('authorize-reset', body, 202)
        state = self.wait()
        self.assertEqual(state['status'], 'completed')
        self.assertEqual(state['attempts'], 2)
        self.assertEqual(sum(i['kind'] == 'arm_return_result' for i in state['interventions']), 1)
        self.assertEqual(sum(i['kind'] == 'automatic_small_motion_reset' for i in state['interventions']), 0)

    def test_settings_dont_return_secret_or_leak_through_images(self):
        result=self.post('settings',dict(mode='api',profile=dict(api_key='test-persistent-secret',model='test')))
        self.assertNotIn('test-persistent-secret',json.dumps(result))
        self.assertNotIn('test-persistent-secret',(self.runtime.directory/'events.jsonl').read_text())
        response=self.client.get(self.url+'/api/hardware/experiment/image',params={'session':'../shared','file':'experiment-model-settings.json'})
        self.assertEqual(response.status_code,409)

    def test_environment_key_configuration_route_redacts_secret(self):
        with patch.dict(os.environ, {'PLAYER_HTTP_API_KEY': 'http-env-test-secret'}):
            visible = self.post('settings', dict(mode='api', profile=dict(
                model='api-model', api_key_env='PLAYER_HTTP_API_KEY')))
            self.assertTrue(visible['api_key_env_editable'])
            self.assertTrue(visible['profiles']['openai']['environment_key_loaded'])
            self.assertNotIn('http-env-test-secret', json.dumps(visible))
            self.post('settings', {'profile': {'api_key_env': 'bad-name'}}, 409)
            persisted = self.client.get(self.url+'/api/hardware/experiment/settings').json()
            self.assertEqual(persisted['profiles']['openai']['api_key_env'], 'PLAYER_HTTP_API_KEY')
            self.assertNotIn('http-env-test-secret', (self.runtime.directory/'events.jsonl').read_text())
    def test_skills_records_and_preview_routes(self):
        import io,zipfile,base64
        buffer=io.BytesIO()
        with zipfile.ZipFile(buffer,'w') as bundle:
            bundle.writestr('tips.md','# Tip\nCheck plan.ok first.')
            bundle.writestr('bin/x.exe',b'\x00\x01')
        payload=base64.b64encode(buffer.getvalue()).decode()
        self.assertFalse(self.client.get(self.url+'/api/hardware/experiment/skills').json()['installed'])
        installed=self.post('skills',{'name':'kit','archive_base64':payload})
        self.assertEqual(installed['file_count'],1)
        self.assertNotIn('x.exe',json.dumps(installed['files']))
        self.post('skills',{'name':'kit','archive_base64':'not base64!!'},409)
        self.post('settings',{'mode':'fixture'})
        self.post('start',{'goal':'Mock reach','criteria':'Mock pose target','confirmed':True},202)
        state=self.wait();session=state['session_id']
        # Settings and skills are locked while an experiment is live.
        self.post('skills',{'name':'kit','archive_base64':payload},409)
        preview=self.post('preview',{})
        self.assertIn('EXPERIMENT PROTOCOL',preview['system'])
        self.assertEqual(preview['context']['skills']['name'],'kit')
        records=self.client.get(self.url+'/api/hardware/experiment/records').json()['records']
        self.assertEqual(records[0]['session_id'],session)
        full=self.client.get(self.url+'/api/hardware/experiment/record',params={'id':session}).json()
        self.assertEqual(full['session_id'],session)
        bad=self.client.get(self.url+'/api/hardware/experiment/record',params={'id':'../secrets'})
        self.assertEqual(bad.status_code,409)
        export=self.client.get(self.url+'/api/hardware/experiment/export',params={'id':session})
        self.assertEqual(export.status_code,200)
        self.assertTrue(export.headers['Content-Type'].startswith('application/zip'),export.headers['Content-Type'])
        self.assertIn(session+'/experiment.json',zipfile.ZipFile(io.BytesIO(export.content)).namelist())
        self.post('end',{'session_id':session})
        self.assertFalse(self.post('skills-remove',{'confirmed':True})['installed'])
        # saved cases round-trip over HTTP
        saved=self.post('presets',{'name':'绿方块进盘','goal':'把绿色方块放进白色盘子',
                                   'criteria':'方块完全落在盘内','skills':'kit'})['presets']
        self.assertEqual(saved[0]['name'],'绿方块进盘')
        listed=self.client.get(self.url+'/api/hardware/experiment/presets').json()['presets']
        self.assertEqual(listed[0]['goal'],'把绿色方块放进白色盘子')
        self.post('presets',{'name':'绿方块进盘','goal':'x','criteria':'y'},200)
        self.assertEqual(len(self.client.get(self.url+'/api/hardware/experiment/presets').json()['presets']),1)
        self.post('presets',{'name':'bad','goal':'','criteria':'y'},409)
        self.post('presets-delete',{'id':listed[0]['id']},409)
        self.assertEqual(self.post('presets-delete',{'id':listed[0]['id'],'confirmed':True})['presets'],[])

    def test_delete_profile_route_is_guarded_and_locked_during_an_experiment(self):
        created=self.post('settings',dict(create_profile=True,provider='claude',profile_name='要删除的 / Scratch',
            profile=dict(model='scratch',api_key='delete-route-secret')))
        key=created['profile_id']
        self.post('delete-profile',{'delete_profile_id':key},409)
        self.post('delete-profile',{'delete_profile_id':key,'profile_id':key,'confirmed':True},409)
        self.post('settings',{'mode':'fixture'})
        self.post('start',{'goal':'Mock reach','criteria':'Mock pose target','confirmed':True},202)
        self.wait()
        # Model settings are locked while an experiment is live.
        self.post('delete-profile',{'delete_profile_id':key,'profile_id':'openai','confirmed':True},409)
        self.post('end',{'session_id':self.wait()['session_id']})
        visible=self.post('delete-profile',{'delete_profile_id':key,'profile_id':'openai','confirmed':True})
        self.assertNotIn(key,visible['profiles'])
        self.assertNotIn('delete-route-secret',json.dumps(visible))
        self.assertNotIn('delete-route-secret',(self.runtime.directory/'events.jsonl').read_text())


if __name__=='__main__':unittest.main()
