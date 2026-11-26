"""Semantic feedback survives review, reset and the next request (software only)."""
from copy import deepcopy
import json
import unittest

from robol2l_deploy.cap.prompt_history import HISTORY_BYTES, prompt_history, size
from test_feedback_key_results import history_for, ik_call, localization


def motion(index, tool='execute_trajectory', ok=True):
    receipt = dict(dispatch='accepted', target_rad=.01, measured_rad=4.78770885786221,
                   closing_travel_rad=0., target_reached=None, physical_arrival_verified=False,
                   grasp_verified=False, contact_detected=None, completion='sdk_reference_finished')
    error = None if ok else dict(code='outcome_unknown', recoverable=False,
                                details=dict(side='right', last_receipt=receipt))
    return dict(id=f'motion-{index}', tool=tool, line=100+index, ok=ok,
                started_s=100.+index, finished_s=100.+index+.01,
                observation_id=f'frame-{index}', inputs=dict(args=[0.], kwargs=dict(arm='right')),
                output=dict(ok=ok, status='success' if ok else 'outcome_unknown', error=error,
                    terminated=False, truncated=False, final_errors=dict(left=0., right=.012),
                    diagnostics=dict(per_arm={'left': {'dispatch':'not_sent'}, 'right':receipt},
                        grippers=dict(right=receipt), physical_arrival_verified=False,
                        waypoint_receipts=[dict(waypoint=i, per_arm={'right':receipt}) for i in range(3)])))


def expanded(row):
    row = deepcopy(row)
    diagnostics = row.get('output', {}).get('diagnostics', {})
    for waypoint in diagnostics.get('waypoint_receipts', []):
        if waypoint.get('per_arm') == {'_same_as':'output.diagnostics.per_arm'}:
            waypoint['per_arm'] = deepcopy(diagnostics['per_arm'])
    return row


class MotionHistoryTests(unittest.TestCase):
    def test_every_motion_and_gripper_receipt_survives_not_only_first_and_last(self):
        calls = [localization(0), ik_call(4)] + [motion(i) for i in range(12)]
        calls.insert(8, motion(80, 'set_gripper'))
        calls.insert(10, motion(90, 'set_grippers', ok=False))
        history = history_for(calls)
        history[0]['source'] = '\n'.join(f'x = {i}' for i in range(1000)) + '\nif not target.ok:\n    result = "EARLY_EXIT"'
        before = deepcopy(history)
        for suffix in ([], history[1:]):
            output = prompt_history(history[:1] + suffix)
            feedback = output[0]['feedback']
            rows = {r['call_index']: r for r in feedback['key_results']}
            for index, call in enumerate(calls[2:], 2):
                self.assertEqual(expanded(rows[index]), dict(call_index=index, **call))
            self.assertIsNone(rows[8]['output']['diagnostics']['grippers']['right']['target_reached'])
            self.assertFalse(rows[8]['output']['diagnostics']['grippers']['right']['grasp_verified'])
            self.assertEqual(output[0]['source'], history[0]['source'])
            self.assertLessEqual(size(output), HISTORY_BYTES)
        self.assertEqual(history, before)

    def test_poll_storm_does_not_hide_close_to_lift_sequence_or_native_error(self):
        poll = dict(tool='get_robot_state', ok=True, output={'joint_positions':{'right':[0.]*6}})
        close = motion(0, 'close_gripper')
        lift = motion(1, ok=False)
        lift['started_s'] = close['finished_s'] + .05
        calls = [deepcopy(poll) for _ in range(3977)]
        calls[2000] = close
        calls[2002] = lift
        output = prompt_history(history_for(calls))[0]['feedback']
        states = output['state_samples']
        self.assertEqual([r['call_index'] for r in states],[0,1999,2001,2003,3976])
        rows = output['key_results']
        self.assertEqual([r['call_index'] for r in rows], [2000,2002])
        self.assertEqual(expanded(rows[0])['output'], close['output'])
        self.assertEqual(rows[1]['output']['error'], lift['output']['error'])
        self.assertAlmostEqual(rows[1]['started_s']-rows[0]['finished_s'], .05)
        self.assertEqual(output['call_summary']['state_poll_ranges']['samples'],3975)
        self.assertLess(size(output),24000)

    def test_all_error_details_including_non_motion_failures_are_available(self):
        call = motion(0, 'get_observation', ok=False)
        ik = ik_call(0)
        ik['output']['error']['details'] = {'sdk_residual': .141, 'sdk_converged':False}
        plan = deepcopy(ik);plan['tool']='plan_motion'
        rows = prompt_history(history_for([call, ik, plan]))[0]['feedback']['key_results']
        self.assertEqual(rows[0]['output']['error'],call['output']['error'])
        self.assertEqual(rows[1]['error']['details'],ik['output']['error']['details'])
        self.assertEqual(rows[2]['error']['details'],plan['output']['error']['details'])

    def test_overload_drops_whole_records_and_keeps_latest_source_and_all_posts(self):
        history = []
        for version in range(8):
            calls = [motion(i, 'set_gripper' if i%2 else 'execute_trajectory') for i in range(18)]
            history.extend([dict(version=f'v{version:02d}', source='x = 0\n'*1000+'result = "TAIL_CONDITION"',
                                 feedback=dict(ok=True, calls=calls)),
                            dict(phase='review', decision=dict(kind='program', post=f'lesson-{version}')),
                            dict(reset_to_initial={'command_completed':True,'diagnostics':'duplicate receipt '*300})])
        before = deepcopy(history)
        output = prompt_history(history)
        latest = next(e for e in output if e.get('version')=='v07')
        self.assertEqual(latest['source'],history[-3]['source'])
        self.assertEqual(len(latest['feedback']['key_results']),18)
        previous = next(e for e in output if e.get('version')=='v06')
        self.assertEqual(previous['source'],history[-6]['source'])
        self.assertEqual(len(previous['feedback']['key_results']),18)
        self.assertEqual([e['decision']['post'] for e in output if 'decision' in e],
                         [f'lesson-{i}' for i in range(8)])
        for entry in output:
            if 'feedback' not in entry:continue
            rows = entry['feedback'].get('key_results', [])
            if isinstance(rows,dict):
                self.assertEqual(rows['omitted_items']+len(rows['samples']),rows['total_items'])
                rows = rows['samples']
            for row in rows:
                i = int(entry['version'][1:])*3
                self.assertEqual(expanded(row),dict(call_index=row['call_index'], **history[i]['feedback']['calls'][row['call_index']]))
        self.assertLessEqual(size(output),HISTORY_BYTES)
        self.assertEqual(history,before)

    def test_duplicate_results_reference_records_and_keep_program_outcome(self):
        call=motion(0,'set_gripper')
        history=history_for([call])
        history[0]['feedback']['result']={'stage':'stopped_before_lift','task_complete':False,
                                         'records':[deepcopy(call['output'])]*1000}
        result=prompt_history(history)[0]['feedback']['result']
        self.assertEqual(result['stage'],'stopped_before_lift')
        self.assertFalse(result['task_complete'])
        self.assertNotIn(' … [omitted',json.dumps(result))

    def test_only_identical_waypoint_receipts_are_referenced_losslessly(self):
        call=motion(0)
        diagnostics=call['output']['diagnostics']
        diagnostics['waypoint_receipts']=[
            dict(waypoint=1, per_arm={'right':{'measured_rad':2.5}}),
            dict(waypoint=2, per_arm=deepcopy(diagnostics['per_arm']))]
        before=deepcopy(call)
        row=prompt_history(history_for([call]))[0]['feedback']['key_results'][0]
        actual=row['output']['diagnostics']['waypoint_receipts']
        self.assertEqual(actual[0],diagnostics['waypoint_receipts'][0])
        self.assertEqual(actual[1]['per_arm'],{'_same_as':'output.diagnostics.per_arm'})
        self.assertEqual(expanded(row),dict(call_index=0,**call))
        self.assertEqual(call,before)
