"""Decision feedback survives model history and UI budgets without hardware calls."""
from copy import deepcopy
import json
import unittest

from robol2l_deploy.cap.prompt_history import bounded, feedback_summary, prompt_history, size, HISTORY_BYTES


def geometry(x):
    return dict(pose=dict(position=[x, -.627, .02], quaternion_wxyz=[1, 0, 0, 0], frame='left_base'),
                extents=[.09, .06, .05], point_count=1400)


def localization(index):
    return dict(id=f'localize-{index}', tool='localize_object', ok=True, observation_id='frozen-image',
        inputs=dict(args=['fruit'], kwargs=dict(camera_name='global', target_frame='left_base')),
        output=dict(ok=True, error=None, geometry=geometry(.48 + index*.001),
            segmentation=dict(score=.93-index*.01, label='query only', mask={'omitted':'array'},
                              observation_id='frozen-image'),
            diagnostics=dict(class_identity_verified=False, raw_scores=[.001]*2000)))


def ik_call(index):
    ok = index == 4  # Success in the middle must not be lost among failures.
    return dict(id=f'ik-{index}', tool='solve_ik', ok=ok, observation_id='frozen-image',
        inputs=dict(args=[dict(position=[.4+index*.001, -.3, .1], quaternion_wxyz=[0, 1, 0, 0], frame='left_base')],
                    kwargs=dict(arm='right', seed_joints=[0]*6)),
        output=dict(ok=ok, joint_positions=[.1]*6 if ok else None, arm='right',
            error=None if ok else dict(code='ik_failed', message='arx_ik_did_not_converge', recoverable=True),
            diagnostics=dict(sdk_converged=ok, sdk_residual=.00009 if ok else .1+index,
                             residual_semantics='native_solver_residual_not_position_error_m')))


def history_for(calls):
    feedback = dict(ok=True, error=None, calls=calls, result={'full_objects':[c['output'] for c in calls]},
                    stdout='大型数组和日志\\\n'*20000, physical_task_success=False)
    return [dict(version='v02', source='print(result)', feedback=feedback),
            dict(phase='review', decision={'kind':'program', 'reason':'use geometry'}),
            dict(reset_to_initial={'command_completed':True, 'verified':None}),
            dict(human_intervention={'message':'continue'})]


class KeyResultsTests(unittest.TestCase):
    def test_all_candidate_geometry_survives_review_next_generation_and_ui(self):
        calls=[localization(i) for i in range(4)]
        history=history_for(calls);before=deepcopy(history)
        for entries in (history[:1], history):
            output=prompt_history(entries)
            rows=output[0]['feedback']['key_results']
            self.assertEqual(len(rows),4)
            for index,row in enumerate(rows):
                self.assertEqual(row['geometry'],calls[index]['output']['geometry'])
                self.assertEqual(row['id'],f'localize-{index}')
                self.assertEqual(row['observation_id'],'frozen-image')
                self.assertFalse(row['diagnostics']['class_identity_verified'])
            self.assertLessEqual(size(output),HISTORY_BYTES)
        ui=bounded(feedback_summary(history[0]['feedback']),24000)
        self.assertEqual([r['geometry'] for r in ui['key_results']], [c['output']['geometry'] for c in calls])
        self.assertEqual(history,before)

    def test_every_ik_target_and_native_residual_survives_with_middle_success(self):
        calls=[ik_call(i) for i in range(10)]
        output=prompt_history(history_for(calls))
        rows=output[0]['feedback']['key_results']
        self.assertEqual(len(rows),10)
        for index,row in enumerate(rows):
            self.assertEqual(row['request']['target'],calls[index]['inputs']['args'][0])
            self.assertEqual(row['request']['arm'],'right')
            self.assertEqual(row['ok'],index==4)
            self.assertEqual(row['diagnostics'],calls[index]['output']['diagnostics'])
        self.assertEqual(rows[4]['joint_positions'],[.1]*6)
        self.assertEqual(rows[3]['error']['code'],'ik_failed')

    def test_excess_results_are_omitted_whole_with_attribution(self):
        calls=[ik_call(i) for i in range(200)]
        original=feedback_summary(dict(calls=calls))['key_results']
        output=bounded(dict(key_results=original, stdout='x'*50000),4000)
        excerpt=output['key_results']
        self.assertTrue(excerpt['_excerpt'])
        self.assertGreater(excerpt['omitted_items'],0)
        self.assertEqual(excerpt['total_items'],len(original))
        self.assertEqual(excerpt['omitted_items']+len(excerpt['samples']),len(original))
        self.assertLessEqual(size(output),4000)
        for record in excerpt['samples']:
            self.assertEqual(record,original[record['call_index']])
        self.assertNotIn(' … [omitted',json.dumps(excerpt))

    def test_point_candidates_keep_middle_candidate_and_error_is_not_success(self):
        call=dict(tool='segment_points',ok=True,inputs=dict(args=['global',[[10,20]]]),
            output=dict(ok=True,segmentations=[dict(label='point',score=s,observation_id='image') for s in [.9,.8,.2]]))
        failed=localization(0);failed['ok']=False
        failed['output']=dict(ok=False,error=dict(code='no_segmentation',recoverable=True),geometry=None,
                              diagnostics=dict(rejection_reason='no_trusted_candidates',text_confidence_threshold=.5))
        rows=feedback_summary(dict(calls=[call,failed]))['key_results']
        self.assertEqual([c['candidate_index'] for c in rows[0]['candidates']],[0,1,2])
        self.assertEqual(rows[0]['request']['points'],[[10,20]])
        self.assertFalse(rows[1]['ok'])
        self.assertNotIn('geometry',rows[1])
        self.assertEqual(rows[1]['error']['code'],'no_segmentation')
