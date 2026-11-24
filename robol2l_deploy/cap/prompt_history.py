"""Bound model input independently of the immutable, full execution evidence."""
from collections import Counter
import json
import math

HISTORY_BYTES = 160_000
MOTION = {'execute_trajectory', 'set_gripper', 'set_grippers', 'open_gripper',
          'close_gripper', 'step', 'go_home', 'move_to_pose', 'move_to_joints', 'move_synchronized'}
PRIORITY = ('version', 'phase', 'ok', 'error', 'code', 'message', 'recoverable',
            'outcome', 'status', 'physical_task_success', 'feedback', 'key_results',
            'call_summary', 'result', 'calls', 'post', 'reason', 'source')


def size(value):
    return len(json.dumps(value, ensure_ascii=False).encode('utf-8'))


def whole_records(records, budget):
    """Keep complete, attributable results; never turn their numbers into fragments."""
    if size(records) <= budget:
        return records
    selected = {}
    result = dict(_excerpt=True, total_items=len(records), omitted_items=len(records), samples=[])
    # Preserve both early and recent evidence. Every retained record has its call index.
    for index in dict.fromkeys(i for pair in zip(range(len(records)), reversed(range(len(records)))) for i in pair):
        candidate = {**selected, index: records[index]}
        excerpt = dict(_excerpt=True, total_items=len(records),
                       omitted_items=len(records)-len(candidate),
                       samples=[candidate[k] for k in sorted(candidate)])
        if size(excerpt) <= budget:
            selected, result = candidate, excerpt
    return result


def bounded(value, budget):
    """JSON-serializable excerpts with explicit omissions, never silent success."""
    if size(value) <= budget:
        return value
    if isinstance(value, dict):
        keys = sorted(value, key=lambda k: PRIORITY.index(k) if k in PRIORITY else len(PRIORITY))
        result = {'_excerpt': 'Partial evidence; omitted fields do not imply success.'}
        for index, key in enumerate(keys):
            remaining = budget - size(result) - size(key) - 8
            if remaining < 180:
                result['_omitted_fields'] = len(keys) - index
                break
            # Reserve space for later fields while keeping important diagnostics first.
            if key == 'feedback' and isinstance(value[key], dict):
                # Source/stdout/raw arrays must not consume the space for tool feedback.
                allowance = max(180, remaining * 4 // 5)
            elif key == 'key_results' and isinstance(value[key], list):
                result[key] = whole_records(value[key], max(180, remaining * 3 // 4))
                continue
            else:
                allowance = min(remaining, max(180, remaining // min(4, len(keys) - index)))
            result[key] = bounded(value[key], allowance)
        return result
    if isinstance(value, list):
        count = min(8, len(value))
        indexes = sorted(set(list(range((count + 1) // 2)) + list(range(len(value) - count // 2, len(value)))))
        allowance = max(50, (budget - 160) // max(1, len(indexes)))
        return {'_excerpt': True, 'total_items': len(value),
                'samples': [bounded(value[i], allowance) for i in indexes]}
    text = value if isinstance(value, str) else json.dumps(value, ensure_ascii=False)
    raw = text.encode('utf-8')
    # Account for JSON escaping as well as UTF-8 bytes.
    excerpt = raw[:max(0, budget // 4 - 50)].decode('utf-8', errors='ignore')
    return excerpt + ' … [omitted; full value in archived evidence]'


def fields(value, names):
    return {key: value[key] for key in names if key in value} if isinstance(value, dict) else {}


def mask_summary(mask):
    return fields(mask, ('label', 'score', 'camera_name', 'frame', 'box_xyxy', 'observation_id'))


def key_results(calls):
    """Small decision data from every perception/IK call, independent of trace sampling.

    No candidate selection, target verification or new acceptance policy occurs here.
    Raw arrays, stdout and repeated diagnostics remain in the archived trace.
    """
    records = []
    for index, call in enumerate(calls):
        tool, output = call.get('tool'), call.get('output')
        if tool not in ('localize_object', 'estimate_geometry', 'segment_points', 'segment_text',
                        'solve_ik', 'plan_motion', 'plan_synchronized_motion'):
            continue
        row = dict(call_index=index, **fields(call, ('id', 'tool', 'line', 'ok', 'observation_id')))
        inputs = call.get('inputs') or {}
        args, kwargs = inputs.get('args') or [], inputs.get('kwargs') or {}
        row['request'] = fields(kwargs, ('arm', 'camera_name', 'target_frame', 'model_query',
                                         'seed_joints', 'target_pose', 'goal', 'goals', 'query', 'text'))
        if args:
            if tool in ('solve_ik', 'plan_motion', 'plan_synchronized_motion'):
                row['request']['target'] = args[0]
            elif tool == 'localize_object':
                row['request']['query'] = args[0]
            elif tool in ('segment_points', 'segment_text'):
                row['request']['camera_name'] = args[0]
                if len(args) > 1:
                    row['request']['points' if tool == 'segment_points' else 'text'] = args[1]
        if kwargs.get('segmentation') is not None:
            row['request']['segmentation'] = mask_summary(kwargs['segmentation'])
        if call.get('error'):
            row['error'] = fields(call['error'], ('code', 'type', 'message', 'recoverable'))
        if isinstance(output, dict):
            row.update(fields(output, ('ok', 'joint_positions', 'arm')))
            if output.get('error'):
                row['error'] = fields(output['error'], ('code', 'type', 'message', 'recoverable'))
            geometry = output if tool == 'estimate_geometry' else output.get('geometry')
            if isinstance(geometry, dict):
                row['geometry'] = fields(geometry, ('pose', 'extents', 'point_count'))
            if isinstance(output.get('segmentation'), dict):
                row['segmentation'] = mask_summary(output['segmentation'])
            if isinstance(output.get('segmentations'), list):
                row['candidates'] = [dict(candidate_index=i, **mask_summary(mask))
                                     for i, mask in enumerate(output['segmentations'])]
            diagnostics = output.get('diagnostics') or {}
            if tool == 'solve_ik':
                row['diagnostics'] = diagnostics  # Preserve native residuals on success AND failure.
            else:
                row['diagnostics'] = fields(diagnostics, ('original_query', 'model_query',
                    'selection_source', 'text_inference_performed', 'success_semantics',
                    'class_identity_verified', 'prompt_type', 'score_semantics',
                    'text_confidence_threshold', 'rejection_reason', 'accepted_count',
                    'duration_s', 'speed_factor', 'collision_checked', 'collision_missing',
                    'fk_position_residual_m', 'fk_rotation_residual_rad'))
        records.append(row)
    return records


def feedback_summary(feedback):
    calls = feedback.get('calls', [])
    result = {k: v for k, v in feedback.items() if k != 'calls'}
    result['key_results'] = key_results(calls)
    groups = {}
    for index, call in enumerate(calls):
        key = (call.get('tool'), call.get('ok'))
        group = groups.setdefault(key, [])
        group.append(index)
    selected = set()
    for indexes in groups.values():
        selected.update(indexes[:1] + indexes[-1:])
    failures = [i for i, call in enumerate(calls) if call.get('ok') is False or call.get('error')]
    motions = [i for i, call in enumerate(calls) if call.get('tool') in MOTION]
    selected.update(failures[:4] + failures[-4:] + motions[:4] + motions[-4:])
    result['calls'] = [bounded(calls[i], 2500) for i in sorted(selected)]
    result['call_summary'] = dict(total=len(calls), failed=len(failures), motion_calls=len(motions),
        by_tool=dict(Counter(c.get('tool', 'unknown') for c in calls)),
        sampled=len(selected), omitted=len(calls) - len(selected),
        evidence='Full chronological calls remain in this version/result.json; samples are not the entire trace.')
    errors = Counter()
    for i in failures:
        call = calls[i]
        output = call.get('output')
        error = call.get('error') or (output.get('error') if isinstance(output, dict) else None) or {}
        errors[str(error.get('code') or error.get('type') or 'unspecified_failure') if isinstance(error, dict) else 'unspecified_failure'] += 1
    result['call_summary']['error_counts'] = dict(errors)
    # Preserve motion/noise ranges across ALL polls, not merely the sampled calls.
    # This lets review distinguish stable encoders from velocity spikes without
    # declaring the arm safe or changing any native settling threshold.
    ranges = {}
    state_count = 0
    for call in calls:
        state = call.get('output')
        if call.get('tool') != 'get_robot_state' or not call.get('ok') or not isinstance(state, dict):
            continue
        state_count += 1
        for field in ('joint_positions', 'joint_velocities'):
            for arm, values in (state.get(field) or {}).items():
                if not isinstance(values, list) or not all(isinstance(v, (float, int)) and math.isfinite(v) for v in values):
                    continue
                entry = ranges.setdefault(field, {}).setdefault(arm, {'min': list(values), 'max': list(values)})
                if len(entry['min']) != len(values): continue
                entry['min'] = [min(a, b) for a, b in zip(entry['min'], values)]
                entry['max'] = [max(a, b) for a, b in zip(entry['max'], values)]
    if state_count:
        result['call_summary']['state_poll_ranges'] = dict(samples=state_count, **ranges)
    return result


def prompt_history(history):
    """Return a bounded copy. Never change stored history or physical receipts."""
    entries = []
    for entry in history:
        item = dict(entry)
        if isinstance(item.get('feedback'), dict):
            item['feedback'] = feedback_summary(item['feedback'])
        if isinstance(item.get('decision'), dict):
            # Executed source is already in the version's feedback entry.
            item['decision'] = {k: v for k, v in item['decision'].items() if k != 'source'}
        entries.append(item)
    # Keep recent detail; older entries remain explicitly summarized.
    output = []
    remaining = HISTORY_BYTES - 256
    recent_feedback = 0
    for index, item in enumerate(reversed(entries)):
        has_feedback = isinstance(item.get('feedback'), dict)
        # Review/reset records between execution and generation do not age out
        # the just-computed target geometry and IK results.
        detailed = index < 2 or (has_feedback and recent_feedback < 2)
        recent_feedback += has_feedback
        budget = min(24_000 if detailed else 8_000, remaining)
        if budget < 600:
            output.append({'_omitted_older_entries': len(entries) - index,
                           'evidence': 'Full history remains in experiment.json.'})
            break
        excerpt = bounded(item, budget)
        # Pathological dictionaries/escaping must still obey the hard budget.
        while size(excerpt) > budget:
            budget //= 2
            excerpt = bounded(item, budget)
        output.append(excerpt)
        remaining -= size(excerpt) + 2
    return list(reversed(output))
