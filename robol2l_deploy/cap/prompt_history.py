"""Bound model input independently of the immutable, full execution evidence."""
from collections import Counter
from copy import deepcopy
import hashlib
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


def receipt_record(index, call):
    """Losslessly reference repeated final per-arm receipts within the same call."""
    row = deepcopy(dict(call_index=index, **call))
    output = row.get('output')
    diagnostics = output.get('diagnostics') if isinstance(output, dict) else None
    if isinstance(diagnostics, dict) and isinstance(diagnostics.get('per_arm'), dict):
        for waypoint in diagnostics.get('waypoint_receipts') or []:
            if isinstance(waypoint, dict) and waypoint.get('per_arm') == diagnostics['per_arm']:
                waypoint['per_arm'] = {'_same_as': 'output.diagnostics.per_arm'}
    return row


def key_results(calls):
    """Decision data independent of display sampling, including every actuation.

    No candidate selection, target verification or new acceptance policy occurs here.
    Raw arrays, stdout and repeated diagnostics remain in the archived trace.
    """
    records = []
    for index, call in enumerate(calls):
        tool, output = call.get('tool'), call.get('output')
        failed = call.get('ok') is False or bool(call.get('error'))
        if tool in MOTION or failed and tool not in (
                'localize_object', 'estimate_geometry', 'segment_points', 'segment_text',
                'solve_ik', 'plan_motion', 'plan_synchronized_motion'):
            # Public receipts are already serialized without image arrays by the
            # executor. Keep native semantics, both arms, errors and timestamps;
            # never infer arrival/grasp from ok or change false/None into success.
            records.append(receipt_record(index, call))
            continue
        if tool not in ('localize_object', 'estimate_geometry', 'segment_points', 'segment_text',
                        'solve_ik', 'plan_motion', 'plan_synchronized_motion'):
            continue
        row = dict(call_index=index, **fields(call, (
            'id', 'tool', 'line', 'ok', 'observation_id', 'started_s', 'finished_s')))
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
            row['error'] = call['error']
        if isinstance(output, dict):
            row.update(fields(output, ('ok', 'joint_positions', 'arm')))
            if output.get('error'):
                row['error'] = output['error']
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


def trace_selection(calls):
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
    return sorted(selected), failures, motions


def feedback_summary(feedback):
    calls = feedback.get('calls', [])
    result = {k: v for k, v in feedback.items() if k != 'calls'}
    result['key_results'] = key_results(calls)
    selected, failures, motions = trace_selection(calls)
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


def omitted(value):
    """Describe an omitted WHOLE value; never present a broken scalar as evidence."""
    encoded = json.dumps(value, ensure_ascii=False).encode('utf-8')
    return dict(_omitted=True, bytes=len(encoded), sha256=hashlib.sha256(encoded).hexdigest(),
                evidence='Whole value omitted from model input; retained in archive, not available as a live variable.')


def atomic_fields(value, budget):
    """Budget metadata by whole fields, not recursive fragments of receipts/source."""
    if size(value) <= budget:
        return value
    result = {}
    keys = sorted((k for k in value if k != '_omitted_fields'),
                  key=lambda k: (PRIORITY.index(k) if k in PRIORITY else len(PRIORITY), size(value[k])))
    prior = value.get('_omitted_fields', [])
    missing = list(prior) if isinstance(prior, list) else ['previously_omitted_fields']
    for index, key in enumerate(keys):
        candidate = {**result, key: value[key], '_omitted_fields': missing + keys[index+1:]}
        if size(candidate) <= budget:
            result[key] = value[key]
        else:
            missing.append(key)
    if missing:
        result['_omitted_fields'] = missing
    # Even pathological field names must not breach the request budget.
    if size(result) > budget:
        return {'_omitted_fields': len(value)}
    return result


def model_feedback(feedback):
    """Use structured records for the model and whole supplementary trace samples.

    The UI's recursive bounded() preview must never be applied to these records.
    Result/stdout are supplementary (often copies of the same public receipts).
    """
    summary = feedback_summary(feedback)
    rows = summary['key_results']
    indexed = {row['call_index'] for row in rows}
    calls = feedback.get('calls', [])
    def successful_poll(i):
        return (0 <= i < len(calls) and calls[i].get('tool') == 'get_robot_state'
                and calls[i].get('ok') is True and not calls[i].get('error'))
    # Isolated states and polling-run endpoints remain useful supplementary
    # evidence, especially the gripper state between close and lift. Never let
    # those repeated state objects displace actual command receipts or source.
    summary['state_samples'] = [dict(call_index=i, **call) for i, call in enumerate(calls)
        if successful_poll(i) and not (successful_poll(i-1) and successful_poll(i+1))]
    # Preserve small supplementary results, replacing exact duplicate tool
    # outputs by an attributable reference before shortening bulky diagnostics.
    outputs = {json.dumps(call['output'], sort_keys=True, ensure_ascii=False): i
               for i, call in enumerate(calls) if i in indexed and 'output' in call}

    def references(value):
        if isinstance(value, (dict, list)):
            index = outputs.get(json.dumps(value, sort_keys=True, ensure_ascii=False))
            if index is not None:
                return {'key_result_call_index': index}
        if isinstance(value, dict):
            return {k: references(v) for k, v in value.items()}
        if isinstance(value, list):
            return [references(v) for v in value]
        return value

    selected, _, _ = trace_selection(calls)
    # Preserve the existing small-feedback calls/result shapes. Supplementary
    # traces may be omitted wholesale under pressure, never recursively clipped.
    summary['calls'] = [receipt_record(i, calls[i]) for i in selected]
    summary['call_summary'] = dict(summary['call_summary'],
        structured_records_total=len(rows), state_samples_total=len(summary['state_samples']),
        evidence='key_results contains structured decision records; calls contains supplementary whole trace samples. '
                 'Repeated successful polls are summarized by state_poll_ranges. '
                 'Original chronological calls remain in result.json, not an injected file-reading tool.')
    if 'result' in summary:
        value = summary['result']
        if size(value) > 4000:
            # Keep the top-level outcome/error contract even when result itself
            # is a tool receipt. Only nested duplicate objects become references.
            value = {k: references(v) for k, v in value.items()} if isinstance(value, dict) else references(value)
        # Retain top-level program outcome fields such as stage/task_complete.
        summary['result'] = atomic_fields(value, 4000) if isinstance(value, dict) else (
            value if size(value) <= 4000 else omitted(value))
    if 'stdout' in summary and size(summary['stdout']) > 2000:
        summary['stdout'] = omitted(summary['stdout'])
    return summary


def fit_model_entry(item, budget):
    """Keep complete decision records. Only overloads omit whole records/values."""
    if size(item) <= budget:
        return item
    if not isinstance(item.get('feedback'), dict):
        return atomic_fields(item, budget)
    result = atomic_fields({k: v for k, v in item.items() if k not in ('feedback', 'source')},
                           min(2000, budget // 8))
    feedback = item['feedback']
    metadata = {k: v for k, v in feedback.items() if k not in ('key_results', 'calls', 'result', 'stdout', 'state_samples')}
    if isinstance(metadata.get('error'), dict) and size(metadata['error']) > budget // 4:
        metadata['error'] = atomic_fields(metadata['error'], max(180, budget // 4))
    result['feedback'] = atomic_fields(metadata, max(180, budget // 4))
    # Retain source as a whole, or explicitly omit it. Never send just helper
    # definitions while hiding the main program's early-return conditions.
    if 'source' in item:
        result['source'] = omitted(item['source'])
    records = feedback.get('key_results', [])
    allowance = max(180, budget - size(result) - 100)
    result['feedback']['key_results'] = whole_records(records, allowance)
    for key in ('source',):
        if key in item and size(dict(result, **{key: item[key]})) <= budget:
            result[key] = item[key]
    for key in ('result', 'state_samples', 'calls', 'stdout'):
        if key in feedback:
            candidate = dict(result, feedback=dict(result['feedback'], **{key: feedback[key]}))
            if size(candidate) <= budget:
                result = candidate
    missing = [k for k in feedback if k not in result['feedback']]
    if missing:
        result['feedback']['_omitted_fields'] = missing
    if size(result) > budget:
        # This path concerns tiny residual budgets, not partial tool receipts.
        return dict(fields(item, ('version', 'phase')), _omitted=True)
    return result


def prompt_history(history):
    """A model-specific bounded copy, independent of lossy browser previews.

    Reserve decision/POST/continuation records first, then spend the shared
    budget on executions newest first. Intervening reviews/resets cannot age
    out the previous execution. There is no per-call or 24 KB execution cap.
    """
    entries = []
    for entry in history:
        item = dict(entry)
        if isinstance(item.get('feedback'), dict):
            item['feedback'] = model_feedback(item['feedback'])
        if isinstance(item.get('decision'), dict):
            # Executed source is already in the version's feedback entry.
            item['decision'] = {k: v for k, v in item['decision'].items() if k != 'source'}
        entries.append(item)
    if size(entries) <= HISTORY_BYTES:
        return entries
    # Drop supplementary copies before dropping any execution evidence/source.
    states = {}
    for item in entries:
        if isinstance(item.get('feedback'), dict):
            states[id(item)] = item['feedback']['state_samples']
            item['feedback'] = {k: v for k, v in item['feedback'].items() if k not in ('stdout', 'calls', 'state_samples')}
            item['feedback']['_omitted_fields'] = ['stdout', 'calls', 'state_samples']
    if size(entries) <= HISTORY_BYTES:
        return entries
    output = {}
    remaining = HISTORY_BYTES - 512
    control_budget = HISTORY_BYTES // 4
    control = [i for i in reversed(range(len(entries))) if not isinstance(entries[i].get('feedback'), dict)]
    # Old POST/PRE and operator decisions outrank repetitive reset receipts.
    # Otherwise resets alone can spend the entire reserved dialogue budget.
    dialogue = [i for i in control if any(k in entries[i] for k in (
        'decision', 'operator_continuation', 'human_intervention', 'validation_error', 'assistance_validation_error'))]
    other_control = [i for i in control if i not in dialogue]
    executions = [i for i in reversed(range(len(entries))) if isinstance(entries[i].get('feedback'), dict)]
    for i in dialogue + executions[:2] + other_control + executions[2:]:
        budget = min(8000, control_budget, remaining) if i in control else remaining
        if budget < 600:
            continue
        excerpt = fit_model_entry(entries[i], budget - 64)
        output[i] = excerpt
        cost = size(excerpt) + 2
        remaining -= cost
        if i in control:
            control_budget -= cost
        # Only after both recent executions have their own receipts/source can
        # successful state polls spend remaining space. Older POSTs were already
        # reserved above. Retained state samples are also indivisible records.
        if executions[:2] and i == executions[min(1, len(executions)-1)]:
            for recent in executions[:2]:
                rows = states.get(id(entries[recent]), [])
                if (recent not in output or not isinstance(output[recent].get('feedback'), dict)
                        or not rows or remaining < 600):
                    continue
                samples = whole_records(rows, min(remaining-128, 8000))
                output[recent]['feedback']['state_samples'] = samples
                omitted_fields = output[recent]['feedback'].get('_omitted_fields', [])
                output[recent]['feedback']['_omitted_fields'] = [k for k in omitted_fields if k != 'state_samples']
                remaining -= size(samples) + 32
    result = [output[i] for i in sorted(output)]
    count = len(entries) - len(output)
    if count:
        result.insert(0, {'_omitted_older_entries': count,
                          'evidence': 'Some older history entries omitted whole; full history remains in experiment.json.'})
    return result
