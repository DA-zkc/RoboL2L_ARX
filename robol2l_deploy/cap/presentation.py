"""Read-only presentation of persisted CaP decisions, inputs and execution evidence."""
import json
import hashlib
from functools import lru_cache
from ..localization import chinese_error
from .continuation import RESUMABLE


@lru_cache(maxsize=128)
def _read(path, modified):
    with open(path) as stream:
        return json.load(stream)


def read(path):
    return _read(str(path), path.stat().st_mtime_ns) if path.exists() else {}


def failure(feedback):
    for call in reversed(feedback.get('calls', [])):
        if call.get('ok') is False:
            output = call.get('output') or {}
            error = output.get('error') or call.get('error') or {}
            message = error.get('message', 'API call failed')
            return dict(tool=call.get('tool'), line=call.get('line'), message=message,
                        message_zh=chinese_error(message,error.get('details')),
                        code=error.get('code'), details=error.get('details') or output.get('diagnostics') or {})
    error = feedback.get('error') or {}
    if error:
        message = error.get('message', 'Program failed')
        return dict(message=message, message_zh=chinese_error(message), code=error.get('code'),
                    line=error.get('line'), details={})
    return None


def turns(directory, state):
    rows = []
    version_index = 0
    versions = state.get('versions', [])
    by_turn = {v['turn']:v for v in versions if 'turn' in v}
    legacy = [v for v in versions if 'turn' not in v]
    for path in sorted(directory.glob('turn-*-context.json'),key=lambda p:int(p.name.split('-')[1])):
        turn = int(path.name.split('-')[1])
        prefix = directory / f'turn-{turn:02d}'
        decision = read(prefix.with_name(prefix.name + '-decision.json'))
        version = None
        feedback = read(prefix.with_name(prefix.name + '-feedback.json'))
        if decision.get('kind') == 'program':
            record = by_turn.get(turn)
            if record is None and version_index < len(legacy):
                candidate=legacy[version_index]
                digest=hashlib.sha256(decision.get('source','').encode()).hexdigest()
                if candidate.get('source_sha256',digest)==digest:
                    record=candidate;version_index+=1
            if record is not None:
                version = record['version']
                feedback = read(directory / version / 'result.json')
        rows.append(dict(turn=turn, kind=decision.get('kind'), reason=decision.get('reason'),
                         version=version, program_ok=feedback.get('ok'), failure=failure(feedback),
                         call_count=len(feedback.get('calls', [])), has_feedback=bool(feedback),
                         revision=f'{path.stat().st_mtime_ns}:{version}:{bool(feedback)}:{decision.get("kind")}'))
    return rows


def turn_detail(directory, state, turn):
    row = next((row for row in turns(directory, state) if row['turn'] == turn), None)
    if row is None:
        raise ValueError('cap_turn_not_found')
    context = read(directory / f'turn-{turn:02d}-context.json')
    version = row['version']
    return dict(context=context, decision=read(directory / f'turn-{turn:02d}-decision.json'),
                feedback=read(directory / version / 'result.json') if version else
                    read(directory / f'turn-{turn:02d}-feedback.json'))


def summary(state, rows, runtime):
    status = state['status']
    terminal = status not in ('running', 'idle', 'ready')
    last = rows[-1] if rows else {}
    failed = last.get('failure')
    # Prior corrected errors are conversation history, not the terminal cause.
    if last.get('kind') in ('finish', 'assist') or status in ('completed', 'running', 'idle', 'ready'):
        failed = None
    reason = state.get('reason') or state.get('message_zh') or ''
    labels = {'idle':('未启动','Idle'), 'ready':('准备就绪','Ready'),
              'running':('正在运行','Running'), 'completed':('本轮完成','Run completed'),
              'paused':('已暂停','Paused'), 'interrupted':('会话中断','Interrupted'),
              'error':('运行失败','Run failed'), 'waiting_operator':('需要处理','Action needed')}
    title, title_en = labels.get(status, (status, status))
    origin = 'runtime'
    if status == 'waiting_operator' and last.get('kind') == 'assist':
        title, title_en, origin = 'AI 请求协助', 'AI requests assistance', 'model'
    elif status == 'waiting_operator' and (reason == 'runtime_fault_latched' or failed):
        title, title_en = '执行已停止', 'Execution stopped'
    if status == 'completed':
        title, title_en = ('真机任务成功', 'Physical task succeeded') if state.get('physical_task_success') else ('本轮完成', 'Run completed')
        origin = 'verification'
        if state.get('verification',{}).get('required') is False:
            title,title_en,origin='AI 报告完成','AI reports completion','model'
    if failed:
        reason_zh = failed['message_zh']
        if failed.get('tool'):
            reason_zh = f'{failed["tool"]}' + (f' · 第 {failed["line"]} 行' if failed.get('line') else '') + '：' + reason_zh
    elif reason:
        localized=chinese_error(reason)
        reason_zh = state.get('message_zh') or (reason if localized.startswith('操作未完成，请查看错误详情：') else localized)
    else:
        reason_zh = {'running':'等待模型生成程序或返回下一步决策。',
                     'paused':'本轮已在派发边界暂停，没有自动续跑。',
                     'interrupted':'上次运行已中断，没有自动重放程序。'}.get(status,'选择任务后点击开始。')
    if status == 'completed' and not state.get('physical_task_success'):
        reason_zh += ' 本轮完成不代表已完成真机抓放。'
    if runtime.faulted:
        next_zh = '先处理下方控制器故障或未知动作结果。“继续”不会重置控制器或清除故障。原对话和执行记录保留。'
        next_en = 'Resolve the controller fault or unknown outcome first. Continue never resets the controller or clears faults; records are retained.'
    elif status in RESUMABLE:
        next_zh = '点击左侧“继续当前任务”，保留原对话和执行记录；AI 会读取新观测后修订程序。可选填补充说明，无需重启服务。'
        next_en = 'Click Continue current task to retain the conversation and execution records. The AI re-observes and revises. Notes are optional; no restart needed.'
    elif status == 'completed':
        next_zh, next_en = '无需重启。需要再次实验时启动新一轮即可。', 'No restart needed. Start a new run for another experiment.'
    elif status in ('running', 'idle', 'ready'):
        next_zh = '正在运行，无需重启；新程序和反馈会自动出现在下方。' if status == 'running' else '无需重启，选择左侧任务即可开始。'
        next_en = 'No restart needed. New programs and feedback appear below.'
    else:
        next_zh, next_en = '请查看上述状态和执行反馈。', 'See the status and execution feedback above.'
    return dict(title=title, title_en=title_en, origin=origin,
                tone='success' if status == 'completed' else 'error' if status == 'error' or runtime.faulted else 'attention' if terminal else 'neutral',
                reason=reason_zh, reason_code=reason, failure=failed,
                next_step=next_zh, next_step_en=next_en, restart_service=False,
                can_continue=status in RESUMABLE and not runtime.faulted,
                can_start_new=terminal and not runtime.faulted and not runtime.paused,
                verification=state.get('verification', {}))
