"""Restore conversation evidence for an explicit continuation, never replay code."""
import json

RESUMABLE = {'waiting_operator', 'paused', 'interrupted', 'error'}


def load_history(directory, state):
    path = directory / 'history.json'
    if path.exists():
        return json.loads(path.read_text())
    # Older sessions predate history.json. Rebuild from their immutable records.
    from .presentation import read, turns
    history = []
    for row in turns(directory, state):
        turn = row['turn']
        decision = read(directory / f'turn-{turn:02d}-decision.json')
        if row['version']:
            folder = directory / row['version']
            history.append(dict(version=row['version'], reason=decision.get('reason', ''),
                source=(folder / 'source.py').read_text(), feedback=read(folder / 'result.json')))
        elif decision:
            history.append(dict(turn=turn, decision=decision,
                executed=False, note='Decision retained as context only; do not replay it.'))
        else:
            feedback = read(directory / f'turn-{turn:02d}-feedback.json')
            if feedback:history.append(dict(turn=turn, feedback=feedback))
    return history


def task_from_state(state):
    from .tasks import MockReachTask, VisualPreparationTask
    from .physical_task import DualJointTask, PhysicalPickPlaceTask, PhysicalProbeTask
    context = state['task']
    name = context['task_name']
    if name == 'dual-joint-task':return DualJointTask(context['language'])
    if name == 'green-cube-to-plate':return PhysicalPickPlaceTask()
    if name == 'arm-tracking-probe':return PhysicalProbeTask(context['metadata']['distance_m'])
    for task in (MockReachTask(), VisualPreparationTask()):
        if task.context.task_name == name:return task
    raise ValueError('cap_task_mismatch')
