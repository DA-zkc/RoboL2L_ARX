"""Single-task experiments: authorized initial-pose return and small-motion retries.

This is orchestration only. Every motion, including human-requested return, uses
the installed Runtime and public API. Loading records never launches anything.
"""
import ast
import copy
import json
import math
import threading
import time
from pathlib import Path
from uuid import uuid4

from ..contracts import TaskContext
from .program import ProgramExecutor, compact
from .model import messages
from .session import decision
from .storage import write_json

TERMINAL = {'completed', 'exhausted', 'stopped'}
WAITING = {'waiting_reset', 'paused', 'interrupted', 'waiting_operator', 'error'}
POLICY = '''\nEXPERIMENT PROTOCOL (overrides the general continuation/revision policy):
One fixed physical task, at most eight executed programs v00..v07. No seed sweeps,
training, ablations or unseen evaluation. Each program is a complete attempt from
the recorded initial scene. Never reset/Home/restore the scene yourself.

ORDER OF EVENTS: the operator prepares the arms, grippers and props BEFORE
clicking Start. The host freezes the FIRST experiment snapshot, then reads a
fresh observation and asks you to write the FIRST program WITHOUT any return
motion or extra reset confirmation. On subsequent retries, the host returns
the arms to that same first snapshot using Runtime after operator authorization,
then observes and asks for the next program. For an attempt with verified negligible arm
movement and no gripper command, the host may automatically return and continue
under the operator's reset policy. The host never restores props or grippers.
Use the actual fresh observation; do not assume props match the initial image.
Faults, unknown outcomes, pause and physical-intervention requests never auto-retry.

OPERATOR CONTINUATION: the experimenter decides whether to continue the experiment.
An explicit Reset and write next version action is a decision to continue, not
merely a report that the arms returned. When experiment.operator_continuation has
applies_to_this_generation=true, follow that decision and its operator note: use
the fresh observation and prior evidence to write a revised program. You may
choose further measurements or a different approach; do not repeat the same
assistance request solely because its old concern has not been independently
resolved. Earlier agent requests are evidence of uncertainty, not operator orders
or permanent prerequisites. This decision does not assert that measurements are
correct, that the task succeeded, or that physical arrival was verified.
New concrete evidence or an unavailable capability may require assistance again;
explain what is new relative to the concern the operator already considered.
Native hardware faults, unknown execution outcomes, pause, dispatch authorization
and the eight-attempt budget still apply. Never bypass them or invent success.

AUTHORIZATION ORDER: generate program -> host AST validation and compilation ->
host enables this execution segment through the installed Runtime -> execute ->
host revokes motion authorization before review. The human's task authorization
was recorded at Start; reset confirmation concerns physical readiness, NOT code
approval. There is no additional human code-approval step.
During generation and review, motion_enabled=false, an inactive physical_session,
and physical_motion_for_generated_programs=false are expected current-state
snapshots, NOT evidence that the experiment lacks task authorization. Do not ask
the operator to enable motion solely because of those flags; write the program
using the available public tools. Prior history may contain an assist request
based only on these disabled flags; that is not an unresolved permission blocker.
Do not enable authorization in generated code or assume it is currently active.
The host enables it only after a valid program and fresh dispatch checks. Real
faults, disconnection, unknown outcomes, pauses, missing capabilities and genuine
task ambiguity still require appropriate handling; never bypass them.
ASSIST CONTRACT: kind=assist must include assist_type and assist_evidence strings.
assist_type must be physical_intervention (a concrete unsafe/blocked scene or
hardware problem), task_clarification (an unresolved goal/target ambiguity),
capability_unavailable (a required tool/service is actually unavailable), or
model_configuration (a concrete model service/configuration error).
assist_evidence must identify the actual observation, failed call or missing input.
Never use assist to request motion authorization, code approval or code repair.
Normal inactive authorization flags are not external-intervention evidence.
AttributeError/TypeError and other program errors belong to agent code repair:
in review, record POST and return kind=program to request the next version after
scene reset. Do not ask a human to fix code. For non-assist decisions, omit the
assist fields (or use empty strings if the transport schema requires them).
During review, authorization has been revoked again. Analyse the result without
requesting re-enablement; another version must first pass the host reset gate.

The human authorizes arm return and checks physical device and prop readiness. The human does NOT review,
approve or correct your program, and there is no human code gate: once you return a
program it is checked by machine and dispatched. Whether the code is correct is
your responsibility. Invalid or unparseable programs are rejected by a static
check and returned to you WITHOUT consuming an attempt; after three rejections in
a row the host stops and waits for a human. A program that reaches the executor
counts as an attempt even if planning fails or nothing moves.

SPEED: the operator sets a recommended starting speed factor for this run and
it is given to you as experiment.operator_speed_recommendation. Use it as your
default time_dilation_factor for transfers, and go LOWER near contact, during
placement and while holding an object. It is a recommendation, not a host-enforced
cap: the native domain 0<s<=1 still applies and nothing clamps your value, so you
remain responsible for choosing a safe speed and explaining it in your reason.

Record your reasoning in these optional string fields, which the console shows to
the operator as the experiment record:
  pre_a - the root cause you identified and what you changed for this version.
  pre_b - what you predict will happen, and the risks you accept.
  post  - at review time, what the feedback and fresh images actually show.
Keep `reason` as a short summary; put the detail in these fields.

Programs may have multiple public calls. After an execution, inspect the fresh
feedback and images: return finish only if they support completion, assist if
blocked, otherwise return a program to request one more attempt. At review time
`source` is still required by the response schema but is NOT dispatched: the host
discards it and asks you again after the next host reset gate, so every
dispatched program is written against a confirmed scene. Keep it short there and
put your findings in `post`. At eight attempts you may only assess the outcome;
a ninth program is never dispatched.
An AI finish is an AI-reported outcome, not independent physical verification.
'''
DEFAULT_SPEED = 0.05
RECORDS = ('pre_a', 'pre_b', 'post')
DECISION_FIELDS = RECORDS + ('assist_type', 'assist_evidence')
EXTERNAL_ASSISTANCE = {'physical_intervention', 'task_clarification', 'capability_unavailable', 'model_configuration'}


class AssistanceProtocolError(ValueError):
    pass


class ExperimentController:
    def __init__(self, runtime, model_factory, directory=None, skills=None):
        self.runtime = runtime
        self.is_arx = runtime.adapter.describe().get('embodiment', '').startswith('arx_')
        # User-defined experiment budget, independent of hardware policy.
        self.maximum_attempts = 8
        self.model_factory = model_factory
        self.directory = Path(directory) if directory else runtime.directory / 'experiments'
        self.directory.mkdir(parents=True, exist_ok=True)
        self.skills = skills
        self.lock = threading.RLock()
        self.thread = None
        self.stop = threading.Event()
        self.state = None
        self.folder = None
        paths = list(self.directory.glob('*/experiment.json'))
        if paths:
            path = max(paths, key=lambda p: p.stat().st_mtime_ns)
            self.folder = path.parent
            self.state = json.loads(path.read_text())
            if self.state['status'] not in TERMINAL:
                self.state['history'].append({'restart_interruption': {
                    'previous_status': self.state['status'], 'attempts_reserved': self.state['attempts'],
                    'unfinished_versions': [v for v in self.state['versions'] if v['status'] != 'finished'],
                    'instruction': 'Do not replay. Human must inspect device and restore scene before continuing.'}})
                self.state.update(status='interrupted', reason='process_restarted_no_replay', pending=None)
                self.save()

    def busy(self):
        return bool(self.thread and self.thread.is_alive())

    def active(self):
        return bool(self.state and self.state['status'] not in TERMINAL)

    def step(self, node):
        """Record passing through one node of the console's flow graph.

        Statuses alone cannot drive that graph: PLAN, GATE, SUCCESS, BUDGET,
        NEXT and DONE have no status of their own, so without this they would
        never light up and the trace would look like it skipped them.
        """
        with self.lock:
            if not self.state: return
            steps = self.state.setdefault('steps', [])
            steps.append({'node': node, 'time': time.time(), 'attempt': self.state['attempts']})
            del steps[:-240]
            write_json(self.folder / 'experiment.json', self.state)

    def save(self, status=None, **changes):
        with self.lock:
            if status:
                self.state['status'] = status
                self.state['events'].append(dict(time=time.time(), status=status, attempt=self.state['attempts']))
            self.state.update(changes)
            write_json(self.folder / 'experiment.json', self.state)

    def snapshot(self, *, summary=False):
        with self.lock:
            if summary and self.state:
                from .prompt_history import prompt_history, feedback_summary, bounded
                result = copy.deepcopy({k: v for k, v in self.state.items() if k not in ('history', 'versions')})
                result['history'] = copy.deepcopy(prompt_history(self.state['history']))
                result['versions'] = []
                for version in self.state['versions']:
                    row = copy.deepcopy({k: v for k, v in version.items() if k != 'feedback'})
                    if 'feedback' in version:
                        row['feedback'] = copy.deepcopy(bounded(feedback_summary(version['feedback']), 24_000))
                    result['versions'].append(row)
                result['evidence_view'] = 'summary; full records available from record/export'
            else:
                result = copy.deepcopy(self.state) if self.state else {'status': 'idle', 'attempts': 0}
            result['busy'] = self.busy()
            result['maximum_attempts'] = self.maximum_attempts
            result['reset_authorization_supported'] = True
            result['review_retry_supported'] = self._can_retry_review()
            return result

    def _can_retry_review(self):
        return bool(self.state and self.state['status'] in ('error', 'interrupted')
                    and self.state.get('gate', {}).get('phase') == 'review'
                    and self.state.get('versions') and self.state['versions'][-1]['status'] == 'finished'
                    and not self.busy() and not self.runtime.closed and not self.runtime.faulted
                    and not self.runtime.paused)

    def retry_review(self, session_id, expected_attempt):
        """Retry model review only. Never acknowledge faults, reset or run code."""
        with self.lock:
            self.check_session(session_id)
            if type(expected_attempt) is not int or expected_attempt != self.state['attempts']:
                raise ValueError('stale_review_retry_refresh_panel')
            if not self._can_retry_review(): raise ValueError('review_retry_unavailable')
            self.state['interventions'].append(dict(time=time.time(), kind='human_retried_review',
                                                    before_attempt=self.state['attempts']))
            def review():
                self._revoke()
                observation, after = self.observe('review-retry-' + str(time.time_ns()))
                self.state['versions'][-1]['review_observation'] = after
                self.step('POST')
                value = self._decide(self.model_factory(), observation, 'review')
                self._handle(value, 'review')
            return self._launch(review)

    def _launch(self, fn):
        if self.busy(): raise ValueError('experiment_busy')
        self.stop.clear()
        def work():
            try:
                fn()
            except Exception as error:
                # Do not persist provider response bodies, URLs with secrets or headers.
                # These prefixes are fixed internal codes, safe to surface: without
                # them a timeout reaches the operator as a bare "RuntimeError".
                reason = str(error) if str(error).startswith(
                    ('model_configuration_error_', 'model_request_failed_http_', 'model_request_timeout', 'model_transport_failed',
                     'api_key_missing', 'model_name_missing', 'codex_', 'model_assistance_protocol_error')
                ) else type(error).__name__
                from .experiment_models import ModelRequestError
                diagnostics = error.diagnostics if isinstance(error, ModelRequestError) else None
                self.save('waiting_operator' if self.runtime.faulted else 'error', reason=reason,
                          model_error=diagnostics,
                          error_category='model_configuration' if reason.startswith('model_configuration_error_') else 'model_protocol' if reason == 'model_assistance_protocol_error' else 'model_service' if reason.startswith(('model_request_', 'model_transport_')) else 'execution_or_service')
            finally:
                self._revoke()
        self.thread = threading.Thread(target=work, daemon=True)
        self.thread.start()
        return {'accepted': True, 'session_id': self.state['session_id']}

    def _revoke(self):
        if self.runtime.adapter.synthetic:
            self.runtime.allow_motion = False
        else:
            from ..physical import revoke
            revoke(self.runtime)

    def _authorize(self):
        if self.stop.is_set() or self.runtime.faulted or self.runtime.paused or self.runtime.closed:
            raise ValueError('station_faulted_or_paused')
        if self.runtime.adapter.synthetic:
            self.runtime.allow_motion = True
        else:
            from ..physical import authorize
            authorize(self.runtime, self.context().task_name, self.state['authorization'])

    def context(self):
        return TaskContext('physical-experiment', 0, 'experiment-' + self.state['session_id'],
            self.state['goal'] + '\nCompletion criteria: ' + self.state['criteria'],
            'single-task', {'maximum_attempts': self.maximum_attempts, **self.state.get('task_metadata', {})})

    def observe(self, name):
        observation = self.runtime.get_observation()
        value = dict(observation=compact(observation), observation_id=next(reversed(self.runtime.observations)),
                     device_binding=self.runtime.adapter.device_binding(), time=time.time())
        # Retain visible initial/after images independent of the live observation cache.
        import cv2
        value['images'] = []
        for index, (camera, frame) in enumerate(observation.cameras.items()):
            filename = name + '-' + str(index) + '.jpg'
            cv2.imwrite(str(self.folder / filename), cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR))
            value['images'].append(dict(camera=camera, file=filename))
        write_json(self.folder / (name + '.json'), value)
        return observation, value

    def start(self, goal, criteria, confirmed, authorization='', task_metadata=None, speed_factor=None,
              record_cameras=None):
        with self.lock:
            if self.busy() or self.active(): raise ValueError('experiment_already_active')
            if confirmed is not True: raise ValueError('experiment_confirmation_required')
            if not all(isinstance(x, str) and x.strip() for x in (goal, criteria)):
                raise ValueError('task_and_completion_criteria_required')
            cameras = list(record_cameras or [])
            if not all(isinstance(c, str) and c in self.runtime.adapter.cameras for c in cameras):
                raise ValueError('unknown_camera_for_recording')
            speed = (1. if self.is_arx else DEFAULT_SPEED) if speed_factor is None else speed_factor
            if (not isinstance(speed, (int, float)) or isinstance(speed, bool) or not math.isfinite(speed)
                    or speed <= 0 or (not self.is_arx and speed > 1)):
                raise ValueError('positive_finite_speed_factor_required' if self.is_arx
                                 else 'speed_factor_must_be_above_0_and_at_most_1')
            model = self.model_factory()  # Validate settings before capturing an experiment.
            if getattr(model, 'label', '') == 'feedback_fixture_not_llm' and self.runtime.adapter.synthetic:
                task_metadata = {'target_position_m': [0, 0, .31], 'gripper_position': .5}
            # Last, so a rejected model or form leaves the station exactly as it was.
            acknowledged = self._acknowledge('human_started_experiment')
            session = uuid4().hex
            self.folder = self.directory / session
            self.folder.mkdir()
            self.state = dict(session_id=session, status='capturing', goal=goal, criteria=criteria,
                authorization=authorization or 'Human started and confirmed this experiment: ' + goal,
                synthetic=self.runtime.adapter.synthetic, physical_task_success=False,
                attempts=0, versions=[], history=[], events=[], pending=None, steps=[],
                interventions=[dict(time=time.time(), kind='human_started_experiment', fault_acknowledged=True,
                                    acknowledged_robot_state=acknowledged)] if acknowledged else [],
                speed_factor=float(speed), record_cameras=cameras, tags=[],
                model=getattr(model, 'settings', {'model': model.label}), task_metadata=task_metadata or {})
            self.save('capturing')
            for node in ('START', 'SETUP', 'CLICK'): self.step(node)
            def capture():
                self.step('SNAP')
                _, initial = self.observe('initial')
                self.save(initial=initial)
                if self.stop.is_set() or self.runtime.paused:
                    self.save('paused', reason='paused_after_initial_capture'); return
                if self.runtime.faulted or self.runtime.closed:
                    self.save('waiting_operator', reason='runtime_fault_or_unknown_outcome'); return
                # Start confirms the scene already prepared by the operator.
                # Never return before v00, and never replace this snapshot later.
                self.save('observing', reason='initial_scene_captured_no_return')
                self._cycle()
            return self._launch(capture)

    def check_session(self, session_id):
        if not self.state or session_id != self.state['session_id']: raise ValueError('experiment_session_changed')
        if self.state['synthetic'] != self.runtime.adapter.synthetic: raise ValueError('experiment_device_changed')
        initial = self.state.get('initial')
        if not initial: raise ValueError('initial_capture_missing_start_new_experiment')
        if initial['device_binding'] != self.runtime.adapter.device_binding(): raise ValueError('experiment_device_changed')

    def record_unlock(self, receipt):
        """Keep an operator's controller unlock in the live run's record."""
        with self.lock:
            if not self.state or self.state['status'] in TERMINAL: return
            self.state['interventions'].append(dict(time=time.time(), kind='human_unlocked_controller',
                                                    receipt=compact(receipt)))
            self.save()

    def record_manual_positioning(self, action, request):
        with self.lock:
            if not self.state or self.state['status'] in TERMINAL: return
            self.state['interventions'].append(dict(time=time.time(), kind='human_manual_positioning',
                                                    action=action, request=dict(request)))
            self.save()

    def _acknowledge(self, kind):
        """The human's own click clears what a reflex latched.

        Both human gates — confirming the reset and asking for the arm return —
        are pressed after the operator inspected the station and cleared the
        controller, so they acknowledge the latch instead of being refused. The
        device must still read idle and error free, and a station paused for
        dispatch is resumed for the motion they just asked for. Returns the
        measured state to record when something actually had to be cleared.
        """
        if self.runtime.closed: raise ValueError('station_closed')
        latched = self.runtime.faulted or self.runtime.paused
        state = self.runtime.acknowledge_fault(kind)
        if self.runtime.paused: self.runtime.pause(False)
        return compact(state) if latched else None

    def confirm_reset(self, session_id, confirmed, note='', expected_attempt=None):
        if self.state and self.state.get('reset_policy', {}).get('automatic_small_motion'):
            return self.authorize_reset(session_id, confirmed, note, expected_attempt)
        with self.lock:
            self.check_session(session_id)
            if self.busy() or self.state['status'] not in WAITING: raise ValueError('experiment_not_waiting_for_reset')
            if confirmed is not True or not isinstance(note, str): raise ValueError('human_reset_confirmation_required')
            if type(expected_attempt) is not int or expected_attempt != self.state['attempts']:
                raise ValueError('stale_reset_confirmation_refresh_panel')
            acknowledged = self._acknowledge('human_confirmed_reset')
            self._record_operator_continuation(note, readiness='confirmed')
            # The agent is told a latched fault was cleared by hand; the measured
            # state behind that acknowledgement stays in the record, out of the prompt.
            intervention = dict(time=time.time(), kind='human_reset_confirmed',
                                before_attempt=self.state['attempts'], note=note,
                                **({'fault_acknowledged': True} if acknowledged else {}))
            self.state['interventions'].append(dict(intervention, acknowledged_robot_state=acknowledged)
                                               if acknowledged else intervention)
            self.state['history'].append({'human_intervention': intervention})
            self.save('observing', reason='reset_confirmed', error_category=None)
            self.step('RESET'); self.step('CONFIRM')
            return self._launch(self._cycle)

    def _record_operator_continuation(self, note, *, readiness):
        """Record an explicit human decision; automatic returns never create one."""
        previous_assist = next((entry['decision'] for entry in reversed(self.state['history'])
                                if isinstance(entry.get('decision'), dict)
                                and entry['decision'].get('kind') == 'assist'), None)
        record = dict(id=uuid4().hex, time=time.time(), decision='continue_with_revised_program',
                      for_attempt=self.state['attempts'], note=note,
                      previous_status=self.state['status'], previous_reason=self.state.get('reason'),
                      prior_assistance=({k: v for k, v in previous_assist.items() if k != 'source'}
                                        if previous_assist else None), readiness=readiness,
                      evidence_resolved=False, physical_arrival_verified=False)
        self.state['operator_continuation'] = record
        self.state['history'].append({'operator_continuation': copy.deepcopy(record)})

    def build_request(self, observation, phase, history=None):
        """The exact payload sent to the model, so the console can show it."""
        executor = ProgramExecutor(self.runtime, self.context())
        request = messages(executor.api, observation, self.state['history'] if history is None else history, {})
        policy = POLICY
        if self.is_arx:
            policy = policy.replace('For an attempt with verified negligible arm\nmovement and no gripper command', 'For an attempt with no gripper command and no unresolved execution error')
            policy = policy.replace('after three rejections in\na row the host stops and waits for a human', 'the model may correct rejected programs until the operator stops the run')
            start = policy.index('ASSIST CONTRACT:')
            end = policy.index('During review, authorization', start)
            policy = policy[:start] + 'The agent chooses whether to request assistance and explains its reason.\n' + policy[end:]
            start = policy.index('SPEED:')
            end = policy.index('Record your reasoning', start)
            policy = policy[:start] + (
                'SPEED: experiment.operator_speed_recommendation carries the operator\'s suggested '
                'starting time_dilation_factor. It is advisory, not a host-enforced cap. '
                'Choose the actual speed for the task and explain your choice. '
                'ARX motion duration is planned by the SDK.\n\n') + policy[end:]
        request[0]['content'] += policy
        context = json.loads(request[1]['content'][0]['text'])
        context['experiment'] = dict(phase=phase, attempts=self.state['attempts'], remaining=max(0, self.maximum_attempts-self.state['attempts']),
                                     initial=self.state['initial'],
                                     reset_confirmed=phase == 'generate' and self.state['attempts'] > 0,
                                     initial_scene_confirmed=True,
                                     first_attempt_without_return=self.state['attempts'] == 0,
                                     execution_authorization=dict(
                                         task_authorization_recorded=bool(self.state.get('authorization')),
                                         motion_enabled_now=bool(self.runtime.allow_motion),
                                         enable_at='after_static_validation_and_compilation_before_execution',
                                         revoke_at='after_execution_before_review',
                                         owner='host_installed_runtime',
                                         human_code_approval_required=False,
                                         disabled_during_model_decision_is_expected=True),
                                     operator_speed_recommendation=dict(
                                         time_dilation_factor=self.state.get('speed_factor', DEFAULT_SPEED),
                                         meaning='operator-recommended starting speed for transfers',
                                         go_lower_for='contact, placement, held objects',
                                         enforced_by_host=False, native_domain='0<s<=1'))
        if self.is_arx:
            advice = context['experiment']['operator_speed_recommendation']
            advice['native_domain'] = 'positive finite multiplier; duration planned by the SDK'
            advice.pop('go_lower_for', None)
        continuation = copy.deepcopy(self.state.get('operator_continuation'))
        if continuation:
            continuation['applies_to_this_generation'] = (
                phase == 'generate' and continuation['for_attempt'] == self.state['attempts']
                and continuation['readiness'] in ('confirmed', 'return_command_completed'))
        # Keep the current operator decision outside the bounded history summary.
        context['experiment']['operator_continuation'] = continuation
        # Operator-supplied reference library, carried as data next to the API.
        block = self.skills.prompt_block() if self.skills else None
        if block: context['skills'] = block
        request[1]['content'][0]['text'] = json.dumps(context, ensure_ascii=False)
        return request, context

    def _decide(self, model, observation, phase):
        request, _ = self.build_request(observation, phase)
        # The machine owns program correctness: a static check rejects invalid
        # programs and returns them to the model without consuming an attempt.
        self.save('reviewing' if phase == 'review' else 'generating',
                  reason='', error_category=None, model_error=None,
                  gate=dict(phase=phase, rejected=0, last_error=None),
                  assistance_validation=dict(rejected=0, last_error=None))
        if phase == 'generate':
            self.step('PLAN'); self.step('CODE')
        retry = 0
        while self.is_arx or retry < 3:
            retry += 1
            try:
                value = decision(model.decide(request), extra=DECISION_FIELDS)
                if value['kind'] == 'assist' and not self.is_arx:
                    if value.get('assist_type') not in EXTERNAL_ASSISTANCE or not value.get('assist_evidence', '').strip():
                        raise AssistanceProtocolError(
                            'assist_requires_external_problem_type_and_evidence; '
                            'normal_motion_authorization_is_host_managed; '
                            'for_program_errors_return_program_after_POST_for_next_reset')
                if value['kind'] == 'program':
                    from cap_harness.runtime import _ProgramValidator
                    tree = ast.parse(value['source'])
                    _ProgramValidator().visit(tree)
                    compile(tree, '<experiment-program>', 'exec')
                if phase == 'generate' and value['kind'] == 'program': self.step('GATE')
                self.state['history'].append({'phase': phase, 'decision': value})
                self.save()
                return value
            except AssistanceProtocolError as error:
                feedback = {'assistance_validation_error': str(error), 'phase': phase, 'rejected_decision': value}
                self.state['history'].append(feedback)
                request[-1]['content'].append({'type': 'text', 'text': json.dumps(feedback)})
                self.save(assistance_validation=dict(
                    rejected=self.state['assistance_validation']['rejected'] + 1, last_error=str(error)))
                if self.stop.is_set(): return {'kind': 'assist', 'reason': 'paused'}
                if retry == 3:
                    raise RuntimeError('model_assistance_protocol_error') from None
            except (ValueError, SyntaxError) as error:
                feedback = {'validation_error': str(error), 'phase': phase}
                self.state['history'].append(feedback)
                request[-1]['content'].append({'type': 'text', 'text': json.dumps(feedback)})
                self.save(gate=dict(phase=phase, rejected=self.state['gate']['rejected'] + 1, last_error=str(error)[:400]))
                if phase == 'generate': self.step('CODE')  # bounced by the gate, back to writing
                if self.stop.is_set(): return {'kind': 'assist', 'reason': 'paused'}
        return {'kind': 'assist', 'reason': 'three_invalid_model_responses'}

    def _records(self, value, names=RECORDS):
        # Experiment records are prompted for but optional; absent stays absent.
        return {name: value[name].strip() for name in names
                if isinstance(value.get(name), str) and value[name].strip()}

    def _budget_exhausted(self):
        return self.state['attempts'] >= self.maximum_attempts

    def _handle(self, value, phase):
        if self.runtime.faulted or self.runtime.closed:
            self.save('waiting_operator', reason='runtime_fault_or_unknown_outcome', pending=None)
            return
        if self.stop.is_set() or self.runtime.paused:
            self.save('paused', pending=None)
            return
        if phase == 'review' and self.state['versions']:
            # The review verdict is the only task-level signal we have. A program
            # that merely ran without raising is NOT a successful attempt, so the
            # console must not colour it as one.
            self.state['versions'][-1].update(self._records(value, ('post',)),
                outcome={'finish': 'task_complete', 'assist': 'blocked'}.get(value['kind'], 'task_incomplete'))
            self.step('SUCCESS')
        if value['kind'] == 'finish':
            self.step('DONE'); self.step('END')
            self.save('completed', reason=value['reason'], outcome='ai_reported_complete', pending=None)
        elif self._budget_exhausted():
            if phase == 'review': self.step('BUDGET')
            self.step('DONE'); self.step('END')
            self.save('exhausted', reason=value['reason'], pending=None)
        elif value['kind'] == 'assist':
            self.save('waiting_operator', reason=value['reason'], pending=None)
        else:
            # Requesting another attempt only moves us back to the human reset gate.
            # The next program is written after that confirmation, against a fresh
            # observation, so a program proposed at review time is never dispatched.
            if phase == 'review': self.step('BUDGET'); self.step('NEXT')
            self.save('waiting_reset', reason=value['reason'], pending=None)

    def _cycle(self):
        while True:
            self._cycle_once()
            if (self.state['status'] != 'waiting_reset' or self._budget_exhausted()
                    or not self.state.get('reset_policy', {}).get('automatic_small_motion')
                    or self.stop.is_set() or self.runtime.paused or self.runtime.faulted or self.runtime.closed):
                return
            audit = self.state['versions'][-1].get('reset_motion', {})
            if not audit.get('eligible'): return
            self.state['interventions'].append(dict(time=time.time(), kind='automatic_small_motion_reset',
                                                    before_attempt=self.state['attempts'], evidence=audit))
            self.save()
            if not self._restore_initial(automatic=True): return

    def _cycle_once(self):
        # Generation never inherits motion permission from an operator action.
        # A valid program is required before _authorize enables this segment.
        self._revoke()
        self.step('OBSERVE')
        observation, before = self.observe('before-' + str(self.state['attempts']))
        model = self.model_factory()
        if self._budget_exhausted():
            self._handle(self._decide(model, observation, 'review'), 'review')
            return
        value = self._decide(model, observation, 'generate')
        if self.stop.is_set() or self.runtime.paused:
            self.save('paused'); return
        if value['kind'] != 'program': self._handle(value, 'generate'); return
        self._authorize()
        version = 'v%02d' % self.state['attempts']
        entry = dict(version=version, reason=value['reason'], source=value['source'], before=before,
                     status='dispatched', rejected=self.state.get('gate', {}).get('rejected', 0),
                     **self._records(value, ('pre_a', 'pre_b')))
        self.state['versions'].append(entry)
        self.save('executing', attempts=self.state['attempts']+1, pending=None)
        self.step('RUN')
        from .recording import AttemptRecording
        from .reset import ResetMotionAudit
        audit = ResetMotionAudit(self.state['initial']['observation']['robot_state'], observation.robot_state)
        with self.runtime.audit_motion(audit), AttemptRecording(self.runtime, self.folder,
                              self.state.get('record_cameras'), version) as recording:
            feedback = ProgramExecutor(self.runtime, self.context()).execute(value['source'], self.folder/version,
                                                                              should_stop=self.stop.is_set)
            try:
                audit.sample(self.runtime.get_robot_state())
            except Exception:
                audit.blockers.add('final_state_unavailable')
        entry['reset_motion'] = audit.result()
        videos = recording.receipts()
        if videos: entry['videos'] = videos
        self._revoke()
        entry.update(status='finished', feedback=feedback)
        self.state['history'].append(dict(version=version, source=value['source'], reason=value['reason'], feedback=feedback))
        self.save('feedback')
        self.step('RESULT')
        if self.runtime.faulted or (feedback.get('error') or {}).get('code') == 'outcome_unknown':
            self.save('waiting_operator', reason='runtime_fault_or_unknown_outcome'); return
        observation, after = self.observe('after-' + version)
        entry['after'] = after
        if self.stop.is_set() or self.runtime.paused:
            self.save('paused'); return
        self.step('POST')
        value = self._decide(model, observation, 'review')
        # Keep the POST record and the task-level verdict even if a pause lands
        # before dispatch resumes.
        entry.update(self._records(value, ('post',)),
                     outcome={'finish': 'task_complete', 'assist': 'blocked'}.get(value['kind'], 'task_incomplete'))
        if self.stop.is_set() or self.runtime.paused:
            self.save('paused'); return
        self._handle(value, 'review')

    def records(self):
        """Summaries of every stored experiment, newest first, for the console list."""
        rows = []
        for path in self.directory.glob('*/experiment.json'):
            try:
                value = json.loads(path.read_text())
            except (ValueError, OSError):
                continue
            versions = value.get('versions', [])
            rows.append(dict(
                session_id=value.get('session_id', path.parent.name), status=value.get('status'),
                goal=value.get('goal', ''), criteria=value.get('criteria', ''),
                attempts=value.get('attempts', 0), versions=len(versions),
                synthetic=value.get('synthetic'), outcome=value.get('outcome'),
                physical_task_success=value.get('physical_task_success', False),
                succeeded=any(v.get('outcome') == 'task_complete' for v in versions),
                model=(value.get('model') or {}).get('model'), tags=value.get('tags', []),
                videos=sorted({name for v in versions for name in (v.get('videos') or {})}),
                started_s=(value.get('events') or [{}])[0].get('time'),
                updated_s=path.stat().st_mtime, directory=str(path.parent),
                active=bool(self.state and value.get('session_id') == self.state.get('session_id')),
                # A run still in progress owns its evidence: it cannot be deleted.
                locked=bool(self.state and value.get('session_id') == self.state.get('session_id')
                            and self.active())))
        return sorted(rows, key=lambda r: r['updated_s'], reverse=True)

    def record(self, session_id):
        """One stored experiment in full. Reading never resumes anything."""
        if Path(session_id).name != session_id: raise ValueError('invalid_experiment_id')
        path = self.directory / session_id / 'experiment.json'
        if not path.is_file(): raise ValueError('experiment_record_not_found')
        return json.loads(path.read_text())

    def archive(self, session_id):
        """Zip one experiment directory for download; built fresh, never cached."""
        import io, zipfile
        if Path(session_id).name != session_id: raise ValueError('invalid_experiment_id')
        folder = self.directory / session_id
        if not (folder / 'experiment.json').is_file(): raise ValueError('experiment_record_not_found')
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, 'w', zipfile.ZIP_DEFLATED) as bundle:
            for item in sorted(folder.rglob('*')):
                if item.is_file():
                    bundle.write(item, str(Path(session_id) / item.relative_to(folder)))
        return buffer.getvalue()

    def tag_records(self, session_ids, tags, confirmed=True):
        """Replace the operator tags on one or more stored experiments."""
        if confirmed is not True: raise ValueError('explicit_tag_confirmation_required')
        if not isinstance(tags, list) or not all(isinstance(t, str) for t in tags):
            raise ValueError('invalid_tags')
        clean = []
        for tag in tags:
            tag = tag.strip()[:40]
            if tag and tag not in clean: clean.append(tag)
        if len(clean) > 12: raise ValueError('too_many_tags')
        import os
        for session_id in self._known(session_ids):
            path = self.directory / session_id / 'experiment.json'
            value = json.loads(path.read_text())
            value['tags'] = clean
            stamp = path.stat()
            write_json(path, value)
            # Restoring on startup picks the most recently touched record, so a
            # tag must not make an old experiment look like the newest one.
            os.utime(path, ns=(stamp.st_atime_ns, stamp.st_mtime_ns))
            with self.lock:
                if self.state and self.state.get('session_id') == session_id:
                    self.state['tags'] = clean
        return {'records': self.records()}

    def _known(self, session_ids):
        """Validate a batch of record ids before touching any of them."""
        if not isinstance(session_ids, list) or not session_ids:
            raise ValueError('select_at_least_one_record')
        if len(session_ids) > 100: raise ValueError('too_many_records_selected')
        for session_id in session_ids:
            if not isinstance(session_id, str) or Path(session_id).name != session_id:
                raise ValueError('invalid_experiment_id')
            if not (self.directory / session_id / 'experiment.json').is_file():
                raise ValueError('experiment_record_not_found')
        return session_ids

    def delete_records(self, session_ids, confirmed):
        """Delete several stored experiments. Refuses if the live one is among them."""
        if confirmed is not True: raise ValueError('explicit_record_delete_confirmation_required')
        wanted = self._known(session_ids)
        with self.lock:
            if self.state and self.active() and self.state.get('session_id') in wanted:
                raise ValueError('end_the_experiment_before_deleting_its_record')
        for session_id in wanted:
            self.delete_record(session_id, True)
        return {'records': self.records()}

    def delete_record(self, session_id, confirmed):
        """Remove one stored experiment and its evidence. Never the live one."""
        import shutil
        if confirmed is not True: raise ValueError('explicit_record_delete_confirmation_required')
        if Path(session_id).name != session_id: raise ValueError('invalid_experiment_id')
        folder = self.directory / session_id
        if not (folder / 'experiment.json').is_file(): raise ValueError('experiment_record_not_found')
        with self.lock:
            if self.state and self.state.get('session_id') == session_id and self.active():
                raise ValueError('end_the_experiment_before_deleting_its_record')
            if self.busy(): raise ValueError('experiment_busy')
            shutil.rmtree(folder)
            # Forget an in-memory snapshot whose evidence has just been removed.
            if self.state and self.state.get('session_id') == session_id:
                self.state, self.folder = None, None
        return {'records': self.records()}

    def preview(self, phase='generate', goal=None, criteria=None):
        """Render the payload the agent would receive, without sending it.

        Works before an experiment exists: pass the goal and criteria from the
        form and this shows the very first message that clicking start would
        produce. Images are replaced by their size so the console can show text.
        """
        if self.busy(): raise ValueError('experiment_busy')
        with self.lock:
            restore = self.state
            try:
                if self.state is None or goal is not None:
                    if not (isinstance(goal, str) and goal.strip()):
                        raise ValueError('fill_in_the_task_goal_to_preview_the_first_message')
                    # A throwaway state, so previewing never touches a stored run.
                    self.state = dict(session_id='preview', status='idle', goal=goal,
                                      criteria=criteria if isinstance(criteria, str) else '',
                                      authorization='', attempts=0, versions=[], history=[],
                                      interventions=[], events=[], steps=[], pending=None,
                                      initial=None, task_metadata={})
                observation = self.runtime.get_observation()
                request, context = self.build_request(observation, phase)
            finally:
                self.state = restore
        blocks = []
        for block in request[1]['content']:
            if block['type'] == 'text':
                blocks.append({'type': 'text', 'chars': len(block['text'])})
            else:
                blocks.append({'type': 'image', 'chars': len(block['image_url']['url'])})
        return {'system': request[0]['content'], 'context': context, 'blocks': blocks,
                'phase': phase, 'system_chars': len(request[0]['content']),
                'context_chars': len(json.dumps(context, ensure_ascii=False))}

    def authorize_reset(self, session_id, confirmed, note='', expected_attempt=None):
        """Explicit operator permission: return to the first snapshot, then continue."""
        with self.lock:
            self.check_session(session_id)
            if self.busy() or self.state['status'] not in WAITING:
                raise ValueError('experiment_not_waiting_for_reset')
            if confirmed is not True or not isinstance(note, str):
                raise ValueError('explicit_return_confirmation_required')
            if type(expected_attempt) is not int or expected_attempt != self.state['attempts']:
                raise ValueError('stale_reset_confirmation_refresh_panel')
            acknowledged = self._acknowledge('human_authorized_reset')
            self._record_operator_continuation(note, readiness='awaiting_return')
            from .reset import JOINT_THRESHOLD_RAD, TCP_THRESHOLD_M
            self.state['reset_policy'] = dict(automatic_small_motion=True,
                joint_threshold_rad=JOINT_THRESHOLD_RAD, tcp_threshold_m=TCP_THRESHOLD_M)
            self.state['interventions'].append(dict(time=time.time(), kind='human_authorized_reset',
                before_attempt=self.state['attempts'], note=note, acknowledged_robot_state=acknowledged))
            self.save('returning')
            def resume():
                if self._budget_exhausted() or self._restore_initial():
                    self._cycle()
            return self._launch(resume)

    def _restore_initial(self, *, automatic=False):
        """Shared automatic/manual return. Never clear faults or pause automatically."""
        self.check_session(self.state['session_id'])
        if self.stop.is_set() or self.runtime.paused:
            self.save('paused'); return False
        if self.runtime.faulted or self.runtime.closed:
            self.save('waiting_operator', reason='runtime_fault_or_unknown_outcome'); return False
        self.save('returning')
        self.step('RESET')
        self.step('CONFIRM')
        self.step('OBSERVE')
        targets = self.state['initial']['observation']['robot_state']['joint_positions']
        self._authorize()
        try:
            tools = ProgramExecutor(self.runtime, self.context()).api.tools()
            with self.runtime.program_lease(self.stop.is_set):
                self.check_session(self.state['session_id'])
                if automatic:
                    from .reset import ResetMotionAudit
                    current = self.runtime.get_robot_state()
                    fresh = ResetMotionAudit(self.state['initial']['observation']['robot_state'], current).result()
                    if not fresh['eligible']:
                        self.save('waiting_reset', reason='scene_changed_before_automatic_reset')
                        return False
                if hasattr(self.runtime.adapter, 'plan_recorded_return'):
                    result = self.runtime.return_recorded_joints(targets)
                else:
                    plan = tools['plan_motion'](targets['primary']) if set(targets) == {'primary'} else tools['plan_synchronized_motion'](targets)
                    result = tools['execute_trajectory'](plan.trajectory) if plan.ok and not self.stop.is_set() else plan
                verified = False
                if result.ok and not self.stop.is_set() and not self.runtime.paused:
                    import numpy as np
                    measured = self.runtime.get_robot_state()
                    verified = (set(measured.joint_positions) == set(targets) and all(
                        np.isfinite(measured.joint_positions[a]).all() for a in targets))
                    if not self.is_arx:
                        verified = verified and all(
                            np.max(np.abs(measured.joint_positions[a] - targets[a])) < .02 for a in targets)
                entry = dict(time=time.time(), kind='arm_return_result', result=compact(result),
                             verified=bool(verified) if not self.is_arx else None,
                             command_completed=bool(verified))
                self.state['interventions'].append(entry)
                self.state['history'].append({'reset_to_initial': entry})
                if self.stop.is_set() or self.runtime.paused:
                    self.save('paused'); return False
                if self.runtime.faulted or not result.ok or not verified:
                    self.save('waiting_operator', reason='initial_return_failed_or_unverified'); return False
                continuation = self.state.get('operator_continuation')
                if (not automatic and continuation
                        and continuation['for_attempt'] == self.state['attempts']):
                    continuation['readiness'] = 'return_command_completed'
                self.save('observing', reason='initial_return_verified' if not self.is_arx else 'initial_return_command_completed')
                return True
        finally:
            self._revoke()

    def return_to_initial(self, session_id, confirmed):
        with self.lock:
            self.check_session(session_id)
            if self.busy() or self.state['status'] not in WAITING: raise ValueError('experiment_not_waiting_for_reset')
            if confirmed is not True: raise ValueError('explicit_return_confirmation_required')
            acknowledged = self._acknowledge('human_requested_arm_return')
            entry = dict(time=time.time(), kind='human_requested_arm_return')
            if acknowledged: entry['acknowledged_robot_state'] = acknowledged
            self.state['interventions'].append(entry)
            self.save('returning')
            self.step('RESET')
            def move():
                self._authorize()
                executor = ProgramExecutor(self.runtime, self.context())
                tools = executor.api.tools()
                initial = self.state['initial']
                targets = initial['observation']['robot_state']['joint_positions']
                with self.runtime.program_lease(self.stop.is_set):
                    if initial['device_binding'] != self.runtime.adapter.device_binding():
                        raise ValueError('experiment_device_changed')
                    if hasattr(self.runtime.adapter, 'plan_recorded_return'):
                        result = self.runtime.return_recorded_joints(targets)
                    else:
                        plan = tools['plan_motion'](targets['primary']) if set(targets) == {'primary'} else tools['plan_synchronized_motion'](targets)
                        result = tools['execute_trajectory'](plan.trajectory) if plan.ok and not self.stop.is_set() else plan
                self.state['interventions'].append(dict(time=time.time(), kind='arm_return_result', result=compact(result)))
                self.save('paused' if self.stop.is_set() else 'waiting_operator' if self.runtime.faulted or not result.ok else 'waiting_reset',
                          reason='restore_gripper_and_objects_manually_then_confirm')
            return self._launch(move)

    def pause(self, session_id):
        with self.lock:
            self.check_session(session_id)
            if self.state['status'] in TERMINAL: raise ValueError('experiment_finished')
            self.stop.set()
            if not self.busy(): self.save('paused', pending=None)
            self.state['interventions'].append(dict(time=time.time(), kind='pause_requested'))
            self.save()

    def end(self, session_id):
        with self.lock:
            if not self.state or session_id != self.state['session_id']: raise ValueError('experiment_session_changed')
            if self.state['status'] in TERMINAL: raise ValueError('experiment_finished')
            if self.busy(): raise ValueError('pause_and_wait_before_ending')
            self.state['interventions'].append(dict(time=time.time(), kind='human_ended_experiment'))
            self.step('END')
            self.save('stopped', pending=None)
            self._revoke()
