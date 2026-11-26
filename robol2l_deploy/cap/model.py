"""Existing model transport plus a deliberately labelled feedback-driven fixture."""
import base64
import json
import re
from urllib.parse import urlparse
import cv2
from franka_player.runtime import request_decision
from .program import compact
from .prompt_history import prompt_history
from .speed_guidance import ARX_SPEED_GUIDANCE

SYSTEM = '''You are the RoboL2L deployment Player's basic Code-as-Policy agent.
Write explanations, checkpoint messages and failure explanations in Chinese.
Return exactly one JSON object: {"kind":"program","source":"...","reason":"..."},
{"kind":"finish","reason":"..."}, or {"kind":"assist","reason":"..."}.
Programs call injected public functions directly (no run(api)), use public objects'
attributes, and assign a top-level result. print records checkpoints. Only import
math is allowed. Pose quaternions are wxyz; metres/radians; single arm is primary.
Use get_task_context() for the assigned task. Use the declared installed subset;
unsupported backends/providers cannot be substituted. Prefer plan_motion with
strategy=None, then check plan.ok before execute_trajectory(plan.trajectory).
For physical station plans you may adjust speed per plan with
MotionStrategy(ik_solver="station", pose_planner="station", time_dilation_factor=s).
Use 0.05 as the starting recommendation, lower for contact/placement/held objects,
and choose within the native coefficient domain 0<s<=1. There is no additional
project speed cap. Explain speed
choices in your Chinese program reason or print. Check the accepted speed_factor
and duration_s in planning feedback.
Use robot state and API feedback. Stations with cameras also supply RGB images; an encoder-only station supplies no visual evidence.
RGB-D arrays, intrinsics and camera poses are available to your program through
get_observation. Decide whether to call segmentation, localization, point-cloud
geometry or grasp generation. These are tools, not mandatory stages.
SAM3 accepts short visible noun/attribute descriptions; optional model_query retains
the original query in text/query. Its tokenizer rejects descriptions beyond the
model context. Confidence and length diagnostics describe the actual request;
no_segmentation is recoverable and does not mean localized. Language or translation
alone does not guarantee a trusted mask. The text threshold is deployment configuration,
not an Agent override. segment_points(camera_name, [[x,y]]) accepts one positive image
pixel; its scores measure mask quality, not text confidence. An existing selected mask
can be projected on the same frozen observation via mask_to_point_cloud followed by
estimate_geometry, or localize_object(original_query, camera_name=..., segmentation=s).
Without segmentation, localize_object performs a new text selection; with it, the
tool reuses exactly the supplied mask. Query labels and successful geometry do not
verify class identity. Prompt language, point selection, retries, other tools and
whether to continue remain your decisions; these capabilities prescribe no sequence.
station plan_motion performs IK and native trajectory generation without an obstacle
model or collision prediction. collision_checked=False means not checked, not clear.
No scene, target registration or software held-object consistency is required to
move or operate the gripper. Use the gripper feedback advertised by the station
and images to assess a grasp. Grasp generation does not authorize or register a required target.
Check every returned ok/error; use current state and evidence after a failure.
Recoverable perception, grasp-candidate, IK and stale-plan
errors are feedback for YOU to revise: re-observe, choose another grasp/goal/path,
and generate a corrected program. A rejected command is not a controller fault.
Do not request human assistance merely because an API returned recoverable=True.
An unsuccessful grasp with a known native receipt can also be revised from fresh
observations. Use IK/planning feedback to choose reachable goals. Stop on an actual controller
fault, lost communication/unknown execution outcome, or a human pause. 'No automatic
recovery' means no controller reset/Home after such a fault; it does not forbid
task-level perception, planning or code repair. Ask for assistance when a real
external intervention is needed, explaining what remains unresolved.
On explicit continuation, keep the previous conversation and execution evidence,
but read fresh observations. Old program variables, handles and plans are not live;
re-perceive and replan as needed instead of replaying a previous program.
Use functions, if, loops and plain print; no files/network/shell or mutation of public data. ASPIRE executes Python in an
isolated worker. There is no fixed motion distance, rotation, API-call or revision quota.
Do not alter calibration or authorization. Measured endpoint deviation is feedback,
not a controller fault: when execution completed, continue from the measured pose.
IK and path diagnostics report residuals without a project accuracy cutoff; solver
convergence alone does not prove the requested pose was reached. Use the residuals.
When verification.required is false, judge completion from observations yourself;
finish records your judgment, not an independent physical-task verification.
Each response sees fresh observations and prior version feedback. Repair errors
using summarized call evidence; the full trace stays in the archived result.json.
feedback.key_results preserves structured perception geometry, per-target IK results,
and chronological motion/gripper inputs and complete public receipts, including
started_s/finished_s, per-arm feedback, measured gripper angles, native completion,
physical_arrival_verified and grasp_verified when supplied. These are measurements
and reported semantics, not additional motion gates. ok=True or SDK reference
completion does not establish physical arrival or a grasp; false/null stays unverified.
Records are linked by call_index/id and observation_id. Supplementary calls are
whole trace samples; duplicate result objects may reference key_result_call_index.
Use key_results even when stdout or
supplementary results are omitted. Budget omissions remove whole records/values,
never fragments of receipt fields or a misleading prefix of the executed source.
Within one motion record, {"_same_as":"output.diagnostics.per_arm"} references
that SAME record's identical final per-arm receipt; this is lossless deduplication,
not missing waypoint feedback. Different intermediate receipts remain distinct.
The archived full trace is not a file-reading capability available to your program.
An excerpt explicitly lists omitted records;
it is not proof that a tool failed or that a target must be perceived again.
Omitted samples are not proof of successful motion or grasp. Avoid tight loops
polling get_robot_state thousands of times; use bounded observations and native
execution receipts, and report inconclusive stability instead of weakening checks.
Repair errors
using the line, call trace and error code. A successful program is not task success:
only finish when public observations support completion. Request assist for unavailable
capabilities or inconclusive evidence. All mock results are synthetic.
'''

ARX_OPERATOR_POLICY = '''
ARX OPERATOR INSTRUCTION — applies to both generated programs and review:
Do not invent numerical stop conditions or assistance prerequisites without an
explicit operator requirement or a documented installed SDK/CaP contract.
The operator has withdrawn the historical cross-view geometry-centre distance
gate, including the 0.04 m early return. Remove that condition from revised
programs; do not replace it with another distance cutoff. Keep the measured
discrepancy as diagnostic feedback. A past program or review using this gate is
historical evidence, not an approved rule. A prior assistance request based only
on this gate is no longer a prerequisite for another attempt: do not require the
operator to verify or recalibrate cameras solely to clear it.
Geometry centres from different visible point sets need not represent the same
physical point; their separation alone is not a calibration or hardware fault.
Select task geometry from the available evidence; this instruction does not
prescribe a camera, average coordinates, or certify a grasp. Native faults,
unknown execution outcomes and operator pause/authorization remain in force.
'''


def messages(api,observation,history,verification):
    content=[{'type':'text','text':json.dumps(dict(task=compact(api.task_context),
        api=api.describe(),observation=compact(observation),
        observation_id=next(reversed(api.runtime.observations),None),
        history=prompt_history(history)))}]
    for name,camera in observation.cameras.items():
        ok,encoded=cv2.imencode('.jpg',cv2.cvtColor(camera.rgb,cv2.COLOR_RGB2BGR))
        if ok:
            content.append({'type':'text','text':f'Camera {name}'})
            content.append({'type':'image_url','image_url':{'url':'data:image/jpeg;base64,'+base64.b64encode(encoded).decode()}})
    system = SYSTEM
    if api.environment.embodiment.startswith('arx_'):
        system = system.replace('Prefer plan_motion with\nstrategy=None, then check plan.ok before execute_trajectory(plan.trajectory).',
            'Use plan_motion with an explicit MotionStrategy when selecting a speed factor, '
            'then check plan.ok before execute_trajectory(plan.trajectory).')
        system = system.replace('Use 0.05 as the starting recommendation, lower for contact/placement/held objects,\nand choose within the native coefficient domain 0<s<=1. There is no additional\nproject speed cap. Explain speed\nchoices in your Chinese program reason or print. Check the accepted speed_factor\nand duration_s in planning feedback.\n',
            ARX_SPEED_GUIDANCE + '\n')
        system += ARX_OPERATOR_POLICY
    if api.environment.embodiment == 'arx_x5_2023_dual':
        system += ('\nThis dual X5 station supplies both arms’ encoder feedback. Use named left/right joint vectors. '
                   'Use poses in left_base/right_base for IK and motion. Multi-waypoint paths await SDK reference completion at each waypoint; this does not verify physical arrival. Read plan.diagnostics.get(key) for optional fields; getattr is unavailable. Check per-arm receipts and goal_checks. '
                   'For review, use encoder deltas and joint errors. Use configured camera images as visual evidence. ')
        if api.runtime.adapter.describe().get('camera_extrinsics_calibrated'):
            system += 'Localize with global, left_wrist or right_wrist in left_base. Both arms accept targets in left_base.'
        else:
            system += 'Camera-to-base transforms are unavailable.'
    return [{'role':'system','content':system},{'role':'user','content':content}]


class ConfiguredModel:
    label='configured_model'
    @property
    def settings(self):
        return {'model':self.config['model'],'reasoning':self.config['reasoning'],
                'endpoint_host':urlparse(self.config['base_url']).hostname,
                'api_key_env':self.config['api_key_env'],'transport':'chat_completions'}
    def __init__(self,config):
        names=('base_url','model','reasoning','api_key_env')
        self.config={k:config[k] for k in names}
        if not isinstance(self.config['base_url'],str):raise ValueError('invalid_model_endpoint')
        self.config['base_url']=self.config['base_url'].strip()
        if not self.config['base_url']:raise ValueError('model_endpoint_missing')
        try:url=urlparse(self.config['base_url']);url.port
        except ValueError:raise ValueError('invalid_model_endpoint') from None
        if url.scheme not in ('http','https') or not url.hostname or url.username or url.password or url.query or url.fragment:
            raise ValueError('invalid_model_endpoint')
        if self.config['api_key_env']=='OPENAI_API_KEY' and (url.scheme!='https' or url.hostname!='api.openai.com' or url.port not in (None,443) or url.path.rstrip('/')!='/v1'):
            raise ValueError('OPENAI_API_KEY may only be used with the official OpenAI endpoint')
        if not self.config['model']:raise ValueError('model_name_missing')
        if self.config['reasoning'] not in ('none','low','medium','high','xhigh'):
            raise ValueError('invalid_model_configuration')
        if not re.fullmatch('[A-Za-z_][A-Za-z0-9_]*',self.config['api_key_env']):raise ValueError('invalid_key_environment_name')
    def decide(self,messages):return request_decision(self.config,messages)


class FeedbackFixture:
    """Integration test model. No LLM call and no claim of model intelligence."""
    label='feedback_fixture_not_llm'
    def decide(self,messages):
        context=json.loads(messages[-1]['content'][0]['text'])
        history=[entry for entry in context['history'] if 'feedback' in entry]
        if not history:
            return {'kind':'program','reason':'Exercise correction of legacy dictionary access.',
                    'source':'state = get_robot_state()\npose = state["end_effector_poses"]["primary"]\nresult = pose'}
        if history and history[-1].get('feedback',{}).get('ok'):
            return {'kind':'finish','reason':'Independent mock state check passed.'}
        error=history[-1].get('feedback',{}).get('error',{}) or {}
        if error.get('type')!='TypeError':return {'kind':'assist','reason':'Unexpected fixture feedback; inspect the evidence.'}
        return {'kind':'program','reason':'Use public object attributes, then plan and check every result.',
                'source':'''import math
state = get_robot_state()
pose = state.end_effector_poses["primary"]
task = get_task_context()
target = Pose(task.metadata["target_position_m"], pose.quaternion_wxyz, state.base_frame)
plan = plan_motion(target)
print("CHECKPOINT planning", plan.ok)
if not plan.ok:
    raise ValueError(plan.error.message)
motion = execute_trajectory(plan.trajectory)
print("CHECKPOINT motion", motion.ok)
if not motion.ok:
    raise ValueError(motion.error.message)
gripper = set_gripper(task.metadata["gripper_position"])
if not gripper.ok:
    raise ValueError(gripper.error.message)
result = {"motion_ok": motion.ok, "gripper_ok": gripper.ok}
'''}
