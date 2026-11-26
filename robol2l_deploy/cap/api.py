"""ASPIRE composition root. Device ownership remains in StationRuntime."""
from dataclasses import replace
from functools import wraps
import inspect
import numpy as np
from cap_harness.api import CapApi
from cap_harness.registry import ToolRegistry
from .. import contracts as c
from ..errors import ApiError, ErrorCode
from ..runtime import OutcomeUnknown, fingerprint
from ..adapters.arx_gripper_feedback import ARX_GRASP_SEMANTICS
from .speed_guidance import ARX_SPEED_GUIDANCE

CONSTRUCTORS = {name:getattr(c,name) for name in (
    'Pose','MotionStrategy','ArmCommand','RobotAction','Trajectory','SynchronizedTrajectory')}
MOTION = frozenset({'execute_trajectory','set_gripper','step','go_home','move_to_pose',
    'move_to_joints','move_synchronized','set_grippers','open_gripper','close_gripper'})


class StationEnvironment:
    """ASPIRE adapter over the existing owner, with a frozen perception epoch."""
    manages_trajectory_dispatch = True

    def __init__(self, runtime, task_context):
        self.runtime, self.task_context = runtime, task_context
        self.embodiment = runtime.adapter.describe()['embodiment']
        self.observation = None
        self.observation_id = None
        self.binding = None

    def refresh(self):
        self.observation = replace(self.runtime.get_observation(), task_context=self.task_context)
        self.observation_id = next(reversed(self.runtime.observations))
        binding = self.runtime.adapter.device_binding() if hasattr(self.runtime.adapter,"device_binding") else self.runtime.adapter.binding()
        self.binding=binding
        return self.observation

    def invalidate(self):
        self.observation = None

    def get_observation(self):
        if self.observation is None:
            return self.refresh()
        return self.observation

    def prepare_projection(self, segmentation, observation):
        # Record the selected camera for provider evidence; accept ASPIRE's raw
        # boolean masks as well as Segmentation objects. Geometry filters depth.
        self.perception_camera_name = next((name for name,camera in observation.cameras.items()
            if segmentation.camera_name in (name,camera.frame)),segmentation.camera_name)
        return segmentation

    def monitor_snapshot(self):
        return self.runtime.snapshot()

    def resolve_arm(self, arm):
        aliases = self.runtime.adapter.describe().get('arm_aliases', {})
        return aliases.get(arm, arm)

    def get_task_context(self):
        return self.task_context

    def get_robot_state(self):
        return self.runtime.get_robot_state()

    def get_planning_context(self):
        state = self.get_robot_state()
        adapter = self.runtime.adapter
        model = getattr(adapter, 'model_name', 'fr3' if self.embodiment == 'franka_fr3' else 'mock')
        ee_link = getattr(adapter, 'end_effector_link', 'fr3_hand_tcp')
        transforms = {arm: np.eye(4) for arm in state.arms}
        if state.base_frame == "arm_local":
            calibrated = adapter.collision.base
            transforms = calibrated if set(calibrated) == set(state.arms) else {}
        elif self.embodiment == 'arx_x5_2023_dual' and state.base_frame == 'left_base':
            transforms['right'] = adapter.extrinsics['T_left_base_from_right_base']
        links = getattr(adapter, 'end_effector_references', {arm: ee_link for arm in state.arms})
        return c.RobotPlanningContext(self.embodiment, model, state.joint_names, transforms, links)

    def planning_scene_request(self, scene, arm):
        return scene

    def _execute(self, operation, *args, **kwargs):
        try:
            return operation(*args, **kwargs)
        except OutcomeUnknown as error:
            return c.ExecutionResult(False, 0, error=ApiError(ErrorCode.OUTCOME_UNKNOWN,
                error.diagnostics.get('cause_message','outcome_unknown'), recoverable=False,details=error.diagnostics),
                diagnostics=dict(error.diagnostics,outcome='unknown',steps_count_known=False))
        finally:
            self.invalidate()

    def execute_trajectory(self, trajectory):
        return self._execute(self.runtime.execute_trajectory, trajectory)

    @property
    def collision_missing(self):
        checker = getattr(self.runtime.adapter, 'collision', None)
        return checker.missing if checker is not None else ['collision_checker']

    def execute_planned_move(self, trajectory, targets, tolerance, max_steps):
        return self._execute(self.runtime.execute_trajectory, trajectory, targets=targets,
                             tolerance=tolerance, max_steps=max_steps)

    def set_gripper(self, position, *, arm='primary'):
        return self._execute(self.runtime.set_gripper, position, arm=arm)

    def close_gripper(self, *, arm='primary'):
        return self._execute(self.runtime.close_gripper, arm=arm)

    def set_grippers(self, positions):
        return self._execute(self.runtime.set_grippers, positions)

    def go_home(self, **kwargs):
        return self._execute(self.runtime.go_home, **kwargs)

    def step(self, action):
        try:return self.runtime.step(action)
        finally:self.invalidate()


class StationPlanner:
    # This provider generates a native joint trajectory, without an environment model.
    requires_observed_scene = False
    accepts_start_joints = True

    def __init__(self, runtime, environment):
        self.runtime = runtime
        self.environment = environment

    def solve_ik(self, target_pose, robot_state, *, arm='primary'):
        return self.runtime.solve_ik(target_pose, robot_state.joint_positions[arm], arm=arm)

    def plan_to_pose(self, robot_state, target_pose, *, arm='primary', **kwargs):
        options={key:kwargs[key] for key in ('start_joints','interpolation_dt_s','maximum_trajectory_dt_s')
                 if kwargs.get(key) is not None}
        if kwargs.get('time_dilation_factor') is not None:options['speed_factor']=kwargs['time_dilation_factor']
        result=self.runtime.plan_motion(target_pose,arm=arm,**options)
        if not result.ok:return result
        trajectory=replace(result.trajectory,expected_start=robot_state)
        self.runtime.bind_public_plan(result.diagnostics['plan_id'],trajectory)
        return replace(result,trajectory=trajectory)

    def plan_to_joints(self, robot_state, joint_goal, *, arm='primary', **kwargs):
        return self.plan_to_pose(robot_state,joint_goal,arm=arm,**kwargs)

    def plan_synchronized_motion(self, robot_state, targets, **kwargs):
        if hasattr(self.runtime.adapter, 'plan_pair'):
            return self.runtime.plan_pair(robot_state, targets, **kwargs)
        if set(targets)!={'primary'}:
            return c.SynchronizedPlanResult(False,error=ApiError(ErrorCode.NOT_FOUND,'arm_unavailable'))
        result=self.plan_to_pose(robot_state,targets['primary'],**kwargs)
        if not result.ok:return c.SynchronizedPlanResult(False,error=result.error)
        t=result.trajectory
        trajectory=c.SynchronizedTrajectory({'primary':t.joint_positions},t.dt_s,
            {'primary':t.joint_names},t.planner,t.collision_aware,t.expected_start,embodiment=t.embodiment)
        return c.SynchronizedPlanResult(True,trajectory=trajectory,diagnostics=result.diagnostics)


class PublicAPI:
    """Compatibility facade; all public operations come from the shared CapApi."""
    def __init__(self, runtime, task_context, *, providers=None):
        self.runtime, self.task_context = runtime, task_context
        self.environment = StationEnvironment(runtime, task_context)
        from ..providers.perception import configured_providers
        self.providers = configured_providers(self.environment) if providers is None else providers
        planner = StationPlanner(runtime, self.environment)
        # Home is captured by Runtime on the first measured state; no constructor I/O.
        self.core = CapApi(self.environment, default_camera='global' if 'global' in runtime.adapter.cameras else next(iter(runtime.adapter.cameras),'global'),
            home_joint_positions={},
            segmentation_provider=self.providers.get('segmentation'),
            grasp_providers=self.providers.get('grasps', {}),
            ik_providers={'station':planner}, integrated_pose_planning_providers={'station':planner},
            trajectory_planning_providers={'interpolation':planner},
            default_motion_strategy=c.MotionStrategy(ik_solver='station', pose_planner='station'))
        self.registry = self.core.register_tools(ToolRegistry())

    @wraps(CapApi.get_observation)
    def get_observation(self, camera_names=None):
        self.environment.refresh()
        return self.core.get_observation(camera_names)

    def tools(self):
        unsupported = getattr(self.runtime.adapter, 'unsupported_operations', ())
        result = {spec.name:spec.function for spec in self.registry.all_specs(public_only=True)
                  if spec.name not in unsupported}
        if 'get_observation' in result:
            result['get_observation'] = self.get_observation
        original = result['plan_motion']
        @wraps(original)
        def registered_plan(*args, **kwargs):
            plan = original(*args, **kwargs)
            if plan.ok:
                key = plan.diagnostics.get('plan_id')
                if key is not None:self.runtime.bind_public_plan(key, plan.trajectory)
            return plan
        result['plan_motion'] = registered_plan
        if self.environment.embodiment == 'arx_x5_2023_dual':
            def go_home(*, arm='both'):
                return self.core.go_home(arm=arm)
            result['go_home'] = go_home
        return result

    def describe(self):
        capabilities=self.runtime.describe()
        # Runtime lists device primitives; the Player also has shared perception
        # and geometry tools. Do not advertise the lower-layer list as its subset.
        capabilities['runtime_operations']=capabilities.pop('supported')
        available=set(self.tools())
        if not self.providers.get('segmentation'):
            available-={'segment_text','segment_points','localize_object'}
        if not self.providers.get('grasps'):available.discard('generate_grasps')
        capabilities['supported']=sorted(available)
        description = {'format':'ASPIRE Python; injected functions/types; top-level result; captured print; import math only',
            'tools':{n:str(inspect.signature(f)) for n,f in self.tools().items()},
            'constructors':list(CONSTRUCTORS), 'capabilities':capabilities,
            'perception_providers':self.core.configured_providers,
            'perception_input_guidance':{
                'geometry':'estimate_geometry reports the bounding geometry of the supplied visible point set. Its pose centre is not a verified object centre or a corresponding physical landmark across cameras. Different masks, visibility and depth selections can change that centre. Cross-view centre separation is diagnostic feedback, not by itself calibration error or a motion prerequisite.',
                'depth':'mask_to_point_cloud and localize_object accept optional depth_range_m=[min,max] (camera Z in metres), and seed_px=[x,y] together with max_depth_jump_m (4-neighbour depth continuity). These explicitly filter only inside the supplied mask; no largest-cluster selection or mask expansion. Defaults keep all finite positive masked depths. Inspect cloud.diagnostics.projection or localized.diagnostics.projection for input/output depth quantiles, rejected counts and retained_fraction. A seed outside the valid selected mask fails without fallback. Continuity can follow a gradual depth ramp; use a depth range when a total depth bound is needed. These measurements do not verify object identity or sensor accuracy; a wrong table mask remains wrong after depth filtering.',
                'text':'segment_text(camera_name, text, model_query=...) and localize_object(query, camera_name=..., model_query=...) preserve the original query. Supply a short visible noun/attribute phrase, not task instructions or spatial reasoning. The actual tokenizer rejects inputs over the model context (32 tokens including specials for the pinned model). Short English visual descriptions worked in station samples; language or translation alone never guarantees detection.',
                'confidence':'Inspect ok/error and diagnostics: raw_scores, text_confidence_threshold, rejection_reason and token count. NO_SEGMENTATION is recoverable: revise the visual description or choose a point from the supplied image; do not lower thresholds or assume the query label confirms class. Point scores measure mask quality and are not text confidence.',
                'points':'segment_points(camera_name, [[x,y]]) accepts exactly one positive image pixel, x=column and y=row; no negative or multi-point prompts. Choose it on the target visible in that camera and frozen observation. Inspect candidates against the image; ranking alone does not confirm target identity.',
                'reuse':'After choosing a mask s from segment_points(...).segmentations, use mask_to_point_cloud(s, camera_name) then estimate_geometry(cloud), or localize_object(original_query, camera_name=..., segmentation=s). The latter performs no text inference. Reuse the same frozen RGB-D: do not refresh, move, or independently text-localize between mask selection and projection. Retained/raw masks remain accepted; caller is responsible for snapshot correspondence. localize_object ok means selected-mask geometry, not verified class or physically verified size.'},
            'grasp_semantics':'generate_grasps poses are already converted from model grasp frame to configured FR3 Hand TCP, in base. width_m is predicted opening, not a measured object width. The transform is model-derived, not physically commissioned.',
            'physical_motion_for_generated_programs':bool(self.runtime.physical_session and self.runtime.physical_session.permits(self.task_context.task_name)),
            'units':'metres, radians, seconds, quaternion wxyz; named frames; arm primary',
            'motion_speed_guidance':'For station pose planning, MotionStrategy(ik_solver="station", pose_planner="station", time_dilation_factor=s) selects this plan\'s native speed factor. Default/recommended s=0.05; choose lower near contact, with held objects or uncertain geometry, and within the native speed-factor range for clear transfers. Must satisfy the native domain 0<s<=1; no additional project speed cap. Smaller is slower; actual duration is provider-dependent, not a linear task-time guarantee. Check plan.diagnostics.speed_factor and duration_s. Omit strategy to use the configured default. This does not change subsequent plans, gripper speed, controller gains, collision settings or the server limit.',
            'observation_semantics':'The model receives camera RGB images each decision turn. get_observation returns RGB-D arrays, intrinsics, camera poses and robot state. Explicit perception calls share that frozen snapshot; raw masks and retained result references are accepted, and the caller selects which snapshot to use. The agent chooses whether to segment, localize or generate grasps. Planning and gripper operations never trigger perception, build an obstacle model or require a registered target.',
            'step_semantics':'step sends one joint target over the configured control_period_s (default 1/30 s); native FR3 dynamics can reject a target too far for that duration. Use move_to_joints for full native planned moves. Camera capture follows execution; this Python API is not a 1 kHz streaming servo.',
            'home_semantics':'go_home explicitly moves to configured home joints, or the session first measured joints persisted in home.json. It never runs automatically.',
            'execution_semantics':'All 24 public ASPIRE tools are registered. Single-arm dictionary operations use primary. Hand columns execute at change waypoints with blocking dwell, not simultaneous servo timing. Public constructed trajectories use their supplied dt; registration and single-use are not required. plan_motion solves IK and time-parameterizes a native trajectory; collision_checked=False. No software scene, collision or held-object inference blocks dispatch. close_gripper performs native force-contact closing; set_gripper commands opening width. Native ok/is_grasped/width and measured pose are feedback, not independent proof of task success.'}
        if self.environment.embodiment in ('arx_x5_2023', 'arx_x5_2023_dual'):
            description.update(
                ik_diagnostics='ARX solve_ik reports diagnostics.sdk_converged and diagnostics.sdk_residual on both solver success and nonconvergence. sdk_residual is the unchanged native solver residual, not a Cartesian position error in metres or physical arrival verification. Invalid inputs rejected before the solver have no solver residual.',
                motion_speed_guidance=ARX_SPEED_GUIDANCE,
                grasp_semantics=ARX_GRASP_SEMANTICS,
                step_semantics='This backend uses point-to-point moves via move_to_joints or move_to_pose.',
                execution_semantics='Plan from the latest measured state. The ARX SDK accepts one six-joint target with a duration. The station reports measured deviation and SDK reference completion. Physical arrival is checked only against caller-supplied tolerances. Scene collision checking is unavailable. Uncalibrated gripper state is a normalized encoder fraction.')
        if self.environment.embodiment == 'arx_x5_2023_dual':
            description.update(units='Joint positions in radians, per arm left/right. End-effector poses use independent arm-local frames.',
                observation_semantics='Both arms provide encoders and FK in their own base frames. Configured RGB-D cameras share frames with previews and recording. Camera poses are identities in their own optical frames. Robot-base camera extrinsics are unavailable.',
                execution_semantics='Use move_to_joints(target, arm="left" or "right") or move_synchronized({"left": six_joints, "right": six_joints}). Use left_base/right_base poses for solve_ik, plan_motion and move_to_pose. Native reference trajectories execute in order; measured error is reported. Both commands use one station owner. Synchronization is host-timed point-to-point with measured submission skew. Inspect diagnostics.per_arm and final_errors. Inspect collision_checked and collision_missing. MotionStrategy(ik_solver="station",pose_planner="station",require_collision_check=True) requires an available inter-arm model check.',
                step_semantics='One CaP control-period joint target for left, right or both; the SDK selects feasible execution timing. Check the returned observation.',
                home_semantics='go_home() or both returns both arms to station Home. Named arms return individually. Experiment reset uses its recorded initial snapshot.')
            if capabilities['camera_extrinsics_calibrated']:
                description.update(
                    units='Metres, radians, seconds, quaternion wxyz. Both measured end-effector poses use left_base.',
                    observation_semantics='global, left_wrist and right_wrist RGB-D project into left_base. '
                    'The global camera is fixed. Each wrist pose uses its own exposure time and measured FK. '
                    'Use localize_object(query, camera_name=name) or mask_to_point_cloud(mask, name) on a frozen '
                    'observation for a target in left_base. IK and planning accept left_base on either arm and '
                    'right_base on the right arm.')
            description['end_effector_semantics'] = (
                'Read capabilities.end_effector_references and capabilities.grasp_geometry per arm. '
                'With configured geometry, end_effector_poses, solve_ik, pose planning, move_to_pose and '
                'pose goal checks all refer to grasp_tcp; the adapter converts to/from native link6. '
                'Do not add a finger offset again. grasp_tcp axes are +Z approach toward the object, '
                '+X finger closing axis, +Y completing the right-handed frame. '
                'Without configured geometry these APIs still refer to link6, NOT the finger contact center. '
                'Official-model geometry is nominal CAD geometry, not physical TCP acceptance: inspect '
                'physically_verified and geometry.contact_geometry. The contact center is behind the '
                'forwardmost fingertip; use the pad bounds for approach clearance. '
                'gripper_calibrated only describes encoder endpoints and does not certify grasp geometry. '
                'Missing geometry cannot be replaced by treating a visible surface center or estimated '
                'finger length as a calibrated TCP. Wrist-camera extrinsics always use the native link6 frame. '
                'A configured rigid TCP must be verified invariant across gripper opening; '
                'normalized opening is not a width in metres. Physical grasp success still requires evidence.')
        return description
