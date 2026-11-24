# Modular hardware workbench / 模块化硬件工作台

This is the first implementation stage: typed hardware API, registered plans,
persistent camera owners, evidence and a Web workbench. The [basic CaP loop](basic-cap.md)
now supports mock validation and automatic read-only perception/planning; the Player learning protocol
remains a subsequent stage. The existing
legacy `def run(api)` interpreter is not advertised as the new public contract.

## Software-only use

```bash
.venv/bin/python -m franka_player.server --hardware-mock --hardware-motion --data-dir /tmp/robol2l-hardware-demo
```

Open `http://127.0.0.1:8765/hardware`. Every mock result is synthetic. The optional
motion flag only advances the mock state in this command. Without either hardware
option, the existing console remains unconfigured and opens no devices.

For separately authorized device acquisition, select `--hardware-station
config/station.json` instead of `--hardware-mock`. Motion remains disabled without
`--hardware-motion`; the installed adapter additionally requires reviewed scene
geometry. Do not use that flag as a substitute for physical commissioning. Existing
calibration files are read, never overwritten. FR3 now uses the compiled `panda_py` adapter. Its default motion guard remains off even if the Runtime flag is enabled. Build and provenance are described in [automatic perception](automatic-perception.md).

## Boundaries

- `robol2l_deploy.contracts/errors`: compatibility re-exports of the shared ASPIRE types; source pin and local changes are recorded in `cap_harness/UPSTREAM.json`.
- `StationRuntime`: one dispatch lease, immutable registered trajectory digest,
  measured start/configuration checks, durable jobs and observation/plan artifacts.
- `adapters`: vendor calls and conversions; camera owners retain one recent RGB-D
  frame. No SDK is initialized on import or construction.
- `web`: same-origin, typed allowlisted jobs; no source, shell or file-path endpoint.
- The legacy console and the new station cannot be selected together on the CLI.
  CaP receives this same Runtime and dispatch lease, without another adapter.

Shared calls implemented: `get_robot_state`, `get_observation`, `solve_ik`,
`plan_motion`, `execute_trajectory`, `set_gripper`, `open_gripper`, `close_gripper`. FR3 accepts a base-frame Pose,
measured start and default backend; other strategies return `unsupported`.
`step`, Home, multi-arm control and `max_steps` convenience moves remain unsupported.
Calling these cannot dispatch a native operation. SAM3 text segmentation and Contact-GraspNet are configured through `config/perception.json`; unavailable services return actual typed failures.

`Trajectory` samples use at most 20 ms spacing on the actual `panda_py.motion.JointTrajectory` curve; the same cached library object is used for dispatch. `execute_trajectory` accepts only a
matching current-session registered plan. It never replans from a submitted path.
`steps_executed` counts confirmed public reference samples, not FCI cycles; elapsed
seconds are recorded separately. Interrupted native execution without a terminal
receipt is `outcome_unknown`; no zero-step result is invented.

Gripper normalization is the full 80 mm Franka Hand travel. Native allowed widths
remain 15–80 mm. `close_gripper` uses native contact grasp (5 N) for one observed
graspable box between the fingers, with an expected jaw width of 15–70 mm; it is
not an alias for changing the meaning of normalized zero. A successful receipt
attaches the observed object relative to the measured TCP. Holding motion uses the same panda_py trajectory controller and checks retention before and
after each segment (not continuous slip sensing). `open_gripper` releases through
the same Runtime and records the body's predicted pose, which requires subsequent
visual verification. Release location and task sequence belong to the program.
No implicit Home, recovery or calibration runs.

`manipulation.py` holds device-neutral boxes, scene provenance and payload state;
`adapters/fr3_scene.py` checks FR3, fingers, wrist camera, table, observed boxes and
attached payload. Scene changes and grasp/release invalidate earlier plans. An
untracked held object after restart cannot be moved. A host/provider may install a
scene through `StationRuntime.install_scene`; this does not authorize motion or
expose adapter access to generated programs. Observation-derived scenes default
to unreviewed. The physical workbench remains read-only. The old native spline helper is retained for legacy evidence/tests and is not a fallback controller for the new adapter.

## Manual debug preview (excluded from autonomous CaP)

`robol2l_deploy.tasks.green_cube` is a retained manual debugging baseline, not an autonomous acceptance path: supplied
image regions, green RGB segmentation, depth plane fitting, cube inference and a
plate bottom/rim approximation. It is not a universal perception API. Task names,
pixel regions and grasp/placement choices do not live in the robot adapter.

```bash
.venv/bin/python -m robol2l_deploy.tasks.prepare_pick_place \
  --observation /absolute/path/to/observation-ID \
  --raw-state /absolute/path/to/matching-raw-state.json \
  --regions /absolute/path/to/regions.json \
  --output /absolute/path/to/new-preview-directory
```

Regions are JSON pixel rectangles: `cube`, `plate`, a list `table`, and optional
list `obstacles`. They must refer to that exact frozen image. Raw state must match
the frozen observation's timestamp and joints. The command never reads live
hardware or dispatches motion. It writes geometry, an explicitly labelled review
candidate, actual spline/FK previews and per-segment collision/dynamics results.
Future start states are propagated predictions; these files cannot be submitted
as executable Runtime plans. Hidden geometry, grasp success and object settling
remain assumptions. Failures stop the preview and identify unchecked phases.

RGB-D observations are stationary-bracketed. Gemini Brown-Conrady RGB/depth are
rectified together; depth uses nearest-neighbor sampling. Raw RGB-D and metadata
are saved separately from rectified public observations. Preview streams are live
raw camera pixels; freeze changes to the rectified observation pixel space used
for measurement. Devices are not hardware-synchronized. A stream error invalidates
the latest frame; no fixture substitutes for a device failure.

## HTTP and evidence

`POST /api/hardware/jobs` takes `request_id`, `operation`, `arguments` and returns
202. Repeating an identical request ID returns its original job; reusing an ID
with different inputs is rejected. Busy stations reject new jobs rather than queue
movement. Execution arguments contain only `plan_id`. `GET /api/hardware/state`
returns capabilities and jobs. Streams/images, pixel localization and surface-region
measurement have separate typed endpoints. Mutation requests require the existing
loopback Host/same-origin checks and `X-Console-Request: 1`.

Pause blocks the next dispatch; active controller execution completes or reports
its own failure. Restart never resumes a job. An unresolved sent job becomes
`outcome_unknown` and latches motion off; preserve its evidence for operator review.
Start a new reviewed session rather than rewriting a terminal record.

`events.jsonl`, `jobs.json`, `plan-*/plan.json` and `observation-*` live below the
chosen data directory. No model credentials are accepted. Existing experiment
records are preserved unchanged.

## Checks

```bash
.venv/bin/python -m unittest discover -s tests -v
```

Tests use mock devices, loopback HTTP and native `--sample-time` (no connection).
They cover contract provenance, immutable arrays, 6/7 DOF, arbitrary cameras,
rectification, stale/modified/consumed plans, request deduplication, pause during a
blocked dispatch, unknown outcomes and restart. These checks are not physical
commissioning or task-success evidence.
