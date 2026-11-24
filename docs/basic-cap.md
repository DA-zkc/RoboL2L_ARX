# Basic Code-as-Policy loop / 基础 CaP 闭环

Current API support and hardware semantics are documented in [ASPIRE CaP parity](aspire-cap-parity.md). Code generation, execution, feedback and revision use the same StationRuntime for mock and explicitly enabled physical sessions. Startup remains read-only.

## Run offline

```bash
.venv/bin/python -m robol2l_deploy.cap.cli mock
.venv/bin/python -m robol2l_deploy.cap.cli prepare-pick-place
```

The default model is **feedback_fixture_not_llm**. It deliberately generates a
legacy dictionary-access error in v00, reads the returned TypeError and repairs
it in v01. The corrected program plans and moves the mock end effector, sets its
gripper, and checks every result. Completion requires independent measured mock
state, not the program's `result` or the model's assertion. This validates plumbing,
not LLM reasoning, grasp physics or physical success.

The Web workbench has the same loop in mock mode:

```bash
.venv/bin/python -m franka_player.server --hardware-mock --hardware-motion --port 8767 --data-dir .franka-player/cap-workbench
```

Open `/hardware`, select **CaP → 模型入口 → 使用 Codex 额度 / Codex plan** to
use the existing local ChatGPT login with `gpt-6-astra`, medium reasoning. No API
key is needed. The panel shows the selected provider's login/key status and saves
the selection for this experiment, including across a service reload. Login
readiness does not report remaining quota; usage limits appear as Chinese errors
in the task feedback, without falling back to an API provider. The existing API
option remains available. Codex only generates program text and receives the same
observations and API feedback as the API model; execution stays in the Runtime.

You can also select the feedback fixture or a configured model. The configured
model uses the existing console's nonsecret model settings and server environment.
The physical workbench also offers automatic perception and read-only planning with a real model; fixture runs remain mock-only. See [automatic perception](automatic-perception.md).

## Real model, mock hardware

Use an existing ChatGPT-authenticated Codex CLI to consume Codex usage without an
API key:

```bash
.venv/bin/python -m robol2l_deploy.cap.cli mock --codex-model gpt-6-astra --validate
```

The selected model must be supported by the installed CLI version. An isolated
official CLI 0.169.0 is installed locally for this workspace and is selected
automatically before the global CLI. An explicit override is also supported:

```bash
.venv/bin/python -m robol2l_deploy.cap.cli mock --codex-model gpt-6-astra \
  --codex-bin .franka-player/tools/codex/node_modules/.bin/codex --validate
```

The global CLI installation and existing login are unchanged. This local tool
directory is ignored by Git; it is not a bundled project dependency.

This uses the official [`codex exec`](https://learn.chatgpt.com/docs/non-interactive-mode)
interface. It verifies the CLI login method, ignores user provider configuration,
does not inherit API keys, and disables shell, browser, plugin and other agent
tools. Each request runs read-only in a temporary directory and returns structured
program text. Only the existing CaP evaluator dispatches robot APIs; this CLI example uses mock.
No login token is read, copied or exported by the deployment code. Server overrides
are `ROBOL2L_CODEX_MODEL`, `ROBOL2L_CODEX_REASONING`, and `ROBOL2L_CODEX_BIN`;
they are optional. ChatGPT login uses [Codex subscription access](https://learn.chatgpt.com/docs/auth),
separate from the API provider's billing.

`--validate` runs two fresh mock stations: autonomous generation/execution and
repair of an injected dictionary-access error. The injected version is explicitly
labelled as test input, never attributed to the model. Subsequent revisions use
the real model and actual executor feedback. `validation.json` records the two
results, decision origins, model settings and CLI token usage. A first-pass success
in generation alone is not reported as proof of error repair. This is a local,
trusted workflow; do not expose the authenticated CLI to public requests.

For a direct OpenAI connection, enter the key through the hidden terminal prompt:

```bash
.venv/bin/python -m robol2l_deploy.cap.cli mock --openai-model gpt-6-astra --prompt-key
```

Alternatively set `OPENAI_API_KEY` in the process environment and omit `--prompt-key`.
Do not put a key in a file, command argument, browser form, URL or exported evidence.
The prompt requires a TTY and uses process memory only. This mode fixes the endpoint
to `https://api.openai.com/v1`; it never reuses the older third-party endpoint.
Model access depends on the account. GPT-6 Astra supports image input and Chat
Completions: [official model reference](https://developers.openai.com/api/docs/models/gpt-6-astra).

Other existing providers can use `--model-config` with exactly `base_url`, `model`,
`reasoning` and `api_key_env`. Credentials still come only from the environment.
`OPENAI_API_KEY` is rejected with a non-OpenAI endpoint. Provider failures do not
persist response bodies or credential-bearing headers.

## Program contract and scope

Programs use the RoboL2L format: injected public tools, six constructors, typed
read-only returned objects, `primary` arm mappings, metres/radians, named frames,
`wxyz`, captured `print`, and top-level `result`. For example:

```python
state = get_robot_state()
pose = state.end_effector_poses["primary"]
target = Pose([pose.position[0], pose.position[1], pose.position[2] + 0.005],
              pose.quaternion_wxyz, state.base_frame)
plan = plan_motion(target)
print("CHECKPOINT planning", plan.ok)
result = {"plan_ok": plan.ok}
```

This example only plans. Physical motion is available through the station gateway when the operator starts an enabled physical session.
The runtime owns every API call; no program receives an adapter or Runtime object.
The legacy `franka_cap.Program` remains a separate historical entrypoint.

`cap_harness` is a pinned subset of ASPIRE, not a second API implementation.
`PublicAPI.core` composes its `CapApi` with `StationEnvironment`, the station planner
and real providers. `contracts.py` and `errors.py` re-export the same class objects.
The upstream `ProgramExecutor` is reused with a small NumPy reduction compatibility patch; its source pin and patch hash are tested. It validates
and executes Python, including functions, f-strings, comprehensions and `import math`.

The executor runs in a separate Linux process with typed JSON RPC. After imports,
seccomp denies file opens, networking, process creation and process inspection.
The worker receives no credentials, Runtime or driver object. Source validation,
public types and the host dispatch boundary constrain execution; there is no default project call, source-size, step or revision quota.
Pause is checked between calls; an in-flight installed controller is never killed
by a Python timeout. `cap/language.py` is retained history, not an active executor.

Home, direct step, joint goals and primary-arm dictionary operations are implemented. A second physical arm and unconfigured planner backends are not available.
Runtime receipt loss remains `outcome_unknown`. Mock oracle results are stored for
test evaluation and excluded from model context, including failed-finish feedback.

`close_gripper`, held-object `execute_trajectory` and `open_gripper` now share the
Runtime dispatch gate. Programs compose them using ordinary functions and control
flow. The driver has no green-cube/plate sequence or task-success predicate. Scene
modeling is optional and never enables motion or blocks physical dispatch. See the parity document for the current native controller and gripper timing semantics.

## Organization and artifacts

- `cap/api.py`: the public tool binding and capability description.
- `cap/worker.py`, `program.py`, `wire.py`: isolated upstream execution, source hash, stdout and
  per-call source lines, inputs, result/error and observation references.
- `cap/model.py`, `session.py`: model transport/fixture and bounded revision loop.
- `cap/tasks.py`: independent task checks and physical task preparation.

Each session is new: `v00/source.py`, `v00/result.json`, model decisions/context,
subsequent versions, and `session.json`. Previous code is never edited in place.
Completed, paused and interrupted sessions are not replayed on restart. No AC/V,
C/TG, seed loops, automatic Home or automatic unknown-outcome retries are added.
Player learning plans, PRE-A/PRE-B/POST and experience import remain later layers.

`prepare-pick-place` produces a draft for green-cube-to-plate without historical
coordinates. It explicitly records missing semantic localization, validated grasps,
contact/retention/release handling, reviewed scene geometry and independent outcome
checks. It is preparation material, not an executable physical pick/place policy.

### Codex plan integration check (2026-11-17)

Using the existing ChatGPT login and official CLI 0.169.0, `gpt-6-astra` /
`medium` completed both mock generation/execution and seeded-error repair.
Evidence: `runs/codex-plan-validation/9229961505a543a289d689c46051bbed/validation.json`.
Four successful model requests; synthetic results only, no physical actuation.
The repair seed is explicitly labelled as test input. Six Codex-provider checks
and the panel HTTP check passed. The preceding failed run remains recorded; it
identified a mock tuple-payload handling error in plan diagnostics, which was fixed.
