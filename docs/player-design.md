# Player

The Player uses ASPIRE's public execution tools with a neutral RoboL2L worker prompt. The worker explores, writes programs, debugs on the development seeds, checks the unseen seeds and writes findings and a skill report. New campaigns start with an empty prior-experience library; validation and promotion are not automatically launched. The Player provides:

1. a written record around every program version (PRE-A, PRE-B, POST),
2. two gates that refuse to launch or advance a version unless that record exists,
3. a task reflection written after the task is finished.
4. a concise initial learning roadmap, kept with the initial analysis and revised through the existing records.

Its outputs are raw material for an Assistant Coach and a Coach. The Player does not synthesise strategic or tactical skills itself. This is the sole Player implementation on main; Assistant Coach handoff uses its existing records and prompt-staging interface.

## The loop

```
v00    PRE-A → code → PRE-B → run (all development seeds) → POST
v01 …  PRE-A → code → PRE-B → run (all development seeds) → POST
v07    the eighth and final development version
unseen run (once) → POST
```

Version names use ASPIRE's `v00_initial.py`, `v01_<slug>.py` scheme. New runs start directly at `v00`; every version runs all development seeds and counts toward the eight-version budget. There is no first-seed training phase or byte-identical copy promotion. The worker and coordinator overlays explicitly override the upstream first-seed instruction and failure-mode stopping limit. There is no per-mode version cap: each attempt needs evidence, and unresolved modes stay recorded in `debug/blocked/<mode>.md`. After any version solves every development seed (not necessarily `v00`), one one-variable ablation version is required and at most two may follow; the launcher refuses a third. The POST of the solving version carries an `## Ablation decision` section (`Ablate:` or `Skip:` with the earlier version pairs that already isolated each main choice); `player post` refuses it without, and the unseen launch is refused when neither an ablation nor a skip was recorded. Whether a skip justification is true is not machine-checked. The allowance is a ceiling, not a quota.

Before any `player probe` or `player run`, save the initial analysis and learning roadmap in `task_analysis.md`, using available public evidence and marking unknowns. The roadmap normally has 3–6 connected learning stages, each with an objective, exploration method, expected learning outcome and progression conditions. It describes how to learn, not only which robot actions to perform. Detail the next useful question; direct execution can itself provide information.

Keep that file as the initial snapshot. Record new observations and material roadmap revisions in `v00` PRE-A and subsequent PRE-A/POST, with supporting evidence. Do not rewrite files already cited by hash: AC indexing checks those references. This replaces Stage 0's exploration-before-analysis order without adding a planning command or hard execution gate. Exploration sub-budgets apply only when actually assigned; no new probe quota is introduced. The full wording is in [the Player prompt](../player/prompts/player.md).

| Record | Written | Contains |
|---|---|---|
| PRE-A | holding the task, or after the previous POST; before code | what is known, the approach, material learning-plan revisions, the change from the previous version, the targeted failure mode (`initial` for v00 or one of the seven Stage 1 modes thereafter; `ablation` instead, and only, once a version has solved every development seed). The relation is `first` for v00; later versions compare with the preceding version as `same-class` or `new-class` |
| PRE-B | code final | 1 to 6 risks, each with a prediction, a checkpoint label (or none), a cost (`free`, or `motion` with ticks and a fallback), and what would follow if confirmed. Bound to the program's sha256 |
| run | by `player run` | phase, seeds, exact command, output roots, program hash, then exit status, outcome per seed and the captured `CHECKPOINT` lines |
| POST | after the run | a verdict (`hit`, `miss`, `inconclusive`) with evidence files for every PRE-B risk, what was not predicted, and a root cause (one of the seven modes, or `none`) |

## Gates

`player run` is the version launcher. It builds the official `cap-harness run-batch` command from the benchmark profile; standalone probes use `player probe`.

| Gate | Refuses unless |
|---|---|
| PRE | PRE-A exists; PRE-B exists and its program hash equals the file being launched; static lint passes; the version is the next one in sequence |
| POST | every earlier attempt has a POST that gives one verdict per prediction, cites existing files, and names a root cause consistent with the outcome. Checked when the next PRE-A is written and again at launch |

The sequence is enforced: `v00`, `v01`, … `v07`. Decimal versions are refused by new execution commands. A version that produced results never runs again. A launch that produced no results at all (the simulator never started) may be repeated. The unseen check runs once, on `fix_code.py`, which must equal a version that ran. Its launch gate requires either a complete development sweep with joint `task_success` and `program_ok` on every seed, or eight complete development versions. All development attempts must have complete results, normal harness completion and POST records. Partial, missing, interrupted or pending attempts block unseen. Infrastructure blockers or no meaningful executable program require an incomplete report, not meaningless trials to fill the budget. After a version has solved every development seed, unseen needs an ablation version or a recorded skip, a third version after it is refused (`ABLATION_LIMIT` = 2, inside the eight-version cap), and those versions must target `ablation`. This new-launch check does not reinterpret historical ledgers or change their read-only audits. Independent probes may still default to the first development seed; this does not create a separate training phase.

The gates establish that records exist and bind to the right program and results. They do not judge the thinking. A run started by calling cap-harness directly bypasses them, and `player check` finds it by comparing the official `debug/` outputs with the ledger.

Once unseen has been launched, new development PRE-A, PRE-B and run commands are refused. An earlier nonterminal trajectory cannot be completed after seeing unseen feedback. Historical read-only audits remain unchanged; continue old experiments only with their frozen implementation.

## Lint

`player lint` (and PRE-B and `run`) check the program against the sandbox contract before anything launches: the harness's own validator rules (`global`, `from` imports, classes, `with`, private names, imports other than `math`), builtins outside the whitelist, and `ArmCommand`, `Trajectory` or `SynchronizedTrajectory` built without `embodiment=`. The rules are read from the ASPIRE checkout (`cap_harness.runtime`) when it is importable and fall back to a vendored copy otherwise; the rules' source is recorded.

Run against the saved programs of the 80-task campaign's tasks 1 and 3, the lint flags exactly the defective ones: task 3 v01 and v02 (zero information for fifteen seeds each) and task 1 v00 to v04, and passes every version after the fix. It cannot catch errors that appear only at run time.

## Files in `$TASK_DIR`

| File | Role |
|---|---|
| `player_events.jsonl` | the ledger: append-only, hash-chained, the only source of truth |
| `learning_journal.md` | generated view for people, in time order |
| `attempts.jsonl` | generated view for Assistant Coach and Coach: one object per version, with approach, risks, run, outcome, checkpoints, POST. New records use `schema_version` 2 and the full-development learning protocol |
| `task_reflection.md` | written by the worker last: path, judgments that mattered, mistakes, information that came too late, analysis that transfers |

`player check` verifies the chain, that both views still equal what the ledger renders, that programs were not edited after they ran, that every launched version has a POST, and that no official output exists without a logged launch. With `--final` it also requires `findings.md`, `skill_report.json`, `fix_code.py`, the unseen measurement and a reflection with its five sections. `player summary` prints the counts the reflection needs.

`player check --session-dir DIR` also scans the worker's session logs (Codex `.jsonl`, found recursively under DIR) for shell commands that read forbidden sources: scene files (`.bddl`, `.xml`, `.urdf`, `init_files`, the LIBERO package), ASPIRE harness source (`src/cap_harness`) and upstream skills, docs and examples. Those are problems. A bare mention of a simulator-internal name (`native_env`, `qpos`, ...) is only a note, because audit scripts list such names. The robot program itself cannot reach simulator internals; this scan checks the agent. A clean scan means no matching command was found, not that none occurred: only shell commands are read, and a differently written command can slip past. It is not a substitute for file-permission isolation. Without `--session-dir`, `check --final` prints a note that the scan was not run.

Schema v1 ledgers remain readable with their original rendering, decimal version names and promotion records. They are not migrated or rewritten. Appending to a v1 ledger and mixing schemas within one ledger are refused; use the frozen old implementation to continue an old experiment. New V metrics accept only the schema v2 full-development protocol, count `v00` as learning iteration 1, and reject cross-protocol comparisons rather than reinterpreting historical results.

## Prior experience inputs

`campaign.json.prior_skill_library` defaults to `null`, including when omitted. This means no historical robot skills. An explicit absolute directory opts into its Markdown documents. At preparation, the selected documents are copied to `RUN_ROOT/player-inputs/skills`; `skills-manifest.json` beside that directory freezes the configuration, file hashes and ASPIRE-compatible library hash. A truly empty library has hash `e3b0c44298fc1c14`; its directory contains no instructional README.

The same policy applies to direct Player commands, single-cycle AC and A2A. All mutations validate or prepare the snapshot before proceeding. Guided campaigns copy the baseline's frozen input, never reread the original source or import a prior Player's proposals. Source edits after freezing have no effect; snapshot/configuration changes are refused. Existing ASPIRE skill files are never changed. Historical runs without the manifest remain readable but cannot be resumed or silently assigned the new empty policy.

Public API contracts and runtime capabilities remain available. Worker and coordinator prompts are assembled from this repository's neutral bases and protocol overlays, without the upstream skill-reading instructions, historical strategy addenda or task solutions. This is an input and instruction boundary, not a claim of OS-level access isolation.

`skill_report.json` remains required. Its hash and task identity must match the frozen input. In empty mode, `consulted` and `proposed_edits` are empty; `proposed_new` may record supported findings from this run, but is not automatically promoted. The wrapper retains ASPIRE's snippet validation and permits a new Markdown filename for a new proposal. Reports cannot claim to have consulted files absent from the snapshot.

## Commands

`campaign.json.seed_workers` is a positive integer, default 4. Both development
and unseen use `min(seed_workers, assigned seed count)` independent processes on
the same frozen program. The campaign protocol freezes this setting and guided
Players inherit it. Harness passthrough arguments cannot override workers or the
program/seed/output fields needed to verify the batch. Each launch with results
archives an immutable batch receipt; missing or inconsistent worker metadata
blocks completion. Historical runs without this receipt remain read-only
auditable under their original records. Versions and P/AC phases are sequential.

```
player pre-a   --version V --targets T --based-on FILE... < body
player pre-b   --version V --program code/V_<slug>.py < body
player run     --version V [--program P] [--dry-run] [--timeout-s N] [-- extra cap-harness args]
player post    --version V --root-cause C [--interrupted] < body
player probe   --program code/probe_<slug>.py --question "..." [--seed N]
player check   [--final] [--session-dir DIR]        player summary        player lint --program P
player dispatch worker|verify   player coordinator    player init|progress|validate -- ARGS
player skills-check --skill-library-sha
player skills-check --validate-skill-report PATH
```

Global options (`--aspire-root --run-root --benchmark --suite --task-id --gpu`) go before the subcommand. The staged worker prompt defines a shell function `player` that already carries them.

## Staging a worker

```
python -m player --aspire-root ../ASPIRE --run-root RUN --benchmark libero-pro \
  --suite libero_goal_swap --task-id 3 --gpu 0 dispatch worker
```

This stages `worker-base.md`, `player.md` and `reflection.md`, plus an explicitly supplied note. It freezes the prior-experience selection and binds the resulting prompt to that snapshot. It does not append the upstream worker template or historical dispatcher addenda. Every development version uses all seeds within the eight-version budget; one ablation version (or a recorded skip) and at most two follow the first full solve. `player coordinator` similarly uses a neutral base plus the coordinator overlay, with no automatic skill promotion. A provenance file with source hashes is written next to each staged prompt. The separate verifier role still uses official staging.

## Not covered

- Phase-3 reruns write to a new attempt directory; the Player assumes the standard task directory.
- Single-GPU campaigns, where the coordinator launches episodes for the worker.
- Errors that appear only at run time are not caught by lint. Whether `import math` raises `SystemError` in some programs (task 1 v00) is unreproduced and is not linted.
- No claim is made that the prompts improve results. Evaluation is separate and later.

## Tests

`python -m unittest discover -s tests` runs the Player's tests against a fake `cap-harness`; no simulator is used. The gate flow, refusals, bypass audit, lint rules (including parity with the real ASPIRE runtime when the checkout is present), record parsing, ledger views, the CLI and prompt staging are covered. Assistant Coach handoff tests use the same fake worker; real-runtime lint parity checks require the sibling ASPIRE checkout.
