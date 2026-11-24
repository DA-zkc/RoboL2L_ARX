# Publication snapshot validation — 2026-11-24

The independent source snapshot passed 164 offline tests covering operator continuation, experiment lifecycle and budget, reset/model recovery, ARX native policy and dual adapter, gripper commissioning, SAM3 confidence, depth projection, and feedback key results.

Command, from this repository (using the workstation's existing Python 3.12 environment):

```bash
PYTHONPATH=.:tests:../ARX5_beta ../RoboL2L/.venv/bin/python -m unittest \
  test_operator_continuation test_experiment_pipeline test_experiment_auto_reset \
  test_experiment_model_recovery test_arx_native_policy test_arx_dual \
  test_arx_gripper_commissioning test_sam3_confidence test_depth_projection \
  test_feedback_key_results
```

The SDK dependency was used for pure software reference semantics; no physical controller was connected by these tests. HTTP tests used local Mock servers. No live experiment was paused, ended, reloaded, or executed for publication.

Python compilation, JavaScript syntax and startup shell syntax checks passed. The package wheel built successfully using the available system setuptools build backend; required ARX owner/adapter, CaP/provider, web and model assets were inspected in the wheel. Runtime deployment is documented as a source checkout because configuration and launch scripts live at the repository root.

The publication index excludes runtime archives, videos, virtual environments, model weights and credentials. Pattern scanning found only explicit dummy credentials in unit tests. This is a bounded scan, not a claim that all possible secrets are detectable. Source snapshot hashes matched the source working tree at verification. Publication-specific README, package metadata, ignore rules, source notices and dependency locks are intentionally separate from that original-file hash manifest.

These results do not constitute physical commissioning or a full historical test-suite pass. Vendor SDK/model source and weights must be installed separately at the locked versions.
