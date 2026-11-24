# RoboL2L_ARX development

This repository is the ARX physical deployment source snapshot, not the workstation's live data directory.

- Read README.md and the relevant docs before changing runtime behavior.
- All actuation belongs to the installed StationRuntime and resident ARX owner. Do not introduce another device controller.
- Hardware operation, shutdown and owner maintenance require current operator authorization. Software tests are not physical acceptance.
- Keep the explicit eight-program experiment limit. Treat an operator's Reset and write next version action as a continuation decision while retaining evidence and native fault/unknown-outcome handling.
- Do not add private numerical hardware limits without an explicit user requirement or established SDK basis.
- Preserve upstream notices and hashes; keep SDK/model dependencies separate from runtime receipts and secrets.
- Use Mock/software tests by default. Do not run check_* hardware probes as ordinary unit tests.
- Do not publish .env, model keys, runtime archives, calibration image captures, videos, weights or virtual environments.
