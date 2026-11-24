# Sources and notices

This is a source snapshot of the ARX deployment work in https://github.com/Wzz-D/RoboL2L, including local changes recorded by config/source-snapshot.json. A public repository is not itself a new blanket license grant.

- `cap_harness/` is derived from ASPIRE. Its Apache-2.0 license, NOTICE, source commit, original hashes and local patch manifest remain in that directory. `patches/aspire_deployment.patch` records deployment changes.
- `robol2l_deploy/UPSTREAM-LICENSE` retains the related upstream license text.
- ARX gripper geometry and SDK-derived payload URDFs retain their source notices in `robol2l_deploy/models/*/NOTICE.md`; see those notices for the exact scope and source versions.
- Legacy Franka geometry retains `franka_cap/models/LICENSE`.
- The ARX vendor SDK, SAM3 and Contact-GraspNet model source and weights are external dependencies. Their lock files identify upstream repositories and versions. Obtain and use them under their own terms; they are not bundled here.

No additional license is assigned to code whose original license is not established by these notices.
