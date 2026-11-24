# ARX official X5-2023 gripper model subset

Source: https://github.com/ARXroboticsX/ARX_Model/tree/a45e8dbb4b1844ae7aef62d16182e5f1724da6d3/X5/X5-2023

Unmodified upstream files: `urdf/X5.urdf` (stored as `X5.urdf`),
`meshes/link7.STL`, `meshes/link8.STL`, and `package.xml`.
The upstream package declares `BSD`; its original package metadata and URDF
exporter header are retained. The pinned repository root has no separate license file.
SHA-256 values used by the adapter are in `adapters/arx_gripper_model.py`.

This is a geometry/provenance subset, not a complete ROS robot package: arm
meshes are intentionally absent. The installed ARX SDK remains the arm FK/IK
model. No new joint driver or collision-checking claim is introduced.
The application-derived grasp frame is documented in `docs/arx-grasp-geometry.md`.
