# X5-2023 payload models

`X5-2023-sdk.urdf` is an unmodified copy of the official ARX SDK model:
https://github.com/ARXroboticsX/ARX5_beta/blob/main/bimanual/models/X5-2023/urdf/X5-2023.urdf

SHA-256: b843943e07562a5abde076b33ea896fd045e8d020da806b0b5a2ea146b504589

It matches the installed SDK file byte for byte. Original exporter metadata
is retained. No separate license grant is asserted here.

The left/right files are application-derived models generated with
`tools/build_arx_payload.py`. Only link6's inertial element is changed.
They preserve the SDK's existing official gripper contribution, and add
one Orbbec Gemini 305 body using manufacturer mass and a nominal box
approximation for COM/inertia. They do not include an unmeasured bracket.

Sources, assumptions, installation transforms and hashes are recorded in
`config/arx-x5-2023-dual-payload.json`; see `docs/arx-payload-model.md`.
These are SDK dynamics assets, not complete ROS visualization packages;
referenced meshes remain in their upstream package.
