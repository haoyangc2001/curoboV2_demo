# ROKAE Asset Bundle

This directory contains the active `ROKAE` robot asset bundle used by `curobo2_demo_ws`.

Contents:

- `start.launch.yaml`: active launch config for the ROKAE bundle
- `robot/xms5_r800_w4g3b4c_dahuafuhe.yml`: active CuRobo2 robot config for stage 1
- `robot/curobo/xms5_r800_w4g3b4c_dahuafuhe.urdf`: active URDF with workspace-local mesh paths
- `robot/curobo/meshes/`: copied robot and tool meshes
- `robot/spheres/xms5_r800_w4g3b4c_spherized.yml`: active collision-sphere config
- `bundle_manifest.json`: materialization summary

Notes:

- this bundle is now separated from the historical `robot_assets/dahuafuhe` assets
- `robot_assets/dahuafuhe` should be treated as the previous Yongda/dahuafuhe asset location
- current active ROKAE model is `XMS5-R800-W4G3B4C`
