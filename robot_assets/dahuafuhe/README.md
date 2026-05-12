# Dahuafuhe Stage-1 Asset Bundle

This directory contains the standalone `dahuafuhe` robot asset bundle used by `curobo2_demo_ws` stage 1.

Contents:

- `start.launch.yaml`: copied project reference launch config
- `robot/rokae_cr7_dahuafuhe.yml`: CuRobo2-adapted robot config for stage 1
- `robot/curobo/rokae_cr7_dahuafuhe.urdf`: copied URDF with workspace-local mesh paths
- `robot/curobo/meshes/`: copied robot and tool meshes
- `robot/spheres/rokae_cr7_dahuafuhe_spherized.yml`: copied collision-sphere source
- `bundle_manifest.json`: materialization summary

Important stage-1 rule:

- this bundle is intended only for the standalone demo workspace
- it should not be treated as a ROS integration asset set
- task 7 only guarantees asset loading and path resolution
- planning and MuJoCo playback validation happen in later stage-1 tasks
