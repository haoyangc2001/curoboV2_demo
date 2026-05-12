# CuRobo2 Demo Workspace

This standalone workspace is reserved for CuRobo V2 stage-1 work only.

## Location
`/home/jetson/tashan/curobo2_demo_ws`

The workspace is intentionally a sibling of `tashan_robot/`, not a nested directory inside it.

## Minimal Layout
- `src/`: isolated demo source
- `build/`: local build output
- `install/`: local install output
- `log/`: local build and runtime logs
- `demo_scripts/`: minimal pose-planning scripts
- `robot_assets/`: copied or adapted robot assets
- `playback/`: direct MuJoCo playback scripts
- `third_party/`: vendored runtime dependencies needed by the demo workspace
- `evidence/`: step-level verification evidence

## Rules
- Use Conda environment `tashan_danxia_py310`.
- Keep this workspace independent from `tashan_robot/install`.
- Do not place ROS service compatibility or main-project integration code here during stage 1.

## Runtime Independence
The stage-1 demo scripts now vendor the required CuRobo source tree inside:

- `third_party/curobo/`

The current demo flow no longer needs to read `tashan_robot/readCaohy/third_party/curobo` at runtime.

Remaining non-workspace dependency:

- the Python/Conda environment still needs `torch`, `mujoco`, `matplotlib`, and related packages installed
# curoboV2_demo
