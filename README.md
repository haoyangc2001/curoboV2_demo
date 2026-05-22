# curoboV2_demo

面向 ROKAE SR5 机械臂的离线 CuRobo V2 规划工具，具备工程化输入输出、障碍物场景建模和 MuJoCo 回放能力。

## 项目定位

`curoboV2_demo` 从最初的最小演示升级为一个**离线脚本版的 ROKAE CuRobo 规划工具**，在底层能力上对齐 `tashan_robot/dahuafuhe` 的 CuRobo 规划层，同时保持：

- 脚本执行，不依赖 ROS
- 配置文件驱动，不硬编码参数
- 文件输出，便于下游消费

### 核心能力

| 能力 | 说明 |
|------|------|
| 多种规划模式 | `point_to_point` / `joint_target` / `approach` / `grasp` |
| 障碍物场景 | 支持绝对/相对障碍物 JSON，构建 CuRobo cuboid world |
| 速度缩放 | `speed_scale` 参数控制轨迹速度 (0, 2.0] |
| 方向约束 | `hold_vec_weight` 控制末端方向保持 [x, y, z] |
| 统一输出 | `summary.json` + `trajectory.json` + `world_summary.json` |
| MuJoCo 回放 | 离屏渲染 GIF + 可选实时 viewer |

## 目录结构

```text
curoboV2_demo/
├── scripts/                          # 工程化规划脚本（主入口）
│   ├── config_utils.py               # 配置加载与校验
│   ├── rokae_asset_utils.py          # 机器人资产路径解析
│   ├── rokae_world_utils.py          # 障碍物 JSON → CuRobo world
│   ├── rokae_motion_gen.py           # CuRobo V2 规划核心封装
│   └── plan_rokae_motion.py          # 通用离线规划入口
│
├── demo_scripts/                     # 最小演示样例（保留）
│   ├── rokae_asset_utils.py
│   ├── demo_plan_pose_rokae.py
│   └── verify_rokae_assets.py
│
├── playback/                         # 轨迹导出与 MuJoCo 回放
│   ├── export_rokae_playback_contract.py
│   ├── replay_rokae_mujoco.py
│   └── run_rokae_demo.py
│
├── resource/config/examples/         # 示例配置文件
│   ├── pose_plan_example.yaml
│   ├── joint_plan_example.yaml
│   ├── grasp_plan_example.yaml
│   └── obstacles/                    # 示例障碍物数据
│       ├── abs.autosave.json
│       └── rel.autosave.json
│
├── robot_assets/ROKAE/               # 机器人资产包
│   ├── robot/xms5_r800_w4g3b4c_dahuafuhe.yml
│   ├── robot/curobo/ROKAE_SR5_0.9C.urdf
│   └── robot/curobo/meshes/
│
├── doc/plan/                         # 升级计划文档
├── evidence/                         # 运行产物存储
└── third_party/curobo/               # Vendored CuRobo V2
```

## 快速开始

### 环境准备

```bash
# 进入容器
sudo lxc exec zhongji-dev-2204 -- bash

# 激活环境
source ~/miniforge3/etc/profile.d/conda.sh
conda activate curoboV2

# 验证
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
# 期望: 2.11.0+cu128 True
```

### 从宿主机直接执行（非交互）

```bash
sudo lxc exec zhongji-dev-2204 -- bash -c "\
  source /home/tanshan/miniforge3/etc/profile.d/conda.sh && \
  conda activate curoboV2 && \
  cd /home/tanshan/rep/curoboV2_demo && \
  <your command>"
```

## 完整执行流程

### 流程一：点到点位姿规划

```bash
cd ~/rep/curoboV2_demo

# 1. 位姿规划
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --output-dir /tmp/rokae_demo/pose_plan

# 输出：
#   /tmp/rokae_demo/pose_plan/summary.json    — 规划摘要
#   /tmp/rokae_demo/pose_plan/trajectory.json  — 轨迹数据
```

### 流程二：关节目标规划

```bash
# 2. 关节目标规划
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/joint_plan_example.yaml \
  --output-dir /tmp/rokae_demo/joint_plan
```

### 流程三：带障碍物的规划

```bash
# 3. 带障碍物的位姿规划
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --output-dir /tmp/rokae_demo/pose_plan_obs

# 障碍物配置在 YAML 中通过 world.obstacle_json 和 world.obstacle_rel_json 指定
```

### 流程四：规划 → 合同 → MuJoCo 回放（完整链路）

```bash
cd ~/rep/curoboV2_demo

# 1. 执行规划
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --output-dir /tmp/rokae_full/plan

# 2. 构建 MuJoCo 回放合同
python playback/export_rokae_playback_contract.py \
  --plan-output-dir /tmp/rokae_full/plan \
  --output-dir /tmp/rokae_full/contract

# 3. MuJoCo 离屏回放
export MUJOCO_GL=egl
python playback/replay_rokae_mujoco.py \
  --contract-json /tmp/rokae_full/contract/playback_contract.json \
  --output-dir /tmp/rokae_full/playback \
  --render-every 4

# 输出：
#   /tmp/rokae_full/playback/playback.gif     — 回放 GIF
#   /tmp/rokae_full/playback/playback_start.png
#   /tmp/rokae_full/playback/playback_end.png
```

### 流程五：一键规划 + 回放

```bash
cd ~/rep/curoboV2_demo/playback

# 从已有规划输出一键回放
python run_rokae_demo.py \
  --plan-output-dir /tmp/rokae_full/plan \
  --output-root /tmp/rokae_full \
  --no-viewer \
  --render-every 4

# 带实时 viewer（需要 DISPLAY=:1）
python run_rokae_demo.py \
  --plan-output-dir /tmp/rokae_full/plan \
  --output-root /tmp/rokae_full_viewer \
  --render-every 4 \
  --playback-speed 1.0
```

### 流程六：命令行覆盖参数

```bash
# 覆盖目标关节角
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/joint_plan_example.yaml \
  --output-dir /tmp/rokae_demo/custom \
  --goal-jp='-1.0,1.2,0.5,1.0,0.8,0.3'

# 覆盖目标位姿
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --output-dir /tmp/rokae_demo/custom_pose \
  --goal-pose '0.3,-0.3,0.8,0.0,0.707,0.0,0.707'

# 指定速度缩放
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --output-dir /tmp/rokae_demo/slow \
  --speed-scale 0.5

# 指定方向保持约束
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --output-dir /tmp/rokae_demo/hold \
  --hold-vec-weight '0,0,1'
```

## 更新 collision_spheres

当前活动 ROKAE 机器人使用：

- URDF: `robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf`
- collision spheres: `robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml`

推荐使用 Bubblify 为当前活动 URDF 手工调整 collision spheres。工作流如下：

```bash
# 1. 安装 Bubblify（在 curoboV2 环境中）
pip install bubblify

# 2. 打开当前活动 URDF
bubblify \
  --urdf_path robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf \
  --show_collision

# 3. 在浏览器中手工调整 base/link1..link6 的球，导出原始 YAML
# 4. 将导出结果转换为本项目使用的 cuRobo 格式
python scripts/convert_bubblify_spheres.py \
  --input-yaml /path/to/bubblify_export.yml
```

注意事项：

- 只为 `XMS5-R800-W4G3B4C_base` 与 `XMS5-R800-W4G3B4C_link1..link6` 生成球，不为 `tool0` 生成球。
- 活动 robot config 已改为通过路径引用外部 spheres 文件，`robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml` 是唯一真源。
- 转换脚本会校验 link 名、缺失 link、非正 radius，并补齐 metadata。

完成 spheres 更新后，可执行全链路压测：

```bash
python scripts/stress_test_rokae_pipeline.py \
  --baseline-spheres robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml \
  --candidate-spheres /path/to/candidate_spheres.yml
```

## 配置文件格式

规划输入通过 YAML 配置文件定义，所有路径相对于配置文件所在目录。

### 基本结构

```yaml
# 规划模式
mode: point_to_point    # point_to_point | joint_target | approach | grasp

# 机器人配置（留空使用默认）
# robot_config: ../../robot_assets/ROKAE/robot/xms5_r800_w4g3b4c_dahuafuhe.yml

# 起始关节状态（弧度）
start:
  joint_position: [-1.571, 1.571, 0.0, 1.571, 1.571, 0.0]

# 目标位姿 [x, y, z, qx, qy, qz, qw]
goal:
  pose: [0.45, 0.0, 0.55, 0.0, 0.707, 0.0, 0.707]

# 障碍物场景（可选）
# world:
#   obstacle_json: ../../robot_assets/ROKAE/obstacles/abs.autosave.json
#   obstacle_rel_json: ../../robot_assets/ROKAE/obstacles/rel.autosave.json

# 输出目录
output_dir: /tmp/curoboV2_demo/output

# 可选规划参数
# speed_scale: 0.5              # 速度缩放 (0, 2.0]
# hold_vec_weight: [0, 0, 1]    # 方向保持 [x, y, z]，1=保持，0=不约束
# approach_offset: -0.15        # 接近偏移量（米）
# retract_offset: -0.15         # 提升偏移量（米）
```

### 各模式所需字段

| 模式 | 必需字段 | 说明 |
|------|----------|------|
| `point_to_point` | `start.joint_position`, `goal.pose` | 末端位姿规划 |
| `joint_target` | `start.joint_position`, `goal.joint_position` | 关节空间规划 |
| `approach` | `start.joint_position`, `goal.pose` | 接近段规划 |
| `grasp` | `start.joint_position`, `goal.pose` | 完整抓取（approach → grasp → lift） |

## 输出文件说明

### summary.json

规划摘要，包含输入参数、规划状态和性能指标。

```json
{
  "mode": "point_to_point",
  "success": true,
  "status": "success",
  "speed_scale": 1.0,
  "start_joint": [-1.571, 1.571, 0.0, 1.571, 1.571, 0.0],
  "goal_pose": [0.45, 0.0, 0.55, 0.0, 0.707, 0.0, 0.707],
  "solve_time": 0.46,
  "waypoint_count": 81,
  "interpolation_dt": 0.025
}
```

### trajectory.json

轨迹数据，可直接被 MuJoCo 回放链路消费。

```json
{
  "joint_names": ["XMS5-R800-W4G3B4C_joint_1", "..."],
  "waypoints": [[0.1, 0.2, ...], ...],
  "sample_period_s": 0.025
}
```

### world_summary.json（有障碍物时）

障碍物场景摘要。

```json
{
  "abs_json": "path/to/abs.autosave.json",
  "rel_json": "path/to/rel.autosave.json",
  "summary": {"abs_count": 2, "rel_count": 1, "total_count": 3},
  "obstacle_names": ["obstacle_0", "obstacle_1", "obstacle_rel_2"]
}
```

## 高级功能

### 速度缩放

通过 `speed_scale` 控制轨迹速度。值越小轨迹越慢、路径点越多。

```bash
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --output-dir /tmp/demo_slow \
  --speed-scale 0.5
# speed_scale=0.5 → 221 路径点（原速 81 点）
```

### 方向保持约束

通过 `hold_vec_weight` 控制末端方向保持强度。

```bash
# 保持 z 轴方向（roll/pitch 不约束）
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --output-dir /tmp/demo_hold \
  --hold-vec-weight '0,0,1'
```

- `[0, 0, 0]`：无方向约束
- `[1, 1, 1]`：完全保持初始方向
- `[0, 0, 1]`：只保持 z 轴方向

### 抓取规划

`grasp` 模式执行三段规划：approach（接近）→ grasp（线性抓取）→ lift（提升）。

```bash
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/grasp_plan_example.yaml \
  --output-dir /tmp/demo_grasp
```

> 注意：grasp 模式的线性运动约束对某些机器人构型可能难以满足。如果 grasp 步骤失败但 approach 成功，仍会输出接近段轨迹。

### 命令行参数一览

| 参数 | 说明 |
|------|------|
| `--config` | 输入配置 YAML 路径（必需） |
| `--mode` | 覆盖规划模式 |
| `--start-jp` | 覆盖起始关节角（逗号分隔） |
| `--goal-pose` | 覆盖目标位姿（逗号分隔，x,y,z,qx,qy,qz,qw） |
| `--goal-jp` | 覆盖目标关节角（逗号分隔） |
| `--output-dir` | 覆盖输出目录 |
| `--speed-scale` | 速度缩放 (0, 2.0] |
| `--hold-vec-weight` | 方向保持权重（逗号分隔，x,y,z） |
| `--approach-offset` | 接近偏移量（米） |
| `--approach-axis` | 接近轴（x/y/z，默认 z） |

## 运行依赖

`curoboV2` conda 环境已预装以下依赖：

| 包 | 版本 | 说明 |
|---|---|---|
| `torch` | 2.11.0+cu128 | GPU 加速，CUDA 12.8 |
| `cuda-core` | 1.0.1 | CuRobo V2 后端必需 |
| `cuda-bindings` | 12.9.4 | CUDA Python 绑定 |
| `mujoco` | 3.8.1 | 离屏回放和可视化 |
| `pyyaml` | - | 配置文件解析 |
| `imageio` | - | 图像/GIF 导出 |

容器共享宿主机的 NVIDIA GPU 驱动和 CUDA 12.8 运行时，无需额外安装。

## 环境隔离说明

| 项目 | conda 环境 | Python | CuRobo 版本 |
|---|---|---|---|
| tashan_robot（主项目） | `zhongji` | 3.10 | V1 |
| curoboV2_demo | `curoboV2` | 3.10 | V2 |

两个环境共享系统级 CUDA 12.8，互不干扰。**切勿混用两个环境。**

## 注意事项

- 统一 URDF：`robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf`
- CuRobo V2 默认不编译 pybind CUDA 扩展；如需启用，设置 `CUROBO_USE_PYBIND=1`
- 四元数顺序：配置文件使用 `[qx, qy, qz, qw]`（xyzw），CuRobo 内部使用 `[qw, qx, qy, qz]`（wxyz），脚本自动转换
- `demo_scripts/` 保留为最小验证样例，工程化任务请使用 `scripts/`
- `ROKAE_migration_notes.md` 保留为迁移记录，不作为主使用文档
