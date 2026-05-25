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
| 轻量结果输出 | 默认只保留 `trajectory.json` |
| 统一流水线入口 | `run_rokae_pipeline.py` 统一编排 plan / contract / GIF / viewer，各阶段按参数开启 |
| MuJoCo 回放 | 按需导出 GIF + 首尾 PNG，可选 realtime viewer，带障碍物时自动渲染 planning cuboid |

## 目录结构

```text
curoboV2_demo/
├── scripts/                          # 工程化规划脚本（主入口）
│   ├── config_utils.py               # 配置加载与校验
│   ├── rokae_asset_utils.py          # 机器人资产路径解析
│   ├── rokae_world_utils.py          # 障碍物 JSON → CuRobo world
│   ├── rokae_motion_gen.py           # CuRobo V2 规划核心封装
│   ├── plan_rokae_motion.py          # 规划子阶段入口（兼容保留）
│   └── run_rokae_pipeline.py         # 统一流水线主入口（推荐）
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
└── third_party/curobo/               # Vendored CuRobo V2
```

## 快速开始

### 环境准备

```bash
# 进入容器
sudo lxc exec zhongji-dev-2204 -- bash

# 激活环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate curoboV2

# 验证
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
# 期望: 2.11.0+cu128 True
```

### 从宿主机直接执行（非交互）

```bash
sudo lxc exec zhongji-dev-2204 -- bash -c "\
  source /home/tanshan/miniconda3/etc/profile.d/conda.sh && \
  conda activate curoboV2 && \
  cd /home/tanshan/rep/curoboV2_demo && \
  <your command>"
```

## 推荐用法

### 一条命令跑完整流程

```bash
cd ~/rep/curoboV2_demo

python scripts/run_rokae_pipeline.py \
  --config resource/config/examples/pose_plan_example.yaml
```

默认行为：

- 执行 `plan -> contract -> realtime viewer`
- 默认只保留 `trajectory.json`
- 默认输出到 `/tmp/curoboV2_demo/...`
- 默认会尝试启动 realtime viewer
- 默认不会保留 GIF/PNG；只有显式开启 `replay_gif` 时才会保留视频图片

如果显式开启 `replay_gif`，成功运行后会额外保留：

- `playback.gif`
- `playback_start.png`
- `playback_end.png`

中间产物（合同、摘要、MJCF、review 文件等）在成功后会自动清理；如果运行失败，会保留现场用于排障。

### 常见参数

```bash
# 规划 + GIF，不开 realtime viewer
python scripts/run_rokae_pipeline.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --replay-gif \
  --no-viewer

# 只规划，最终只保留 trajectory.json
python scripts/run_rokae_pipeline.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --no-export-contract \
  --no-replay-gif \
  --no-viewer

# 从已有规划输出继续跑合同 + GIF + viewer
python scripts/run_rokae_pipeline.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --plan-output-dir /tmp/rokae_full/plan

# 从已有合同直接继续跑 GIF + viewer
python scripts/run_rokae_pipeline.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --contract-json /tmp/rokae_full/contract/playback_contract.json

# 复杂障碍物 + 长路径 + 默认 realtime viewer
python scripts/run_rokae_pipeline.py \
  --config resource/config/examples/joint_plan_complex_viewer.yaml
```

## 分阶段用法（高级/调试）

### 流程一：点到点位姿规划

```bash
cd ~/rep/curoboV2_demo

# 仅执行规划子阶段
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --output-dir /tmp/rokae_demo/pose_plan

# 输出：
#   /tmp/rokae_demo/pose_plan/trajectory.json  — 轨迹数据（内含 metadata）
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

### 流程三补充：复杂障碍物 + 长路径 + 实时窗口

当前仓库提供了一份可直接用于窗口演示的复杂场景配置：

- `resource/config/examples/joint_plan_complex_viewer.yaml`
- `resource/config/examples/obstacles/complex_viewer_test.json`

运行命令：

```bash
cd ~/rep/curoboV2_demo

DISPLAY=:1 MUJOCO_GL=glx python scripts/run_rokae_pipeline.py \
  --config resource/config/examples/joint_plan_complex_viewer.yaml
```

这条示例的特点：

- `joint_target` 模式
- 3 个 cuboid 障碍物
- `speed_scale=0.5`
- 一条较长的关节路径
- 默认会打开 MuJoCo realtime viewer

最近一次验证结果：

- 规划成功
- 路径点数：`101`
- 障碍物统计：`abs_count=3`

窗口效果：

- 显示机械臂、地面和 3 个半透明橙色 box 障碍物
- 机械臂按轨迹逐点实时播放
- 最后一帧会停留一小段时间

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

说明：

- `trajectory.json` 内嵌了规划 metadata 和障碍物摘要，回放合同会从这里恢复障碍物 cuboid。
- `replay_rokae_mujoco.py` 会把这些 cuboid 渲染为 MuJoCo 场景中的半透明 box geom。
- 因此带障碍物的规划回放，GIF 中也会显示对应障碍物。

### 流程四补充：基于已有规划结果重放并渲染障碍物

```bash
cd ~/rep/curoboV2_demo

# 已有带障碍物的规划输出目录
PLAN_DIR=evidence/rokae_bubblify_stress/20260523_000707/baseline/point_to_point__pose1__simple_world__spd0.5/plan

# 1. 从规划输出重新导出合同（会自动带上 obstacle_contract）
python playback/export_rokae_playback_contract.py \
  --plan-output-dir "$PLAN_DIR" \
  --output-dir /tmp/rokae_full/contract_with_obstacles

# 2. 重放并渲染障碍物
export MUJOCO_GL=egl
python playback/replay_rokae_mujoco.py \
  --contract-json /tmp/rokae_full/contract_with_obstacles/playback_contract.json \
  --output-dir /tmp/rokae_full/playback_with_obstacles \
  --render-every 4
```

产物示例：

- `/tmp/rokae_full/playback_with_obstacles/playback.gif`
- `/tmp/rokae_full/playback_with_obstacles/playback_start.png`
- `/tmp/rokae_full/playback_with_obstacles/playback_end.png`

### 流程五：一键规划 + 回放（历史辅助入口）

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

### 流程六：规划子阶段命令行覆盖参数

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

更完整的手工调球步骤见：

- `doc/bubblify_workflow.md`

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

# 统一流水线配置
pipeline:
  run_plan: true
  export_contract: true
  replay_gif: false
  realtime_viewer: true
  render_every: 4
  playback_speed: 1.0
  final_hold_s: 1.0
  # resume_from_plan_output_dir: /path/to/existing/plan
  # resume_from_contract_json: /path/to/existing/playback_contract.json

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

### trajectory.json

轨迹数据，可直接被 MuJoCo 回放链路消费。默认成功后只保留这个文件。

```json
{
  "joint_names": ["XMS5-R800-W4G3B4C_joint_1", "..."],
  "waypoints": [[0.1, 0.2, ...], ...],
  "sample_period_s": 0.025,
  "metadata": {
    "mode": "point_to_point",
    "success": true,
    "status": "success",
    "world": {
      "abs_json": null,
      "rel_json": null,
      "summary": {"abs_count": 0, "rel_count": 0, "total_count": 0},
      "obstacle_names": []
    }
  }
}
```

### playback.gif / playback_start.png / playback_end.png

只有显式开启 `export_contract` 和 `replay_gif` 时才会保留这 3 个文件：

```text
playback.gif         # 动图回放
playback_start.png   # 起始帧
playback_end.png     # 结束帧
```

统一入口在成功时会自动清理中间文件，因此默认不会长期保留：

- `playback_contract.json`
- `review_summary.json`
- `run_summary.json`
- `playback_summary.json`
- `rokae_stage1_playback.xml`
- realtime viewer 摘要文件

### realtime viewer

统一入口默认会尝试启动 MuJoCo realtime viewer。

它不是 Isaac Gym/Isaac Sim 那种训练仿真界面，而是一个轨迹回放窗口：

- 显示机械臂当前姿态
- 显示地面
- 如果轨迹 metadata 中带障碍物信息，也会显示半透明 cuboid 障碍物
- 按 `playback_speed` 控制播放速度
- 按 `final_hold_s` 控制末尾停留时间

如果环境没有可用显示，或 viewer 初始化失败：

- 主流程不会因为 viewer 失败而直接报废
- 可以改用 `--no-viewer`
- 或显式开启 `--replay-gif` 保留离屏视频图片
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

### 统一入口命令行参数一览

| 参数 | 说明 |
|------|------|
| `--config` | 输入配置 YAML 路径 |
| `--output-root` | 覆盖统一输出根目录 |
| `--plan/--no-plan` | 是否执行规划阶段 |
| `--export-contract/--no-export-contract` | 是否导出回放合同 |
| `--replay-gif/--no-replay-gif` | 是否执行离屏回放并导出 GIF |
| `--viewer/--no-viewer` | 是否尝试启动 realtime viewer |
| `--plan-output-dir` | 从已有规划输出继续后续阶段 |
| `--contract-json` | 从已有回放合同继续回放/viewer |
| `--render-every` | 离屏回放渲染步长 |
| `--playback-speed` | realtime viewer 播放速度倍率 |
| `--final-hold-s` | realtime viewer 最后一帧停留秒数 |

### 规划子阶段命令行参数一览

| 参数 | 说明 |
|------|------|
| `--config` | 输入配置 YAML 路径 |
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
- 主推荐入口：`scripts/run_rokae_pipeline.py`
- `plan_rokae_motion.py`、`export_rokae_playback_contract.py`、`replay_rokae_mujoco.py` 保留为分阶段调试入口
- CuRobo V2 默认不编译 pybind CUDA 扩展；如需启用，设置 `CUROBO_USE_PYBIND=1`
- 四元数顺序：配置文件使用 `[qx, qy, qz, qw]`（xyzw），CuRobo 内部使用 `[qw, qx, qy, qz]`（wxyz），脚本自动转换
- MuJoCo 回放只会渲染轨迹 metadata 里记录过的 cuboid 障碍物；如果没有障碍物信息，回放场景中只会显示机器人和地面
- `demo_scripts/` 保留为最小验证样例，工程化任务请使用 `scripts/`
- `ROKAE_migration_notes.md` 保留为迁移记录，不作为主使用文档
