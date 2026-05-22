# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在本仓库中工作时提供指引。

## 项目介绍

`curoboV2_demo` 是面向 ROKAE SR5 机械臂的离线 CuRobo V2 规划工具，具备工程化输入输出、障碍物场景建模和 MuJoCo 回放能力。

**核心特性：**
- 使用 vendored 的 `third_party/curobo` (NVIDIA CuRobo V2)
- 支持多种规划模式：`point_to_point` / `joint_target` / `approach` / `grasp`
- 障碍物场景建模（绝对/相对障碍物 JSON → CuRobo cuboid world）
- 速度缩放（`speed_scale`）和方向约束（`hold_vec_weight`）
- 统一输出：`summary.json` + `trajectory.json` + `world_summary.json`
- MuJoCo 离屏渲染和实时可视化

## 环境启动

本项目运行在 LXD 容器 `zhongji-dev-2204` 中（Ubuntu 22.04），与主项目共享 GPU 和 CUDA，但 Python 环境完全隔离。

### 1. 从宿主机进入容器

```bash
sudo lxc start zhongji-dev-2204
sudo lxc exec zhongji-dev-2204 -- bash
```

### 2. 激活 conda 环境

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate curoboV2

# 验证环境
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
# 期望输出: 2.11.0+cu128 True
```

### 3. 运行规划

```bash
cd ~/rep/curoboV2_demo

# 位姿规划
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --output-dir /tmp/rokae_demo/pose_plan

# 关节目标规划
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/joint_plan_example.yaml \
  --output-dir /tmp/rokae_demo/joint_plan
```

### 4. 完整链路：规划 → 合同 → MuJoCo 回放

```bash
cd ~/rep/curoboV2_demo

# 规划
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --output-dir /tmp/rokae_full/plan

# 构建回放合同
python playback/export_rokae_playback_contract.py \
  --plan-output-dir /tmp/rokae_full/plan \
  --output-dir /tmp/rokae_full/contract

# MuJoCo 离屏回放
export MUJOCO_GL=egl
python playback/replay_rokae_mujoco.py \
  --contract-json /tmp/rokae_full/contract/playback_contract.json \
  --output-dir /tmp/rokae_full/playback \
  --render-every 4

# 一键方式（从规划输出直接回放）
python playback/run_rokae_demo.py \
  --plan-output-dir /tmp/rokae_full/plan \
  --output-root /tmp/rokae_full \
  --no-viewer --render-every 4
```

### 5. 非交互方式（从宿主机直接执行）

```bash
sudo lxc exec zhongji-dev-2204 -- bash -c "\
  source /home/tanshan/miniforge3/etc/profile.d/conda.sh && \
  conda activate curoboV2 && \
  cd /home/tanshan/rep/curoboV2_demo && \
  python scripts/plan_rokae_motion.py \
    --config resource/config/examples/pose_plan_example.yaml \
    --output-dir /tmp/rokae_demo/pose_plan"
```

## 项目结构

```text
curoboV2_demo/
├── scripts/                          # 工程化规划脚本（主入口）
│   ├── config_utils.py               # 配置加载与校验
│   ├── rokae_asset_utils.py          # 机器人资产路径解析
│   ├── rokae_world_utils.py          # 障碍物 JSON → CuRobo world
│   ├── rokae_motion_gen.py           # CuRobo V2 规划核心封装
│   └── plan_rokae_motion.py          # 通用离线规划入口
├── demo_scripts/                     # 最小演示样例（保留）
├── playback/                         # 轨迹导出与 MuJoCo 回放
│   ├── export_rokae_playback_contract.py
│   ├── replay_rokae_mujoco.py
│   └── run_rokae_demo.py
├── resource/config/examples/         # 示例配置文件
│   ├── pose_plan_example.yaml
│   ├── joint_plan_example.yaml
│   ├── grasp_plan_example.yaml
│   └── obstacles/                    # 示例障碍物数据
├── robot_assets/ROKAE/               # 机器人资产包
├── doc/plan/                         # 升级计划文档
├── evidence/                         # 运行产物存储
└── third_party/curobo/               # Vendored CuRobo V2
```

### 核心模块职责

| 模块 | 职责 |
|------|------|
| `scripts/config_utils.py` | 配置加载、校验、路径解析 |
| `scripts/rokae_asset_utils.py` | 机器人资产路径和配置解析 |
| `scripts/rokae_world_utils.py` | 障碍物 JSON 加载、坐标变换、world dict 构建 |
| `scripts/rokae_motion_gen.py` | CuRobo V2 MotionPlanner 封装（plan_single/plan_single_js/plan_grasp_single） |
| `scripts/plan_rokae_motion.py` | 通用离线规划入口（CLI + YAML 配置） |
| `playback/export_rokae_playback_contract.py` | 规划输出 → MuJoCo 回放合同 |
| `playback/replay_rokae_mujoco.py` | 合同 → MJCF → 离屏渲染 |
| `playback/run_rokae_demo.py` | 一键编排（规划 → 合同 → 回放） |

### 导入模式

本项目没有 `setup.py` 或包安装。所有脚本通过 `Path(__file__).resolve().parents[1]` 计算 `WORKSPACE_ROOT`，并将 `third_party/curobo` 注入 `sys.path`。

### Vendored CuRobo V2

`third_party/curobo/` 是 NVIDIA CuRobo V2 的 vendored 副本（Apache-2.0）。关键模块：`motion_planner.py`、`types.py`、`_src/cost/tool_pose_criteria.py`。如需启用 pybind CUDA 扩展，设置环境变量 `CUROBO_USE_PYBIND=1`。

## 重要注意事项

- **URDF 统一路径**：`robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf`
- **conda 环境隔离**：`curoboV2`（CuRobo V2）与 `zhongji`（CuRobo V1）互不兼容，切勿混用
- **四元数顺序**：配置文件使用 `[qx, qy, qz, qw]`（xyzw），CuRobo 内部使用 `[qw, qx, qy, qz]`（wxyz），脚本自动转换
- **CuRobo V2 API 差异**：V2 使用 `MotionPlanner`（非 V1 的 `MotionGen`），`ToolPoseCriteria`（非 V1 的 `hold_vec_weight`），实施前必须检索 `third_party/curobo/` 源码确认
- **speed_scale**：通过 robot config 的 `cspace.velocity_scale` 实现，必须在规划器构造时设置
- **grasp 模式**：线性运动约束对某些构型可能难以满足，approach 成功但 grasp 步骤可能失败
- 容器共享宿主机的 NVIDIA GPU 驱动和 CUDA 12.8 运行时
- `ROKAE_migration_notes.md` 记录历史迁移决策，非当前使用文档
