# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在本仓库中工作时提供指引。

## 项目介绍

`curoboV2_demo` 是一个面向 ROKAE SR5 机械臂的独立 CuRobo V2 + MuJoCo 演示仓库。它在不依赖主项目安装树的前提下，完成本地机器人资产加载、位姿规划、轨迹导出和 MuJoCo 回放验证。

**核心特性：**
- 使用 vendored 的 `third_party/curobo` (NVIDIA CuRobo V2)
- 支持 ROKAE SR5 机器人位姿规划和轨迹生成
- 通过 MuJoCo 进行离屏渲染和实时可视化
- 所有运行产物保存在 `evidence/` 目录，便于复查和验证

## 环境启动

本项目运行在 LXD 容器 `zhongji-dev-2204` 中（Ubuntu 22.04），与主项目共享 GPU 和 CUDA，但 Python 环境完全隔离。

### 1. 从宿主机进入容器

```bash
# 确认容器状态
sudo lxc list --format csv -c n,s | grep zhongji

# 进入容器
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

### 3. 运行演示脚本

所有脚本从 `demo_scripts/` 目录运行：

```bash
cd ~/rep/curoboV2_demo/demo_scripts

# 资产检查
python verify_rokae_assets.py

# 位姿规划（输出 summary.json）
python demo_plan_pose_rokae.py --output-dir /tmp/rokae_demo_output

# MuJoCo 回放（完整链路：规划 → 合同 → 回放）
python ../playback/run_rokae_demo.py --output-root /tmp/rokae_demo_run --no-viewer
```

如需实时 MuJoCo 可视化，去掉 `--no-viewer`（需要 `DISPLAY=:1`）。

### 4. 非交互方式（从宿主机直接执行）

```bash
sudo lxc exec zhongji-dev-2204 -- bash -c "\
  source ~/miniforge3/etc/profile.d/conda.sh && \
  conda activate curoboV2 && \
  cd ~/rep/curoboV2_demo/demo_scripts && \
  python demo_plan_pose_rokae.py --output-dir /tmp/rokae_demo_output"
```

## 项目结构

```text
curoboV2_demo/
├── demo_scripts/              # 规划和资产相关脚本
│   ├── rokae_asset_utils.py   # 统一管理 ROKAE 资产路径
│   ├── demo_plan_pose_rokae.py # 核心规划脚本（最小演示）
│   ├── verify_rokae_assets.py # 资产验证工具
│   └── review_rokae_pose_demo.py # 批量复跑验证
├── playback/                  # 轨迹导出和 MuJoCo 回放
│   ├── export_rokae_playback_contract.py # 轨迹 → JSON 合同
│   ├── replay_rokae_mujoco.py # 合同 → MJCF → 离屏渲染
│   ├── run_rokae_demo.py      # 串联完整链路的编排器
│   └── review_rokae_mujoco_playback.py # 批量复跑验证
├── robot_assets/ROKAE/        # 机器人资产包
│   ├── robot/curobo/ROKAE_SR5_0.9C.urdf # 当前激活 URDF
│   ├── robot/curobo/meshes/   # URDF 依赖的 mesh 资源
│   └── robot/xms5_r800_w4g3b4c_dahuafuhe.yml # CuRobo 机器人配置
├── doc/plan/                  # 升级计划文档（重点）
├── evidence/                  # 运行产物存储
├── third_party/curobo/        # Vendored CuRobo V2
└── XMS5-R800-W4G3B4C_description/ # 原始上游机器人描述包
```

### 核心模块职责

| 模块 | 职责 |
|------|------|
| `demo_scripts/rokae_asset_utils.py` | 中央路径注册表 — 定义所有工作区路径和 YAML 加载工具 |
| `demo_scripts/demo_plan_pose_rokae.py` | 核心规划器 — 从 vendored CuRobo 导入 `MotionPlanner`/`MotionPlannerCfg` |
| `playback/export_rokae_playback_contract.py` | 轨迹 → JSON 合同转换器 |
| `playback/replay_rokae_mujoco.py` | 合同 → MJCF → 离屏 MuJoCo 渲染 |

### 导入模式

本项目没有 `setup.py` 或包安装。所有脚本通过 `Path(__file__).resolve().parents[1]` 计算 `WORKSPACE_ROOT`，并将 `third_party/curobo` 注入 `sys.path`。跨目录导入（playback → demo_scripts）也使用 `sys.path` 操作。

### 机器人资产

`robot_assets/ROKAE/` 是当前使用的资产包，包含：
- CuRobo 机器人 YAML 配置（运动学、碰撞球、关节限制）
- URDF 和 mesh 资源
- `start.launch.yaml` 主项目启动配置镜像

`robot_assets/dahuafuhe/` 是备选机器人资产包。

### Vendored CuRobo V2

`third_party/curobo/` 是 NVIDIA CuRobo V2 的 vendored 副本（Apache-2.0）。关键模块：`motion_planner.py`、`kinematics.py`、`types.py`、`config_io.py`、`content/configs/`。如需启用 pybind CUDA 扩展，设置环境变量 `CUROBO_USE_PYBIND=1`。

### Evidence 目录

`evidence/` 按场景存储运行产物。每次运行保存 JSON 摘要、回放合同、复核结果、GIF/截图。这是验证记录，不是源代码。

## 重点：规划能力升级计划

### 背景

当前 `curoboV2_demo` 具备最小可运行的 ROKAE 规划与 MuJoCo 回放链路，但与 `tashan_robot` 中 `dahuafuhe` 项目的工程化 CuRobo 规划仍有明显差距。

**当前规划特点：**
- 使用 `MotionPlanner`，`scene_model=None`
- 起点取默认关节状态，目标为相对当前 `tool0` 位姿的小幅平移
- 仅考虑机器人自身运动学和自碰撞，不考虑外部障碍物
- 输出 `summary.json`

**目标：** 将本项目升级为"离线脚本版的、具备工程化输入输出和障碍物场景能力的 ROKAE CuRobo 规划工具"，在底层能力上向 `dahuafuhe` 对齐，但保持脚本运行方式。

详细计划见 `doc/plan/rokae_curobo_planning_upgrade_plan.md`。

### 升级计划概览

#### Step 1: 明确离线规划输入接口
- 定义统一规划输入格式（YAML/JSON）
- 支持多种规划模式：`point_to_point | joint_target | approach | grasp | level_carry`
- 定义起始状态、目标输入、场景输入和可选规划参数
- **验收标准：** 可以不改 Python 源码，仅通过配置切换输入

#### Step 2: 新增障碍物 world 工具模块
- 新增 `demo_scripts/rokae_world_utils.py`
- 复用 `dahuafuhe` 的障碍物文件格式（`abs.autosave.json`、`rel.autosave.json`）
- 实现绝对/相对障碍物读取和 CuRobo world dict 生成
- **验收标准：** 给定 `abs/rel` JSON 后，能独立产出 CuRobo 可用的 world dict

#### Step 3: 新增离线版 CuRobo 核心封装
- 新增 `demo_scripts/rokae_motion_gen.py`
- 用 `MotionGen` 作为底层规划器
- 提供接口：`plan_single`、`plan_single_js`、`fk_single`、`update_world_from_dict`
- **验收标准：** 在不依赖 ROS 的情况下，可完成与 `dahuafuhe` 类似的单次 pose/joint 规划

#### Step 4: 重构脚本入口为通用规划入口
- 新增 `demo_scripts/plan_rokae_motion.py`
- 支持 `--config path/to/input.yaml` 和命令行覆盖
- 根据 `mode` 分发到不同规划逻辑
- **验收标准：** 单入口即可完成点到点 pose / joint 规划

#### Step 5: 统一输出结构
- 输出 `summary.json`、`trajectory.json`、`world_summary.json`
- 为后续 MuJoCo 回放提供标准化输入
- **验收标准：** 后续 MuJoCo 回放不再依赖 demo 专用摘要格式

#### Step 6: 接入 MuJoCo 回放链路
- 调整 `export_rokae_playback_contract.py` 读取新规划输出
- 保留离屏回放和 viewer 能力
- **验收标准：** 新规划入口生成的结果可以直接回放

#### Step 7: 补齐高级能力
- 逐步实现：`hold_vec_weight`、`speed_scale`、`approach_offset`、`grasp`、`level_carry`
- **验收标准：** 本项目可覆盖大部分 `dahuafuhe` CuRobo 规划模式

### 推荐实施顺序

1. 明确输入配置 schema
2. 新增 `rokae_world_utils.py`
3. 新增 `rokae_motion_gen.py`
4. 完成 `plan_single` 与 `plan_single_js`
5. 新增 `plan_rokae_motion.py`
6. 输出统一结果文件
7. 接回 MuJoCo 回放
8. 再补 approach / grasp / level carry

### 建议的文件演进结构

```text
demo_scripts/
├── rokae_asset_utils.py       # 机器人资产路径和配置解析
├── rokae_world_utils.py       # 障碍物读取和 world 构建（新增）
├── rokae_motion_gen.py        # CuRobo 核心封装（新增）
├── plan_rokae_motion.py       # 通用规划入口（新增）
├── review_rokae_motion.py     # 批量复跑和稳定性验证（新增）
├── demo_plan_pose_rokae.py    # 最小演示样例（保留）
└── verify_rokae_assets.py     # 资产验证工具
```

### 风险与注意事项

1. **不要直接把现有 demo 叠成大而全脚本** — `demo_plan_pose_rokae.py` 当前适合作为最小验证样例，不适合作为长期规划主入口
2. **不要一开始就完整复制 ROS 工程结构** — 保持"离线、轻量、单仓运行"的特点
3. **障碍物先统一成 cuboid** — 第一阶段优先复用 `dahuafuhe` 已有的 `abs/rel` JSON 到 `cuboid` 的逻辑
4. **优先保证输入输出稳定，再扩功能** — 只有在输入配置格式和输出结果格式稳定后，后续功能才容易扩展

## 重要注意事项

- CuRobo V2（`curoboV2` 环境）与 CuRobo V1（`zhongji` 环境，tashan_robot 使用）互不兼容，切勿混用
- conda 环境包含 `torch 2.11.0+cu128`、`cuda-core 1.0.1`、`cuda-bindings 12.9.4`、`mujoco`、`pyyaml`、`imageio`
- 容器共享宿主机的 NVIDIA GPU 驱动和 CUDA 12.8 运行时
- `ROKAE_migration_notes.md` 记录历史迁移决策，非当前使用文档
