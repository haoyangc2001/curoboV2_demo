# curoboV2_demo

这是一个面向 CuRobo V2 阶段一验证的独立演示工作区，主要用于完成以下事情：

- 在不依赖主项目运行时树的前提下，单独执行位姿规划。
- 为大花复合末端和官方 Franka 生成可复现的规划摘要。
- 把规划结果导出为 MuJoCo 可直接回放的关节轨迹合同。
- 通过离屏渲染和可选实时 viewer 验证规划到回放的完整链路。

当前仓库更像“阶段一实验与验收工作区”，而不是通用业务工程。目录里同时保留了：

- 自有演示脚本
- 工作区内复制后的机器人资产
- 阶段性验证证据
- vendored 的 `third_party/curobo`

## 1. 项目定位

本项目聚焦两个目标：

1. 官方 Franka 最小规划与回放基线。
2. 大花复合末端/ROKAE 机器人在阶段一工作区中的独立规划与回放链路。

项目强调“工作区自包含”：

- 规划脚本优先读取当前仓库内的 `third_party/curobo/`
- 机器人资产优先读取 `robot_assets/`
- 运行结果与验收材料统一落到 `evidence/`

## 2. 目录结构说明

下面只列项目自有、与理解主流程强相关的目录。

```text
curoboV2_demo/
├─ README.md
├─ ROKAE_migration_notes.md
├─ demo_scripts/
├─ playback/
├─ robot_assets/
│  ├─ ROKAE/
│  └─ dahuafuhe/
├─ evidence/
├─ third_party/
│  └─ curobo/
└─ XMS5-R800-W4G3B4C_description/
```

### 根目录文件

- `README.md`
  当前中文总览文档，说明项目定位、结构、脚本职责和典型流程。
- `ROKAE_migration_notes.md`
  记录 ROKAE 相关迁移背景、阶段说明和过程性笔记。

### `demo_scripts/`

规划侧与资产侧脚本目录，负责“准备资产 + 执行规划 + 校验资产/规划稳定性”。

- `dahuafuhe_asset_utils.py`
  统一管理当前工作区资产路径，提供 YAML 读取、路径解析、机器人配置归一化等基础函数。
- `materialize_dahuafuhe_assets.py`
  从源项目复制阶段一最小资产到工作区，并改写 URDF 网格路径、生成适配后的机器人配置和 manifest。
- `verify_dahuafuhe_assets.py`
  校验复制后的资产包是否完整、自洽且可被 CuRobo 加载。
- `demo_plan_pose.py`
  官方 Franka 最小位姿规划示例。
- `inspect_official_pose_baseline.py`
  提取官方 Franka 示例的 API 基线，用于迁移对照。
- `demo_plan_pose_dahuafuhe.py`
  大花复合末端/ROKAE 机器人阶段一最小位姿规划示例。
- `review_dahuafuhe_pose_demo.py`
  多次重复运行大花复合末端规划，用于稳定性复核。

### `playback/`

回放侧脚本目录，负责“导出规划合同 + 生成 MuJoCo 模型 + 回放 + 批量复核”。

- `export_mujoco_playback_contract.py`
  基于官方 Franka 真实规划结果导出回放合同。
- `replay_official_franka_mujoco.py`
  把官方 Franka URDF 转成最小 MJCF，并按合同回放、离屏渲染。
- `run_official_franka_demo.py`
  串联官方 Franka 合同导出、离屏回放和可选实时 viewer。
- `export_dahuafuhe_playback_contract.py`
  基于大花复合末端真实规划结果导出回放合同。
- `replay_dahuafuhe_mujoco.py`
  把大花复合末端 URDF 转成最小 MJCF，并按合同回放、离屏渲染。
- `run_dahuafuhe_demo.py`
  串联大花复合末端规划、合同导出、离屏回放和可选实时 viewer。
- `review_dahuafuhe_mujoco_playback.py`
  多次重复运行大花复合末端完整规划与回放闭环，用于阶段复核。

### `robot_assets/`

工作区本地机器人资产目录，供规划与回放脚本直接读取。

- `robot_assets/ROKAE/`
  当前启用的 ROKAE 资产包。
  典型内容包括：
  - `start.launch.yaml`
  - `robot/*.yml`
  - `robot/curobo/*.urdf`
  - `robot/curobo/meshes/`
  - `robot/spheres/*.yml`
  - `bundle_manifest.json`
- `robot_assets/dahuafuhe/`
  较早期的大花复合末端资产位置，保留作历史参照和兼容性比对。

### `evidence/`

阶段性验证产物目录，保存：

- `summary.json`
- `review_summary.json`
- `playback_contract.json`
- `playback.gif`
- 首尾帧截图
- 过程日志

这是项目的“验收记录区”，不是源码目录。

### `third_party/curobo/`

仓库内 vendored 的 CuRobo 运行依赖。项目脚本会优先把这里加入 `sys.path`，以保证当前工作区可独立运行。

说明：

- 该目录属于第三方代码，不建议在本次注释整理中直接修改。
- 阅读本项目主流程时，只需知道脚本依赖它提供规划器、类型定义和配置内容。

### `XMS5-R800-W4G3B4C_description/`

ROKAE 机器人描述包，包含 URDF、mesh、launch 等描述文件，主要用于资产迁移或对照参考。

## 3. 主流程结构

项目可以理解成三层流水线：

### 第一层：资产准备

由 `demo_scripts/materialize_dahuafuhe_assets.py` 和 `demo_scripts/verify_dahuafuhe_assets.py` 负责。

目标：

- 把源项目里的最小必要资产复制到当前工作区
- 把 URDF 网格路径改成工作区内可解析路径
- 把机器人配置改成适合 CuRobo 独立加载的格式
- 验证所有路径、网格、关节顺序和碰撞球配置是否一致

### 第二层：位姿规划

由 `demo_scripts/demo_plan_pose.py` 和 `demo_scripts/demo_plan_pose_dahuafuhe.py` 负责。

目标：

- 初始化 `MotionPlanner`
- 生成一个简单可达的目标位姿
- 执行 `plan_pose`
- 导出轨迹 waypoint、规划耗时、末端误差等信息

其中大花复合末端版本不是写死全局目标，而是：

- 先计算当前工具位姿
- 再叠加一个较小的相对平移增量 `goal_delta_xyz`

这种做法更适合阶段一稳定性验证。

### 第三层：合同导出与 MuJoCo 回放

由 `playback/export_*.py`、`playback/replay_*.py`、`playback/run_*.py` 负责。

目标：

- 把规划结果变成“回放合同”
- 明确关节顺序、关节映射、采样周期和轨迹数据格式
- 把 URDF 转成最小 MJCF
- 在 MuJoCo 中按固定 dt 回放
- 生成 GIF、截图和检查摘要

## 4. 关键数据关系

### 规划摘要 `summary.json`

由规划脚本输出，主要描述：

- 是否规划成功
- 目标位姿
- 轨迹 waypoint 数量
- 插值采样周期
- 末端误差
- 轨迹合同原始数据

### 回放合同 `playback_contract.json`

由合同导出脚本输出，主要描述：

- 关节顺序
- 关节名映射规则
- 固定采样周期
- waypoint 序列
- 回放策略
- 验收断言

### 回放摘要 `playback_summary.json`

由 MuJoCo 回放脚本输出，主要描述：

- MuJoCo 模型路径
- 关节到 `qpos` 的映射
- 渲染结果
- 末端位移
- 一致性检查结果

## 5. 当前建议阅读顺序

如果是第一次接手本仓库，建议按这个顺序看：

1. `README.md`
2. `demo_scripts/dahuafuhe_asset_utils.py`
3. `demo_scripts/materialize_dahuafuhe_assets.py`
4. `demo_scripts/verify_dahuafuhe_assets.py`
5. `demo_scripts/demo_plan_pose_dahuafuhe.py`
6. `playback/export_dahuafuhe_playback_contract.py`
7. `playback/replay_dahuafuhe_mujoco.py`
8. `playback/run_dahuafuhe_demo.py`

如果是看官方基线，则读：

1. `demo_scripts/demo_plan_pose.py`
2. `demo_scripts/inspect_official_pose_baseline.py`
3. `playback/export_mujoco_playback_contract.py`
4. `playback/replay_official_franka_mujoco.py`
5. `playback/run_official_franka_demo.py`

## 6. 运行环境约束

项目依赖的运行环境特征如下：

- 需要 Python 环境中已安装 `torch`
- 需要 `mujoco`
- 需要 `imageio`
- 需要 `pyyaml`
- 建议使用项目历史说明中提到的 Conda 环境

项目设计目标是：

- 不依赖主项目安装树直接运行
- 但仍依赖本地 Python/CUDA/MuJoCo 环境完整可用

## 7. 本次文档与注释整理范围

本次整理主要覆盖：

- `demo_scripts/` 下的项目自有 Python 脚本
- `playback/` 下的项目自有 Python 脚本
- 根 `README.md`

整理方式：

- 每个可注释源码文件增加“文件用途”的模块级说明
- 每个函数补充“作用 / 输入 / 输出”的函数级说明
- 在 `README.md` 中同步项目结构与主流程解释

未直接修改的目录：

- `third_party/`：第三方代码
- `evidence/`：运行产物
- `__pycache__/`：缓存产物
- 大量 mesh / STL / DAE / OBJ 等二进制或模型文件

## 8. 常用入口

常用脚本按用途分组如下：

- 资产生成：`demo_scripts/materialize_dahuafuhe_assets.py`
- 资产校验：`demo_scripts/verify_dahuafuhe_assets.py`
- 官方规划：`demo_scripts/demo_plan_pose.py`
- 大花复合末端规划：`demo_scripts/demo_plan_pose_dahuafuhe.py`
- 官方合同导出：`playback/export_mujoco_playback_contract.py`
- 大花复合末端合同导出：`playback/export_dahuafuhe_playback_contract.py`
- 官方一键回放：`playback/run_official_franka_demo.py`
- 大花复合末端一键回放：`playback/run_dahuafuhe_demo.py`

## 9. 一句话总结

这个仓库的核心不是“做一个长期产品工程”，而是“把阶段一规划与 MuJoCo 回放链路做成可独立运行、可反复验证、可沉淀证据的演示工作区”。
