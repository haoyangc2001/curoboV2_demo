# curoboV2_demo ROKAE CuRobo 规划能力升级方案

## 1. 背景

当前 `curoboV2_demo` 已经具备一条最小可运行的 ROKAE 规划与 MuJoCo 回放链路，核心入口包括：

- `demo_scripts/demo_plan_pose_rokae.py`
- `playback/export_rokae_playback_contract.py`
- `playback/replay_rokae_mujoco.py`
- `playback/run_rokae_demo.py`

现阶段这条链路的定位是“最小验证样例”：

- 使用工作区本地的 ROKAE 机器人资产
- 通过 CuRobo 进行一次末端位姿规划
- 导出轨迹摘要
- 在 MuJoCo 中做离屏或实时回放

但当前规划能力与 `tashan_robot` 中 `dahuafuhe` 项目的工程化 CuRobo 规划仍有明显差距。`dahuafuhe` 中已经形成了完整的 CuRobo 规划层，具备：

- 面向 ROS 服务的统一规划入口
- 当前关节状态输入
- 多种规划模式
- 外部障碍物场景
- 更完整的轨迹输出
- 更贴近实际业务任务的规划约束

本次任务不是把 `curoboV2_demo` 改成 ROS 工程，而是要把它升级成“离线脚本版的 CuRobo 规划工具”，在底层能力和输入输出模型上尽量贴近 `dahuafuhe`，但仍保持：

- 通过脚本执行
- 通过参数或配置文件传入输入
- 通过文件输出规划结果

## 2. 当前现状

### 2.1 `curoboV2_demo` 当前规划特点

当前规划主要由 `demo_scripts/demo_plan_pose_rokae.py` 完成，特点如下：

- 使用 `MotionPlanner`
- `scene_model=None`
- 起点通常取默认关节状态
- 目标为相对当前 `tool0` 位姿的小幅平移
- 输出 `summary.json`
- 规划时仅考虑机器人自身运动学和自碰撞
- 不考虑外部障碍物

因此，当前规划本质上是：

“输入一个起始关节状态和一个目标末端位姿，在无外部场景障碍物的条件下，生成一条满足自碰撞约束的轨迹。”

### 2.2 `tashan_robot/dahuafuhe` 当前规划特点

`tashan_robot` 中的 CuRobo 核心封装位于：

- `src/trajectory_planning/trajectory_planning/curobo_motion/curobo_motion_gen.py`

规划服务入口位于：

- `src/trajectory_planning/trajectory_planning/main.py`

其特点如下：

- 使用 `MotionGen`
- 通过 ROS 服务接收规划请求
- 支持多种规划模式：
  - point-to-point pose
  - joint target
  - approach
  - grasp
  - level carry
- 当前起点来自实时或仿真的 joint state
- 机器人配置通过 `robot_config` 加载
- 外部障碍物通过 `obstacles/abs.autosave.json` 与 `obstacles/rel.autosave.json` 构建
- 障碍物被转换为 CuRobo `cuboid` world
- 输出为 `trajectory_msgs/JointTrajectory`
- 支持速度缩放、保持方向约束、接近/回撤路径等

## 3. 本次任务目标

本次任务的核心目标是：

将 `curoboV2_demo` 升级为一个“非 ROS 的、脚本化的、具备工程化输入输出结构的 ROKAE CuRobo 规划工具”，能力上向 `dahuafuhe` 的 CuRobo 规划层对齐。

明确范围如下。

### 3.1 要达成的目标

- 保持脚本运行方式，不引入 ROS 服务通信
- 支持从参数或配置文件传入规划输入
- 支持显式传入起始关节状态
- 支持显式传入目标末端位姿或目标关节
- 支持加载与 `dahuafuhe` 相同格式的障碍物配置
- 在 CuRobo 中构造外部障碍物 world
- 输出统一的轨迹结果、求解信息和调试摘要
- 为后续 MuJoCo 回放复用同一份规划输出

### 3.2 暂不纳入本阶段的目标

- 不接入 ROS 节点
- 不实现 joint state 订阅
- 不实现真机联动
- 不实现状态机或主流程编排
- 不要求第一阶段就完整覆盖 grasp / level carry 全能力

## 4. 差异分析

### 4.1 当前 demo 与 `dahuafuhe` 的主要差异

1. 规划器层级不同

- 当前 demo：直接使用 `MotionPlanner`
- `dahuafuhe`：封装为 `CuroboMotionGen`，底层使用 `MotionGen`

2. 输入模型不同

- 当前 demo：默认起点 + 内置目标构造
- `dahuafuhe`：显式输入当前 joint、目标 pose/joint、速度、约束参数

3. 场景模型不同

- 当前 demo：没有外部障碍物场景
- `dahuafuhe`：有绝对障碍物和相对障碍物，并构造成 CuRobo world

4. 输出模型不同

- 当前 demo：`summary.json`
- `dahuafuhe`：服务响应里的 `JointTrajectory`，同时保留求解状态和调试信息

5. 能力覆盖不同

- 当前 demo：仅最小 pose plan
- `dahuafuhe`：pose plan、joint plan、approach、grasp、level carry

### 4.2 可以直接复用的部分

- ROKAE 机器人资产读取逻辑
- CuRobo 本地依赖加载方式
- 现有 `summary.json` / playback contract / MuJoCo 回放链路
- 机器人自碰撞球配置

### 4.3 需要新增或重构的部分

- 障碍物 world 构建工具
- 统一的规划输入配置模型
- 独立的 `rokae_motion_gen.py`
- 新的通用规划脚本入口
- 更贴近 `dahuafuhe` 的轨迹结果输出结构

## 5. 总体实施思路

整体实施思路分为两层：

1. 先建立离线版 CuRobo 规划核心

- 不依赖 ROS
- 只做参数输入和脚本执行
- 先打通 pose plan、joint plan、障碍物 world

2. 再逐步补齐高级规划模式

- hold 向量约束
- approach
- grasp
- segmented level carry

这样可以避免一开始把任务做成大重构，优先保证主链路可运行、可验证。

## 6. 分步骤实施计划

## Step 1. 明确离线规划输入接口

### 工作目标

定义 `curoboV2_demo` 的统一规划输入格式，使规划不再依赖脚本内硬编码参数。

### 建议产物

- 一份输入配置文件格式说明
- 一份示例 `yaml/json`

### 计划内容

- 定义规划模式字段：
  - `mode: point_to_point | joint_target | approach | grasp | level_carry`
- 定义机器人配置输入：
  - `robot_config`
- 定义起始状态输入：
  - `start.joint_position`
- 定义目标输入：
  - `goal.pose`
  - `goal.joint_position`
- 定义场景输入：
  - `world.obstacle_json`
  - `world.obstacle_rel_json`
- 定义可选规划参数：
  - `speed_scale`
  - `hold_vec_weight`
  - `approach_offset`
  - `retract_offset`
  - `linear_axis`
  - `segment_length`

### 完成标志

- 可以不改 Python 源码，仅通过配置切换输入

## Step 2. 新增障碍物 world 工具模块

### 工作目标

把 `dahuafuhe` 中障碍物配置的读取和世界构建方式迁移到本仓库。

### 建议新增文件

- `demo_scripts/rokae_world_utils.py`

### 计划内容

- 复用 `dahuafuhe` 的障碍物文件格式：
  - `abs.autosave.json`
  - `rel.autosave.json`
- 实现以下能力：
  - 读取绝对障碍物
  - 读取相对障碍物
  - 根据 `base_pose` 将相对障碍物转换为绝对坐标
  - 生成 `{"cuboid": {...}}` 形式的 CuRobo world dict
- 生成统一输出结构：
  - `boxes`
  - `world_dict`
  - `world_summary`

### 完成标志

- 给定 `abs/rel` JSON 后，能独立产出 CuRobo 可用的 world dict

## Step 3. 新增离线版 CuRobo 核心封装

### 工作目标

新增一个与 `dahuafuhe/CuroboMotionGen` 角色对应的离线核心封装，作为本项目今后的规划内核。

### 建议新增文件

- `demo_scripts/rokae_motion_gen.py`

### 第一阶段建议提供的接口

- `plan_single(start_joint, target_pose, hold_vec_weight=None, speed_scale=None)`
- `plan_single_js(start_joint, goal_joint, speed_scale=None)`
- `fk_single(joint_position)`
- `update_world_from_dict(world_dict)`

### 计划内容

- 用 `MotionGen` 作为底层规划器
- 加载机器人配置时自动解析：
  - `urdf_path`
  - `asset_root_path`
  - `collision_spheres`
- 保留自碰撞诊断能力
- 支持世界障碍物更新
- 输出统一结果：
  - `trajectory_points`
  - `interpolation_dt`
  - `solve_time`
  - `status`
  - `profile`

### 完成标志

- 在不依赖 ROS 的情况下，可完成与 `dahuafuhe` 类似的单次 pose/joint 规划

## Step 4. 重构脚本入口为通用规划入口

### 工作目标

不要继续在 `demo_plan_pose_rokae.py` 上叠加功能，而是建立通用规划入口。

### 建议新增文件

- `demo_scripts/plan_rokae_motion.py`

### 建议保留旧文件的方式

- `demo_plan_pose_rokae.py` 继续保留为最小 demo
- 新文件承担工程化离线规划入口职责

### 计划内容

- 支持 `--config path/to/input.yaml`
- 支持命令行覆盖关键参数
- 根据 `mode` 分发到不同规划逻辑
- 自动加载 world
- 自动加载 robot config
- 输出统一结果文件

### 完成标志

- 单入口即可完成点到点 pose / joint 规划

## Step 5. 统一输出结构

### 工作目标

将当前过于 demo 化的输出整理成更稳定、更适合后续消费的格式。

### 建议输出文件

- `summary.json`
- `trajectory.json`
- `world_summary.json`
- 可选 `playback_contract.json`

### 计划内容

`summary.json` 建议包含：

- 输入配置路径
- 机器人配置路径
- 起始 joint
- 目标 pose 或目标 joint
- world 障碍物统计
- success
- status
- solve_time
- interpolation_dt
- waypoint_count

`trajectory.json` 建议包含：

- `joint_names`
- `waypoints`
- `sample_period_s`

### 完成标志

- 后续 MuJoCo 回放不再依赖 demo 专用摘要格式

## Step 6. 接入 MuJoCo 回放链路

### 工作目标

让新的离线规划输出能直接被现有回放脚本消费。

### 计划内容

- 调整 `export_rokae_playback_contract.py`
- 让回放合同从新规划输出读取轨迹
- 保留当前离屏回放和 viewer 能力

### 完成标志

- 新规划入口生成的结果可以直接回放

## Step 7. 补齐高级能力

### 工作目标

在主链路稳定后，再逐步贴近 `dahuafuhe` 的高级规划能力。

### 建议优先级

1. `hold_vec_weight`
2. `speed_scale`
3. `approach_offset`
4. `grasp`
5. `level_carry`
6. auto prealign

### 完成标志

- 本项目可覆盖大部分 `dahuafuhe` CuRobo 规划模式，但仍以离线脚本方式运行

## 7. 建议的文件演进结构

建议最终把相关脚本组织成如下结构：

```text
demo_scripts/
|-- rokae_asset_utils.py
|-- rokae_world_utils.py
|-- rokae_motion_gen.py
|-- plan_rokae_motion.py
|-- review_rokae_motion.py
|-- demo_plan_pose_rokae.py
`-- verify_rokae_assets.py
```

其中职责建议如下：

- `rokae_asset_utils.py`
  - 机器人资产路径和配置解析
- `rokae_world_utils.py`
  - 障碍物读取和 world 构建
- `rokae_motion_gen.py`
  - CuRobo 核心封装
- `plan_rokae_motion.py`
  - 通用规划入口
- `review_rokae_motion.py`
  - 批量复跑和稳定性验证
- `demo_plan_pose_rokae.py`
  - 最小演示样例

## 8. 第一阶段建议验收标准

建议第一阶段不要追求功能过多，先以以下标准验收：

1. 可以从配置文件读取：

- 起始 joint
- 目标 pose
- 机器人配置
- 障碍物配置

2. 可以正确加载 world：

- 绝对障碍物
- 相对障碍物
- world dict

3. 可以完成两类规划：

- point-to-point pose
- joint-to-joint

4. 可以输出稳定结果：

- `summary.json`
- `trajectory.json`

5. 可以与 MuJoCo 回放链路对接

## 9. 风险与注意事项

### 9.1 不要直接把现有 demo 叠成大而全脚本

`demo_plan_pose_rokae.py` 当前适合作为最小验证样例，不适合作为长期规划主入口。

### 9.2 不要一开始就完整复制 ROS 工程结构

本项目应保持“离线、轻量、单仓运行”的特点，只对齐底层规划能力，不对齐 ROS 通信方式。

### 9.3 障碍物先统一成 cuboid

第一阶段优先复用 `dahuafuhe` 已有 `abs/rel` JSON 到 `cuboid` 的逻辑，不要同时引入 mesh 场景建模。

### 9.4 优先保证输入输出稳定，再扩功能

只有在输入配置格式和输出结果格式稳定后，后续 grasp、level carry、回放、批量评审才容易扩展。

## 10. 推荐实施顺序

建议按照以下顺序推进：

1. 明确输入配置 schema
2. 新增 `rokae_world_utils.py`
3. 新增 `rokae_motion_gen.py`
4. 完成 `plan_single` 与 `plan_single_js`
5. 新增 `plan_rokae_motion.py`
6. 输出统一结果文件
7. 接回 MuJoCo 回放
8. 再补 approach / grasp / level carry

## 11. 一句话总结

本次改造的本质，不是把 `curoboV2_demo` 变成另一个 ROS 工程，而是把它升级为一个“离线脚本版的、具备工程化输入输出和障碍物场景能力的 ROKAE CuRobo 规划工具”，在底层能力上向 `dahuafuhe` 对齐，在使用方式上继续保持轻量和独立。
