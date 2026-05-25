# curoboV2_demo ROKAE CuRoboV2 自动碰撞球与 Bubblify 可视化验证实验计划

## 1. 背景

当前 `curoboV2_demo` 已完成 **CuRoboV2 自动生成碰撞球** 的接入，主规划链路默认启用：

- `scripts/plan_rokae_motion.py`
- `scripts/run_rokae_pipeline.py`
- `scripts/rokae_asset_utils.py`
- `scripts/generate_rokae_spheres.py`

同时，仓库保留了基于 **Bubblify** 的碰撞球可视化与手工校对流程：

- `doc/plan/rokae_curoboV2_auto_spheres_bubblify_validation/bubblify_workflow.md`
- `scripts/convert_bubblify_spheres.py`

本次实验不是开发新功能，而是对“自动生成碰撞球”做一轮工程化验证，重点回答以下问题：

- 自动生成的球是否完整覆盖 base 与各 collision link
- 球数量是否合理，是否存在明显过密或过疏
- 是否存在明显漏包、过度膨胀、局部形状失真
- 接入规划后，是否对成功率、导出、回放产生回归

## 2. 实验目标

本次实验的目标分为两条主线：

1. 几何质量验证

- 验证自动生成碰撞球的覆盖质量、突出度和数量分布
- 通过 Bubblify 或等价可视化手段做人工复核

2. 规划回归验证

- 验证 candidate 碰撞球接入后，规划、合同导出、MuJoCo 回放链路不退化

## 3. 范围与约束

### 3.1 纳入本次实验的范围

- ROKAE 当前活动 URDF：`robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf`
- 当前活动 robot config：`robot_assets/ROKAE/robot/xms5_r800_w4g3b4c_robot.yml`
- 当前目标 spheres 输出路径：`robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml`
- 自动生成工具：`scripts/generate_rokae_spheres.py`
- 压测工具：`scripts/stress_test_rokae_pipeline.py`

### 3.2 不纳入本次实验的范围

- 不修改 `tashan_robot`
- 不重构 CuRobo 主规划框架
- 不把 Bubblify 作为新的球生成真源
- 不在本阶段讨论真机联动

### 3.3 实验约束

- 只为 `XMS5-R800-W4G3B4C_base` 与 `link1..link6` 评估碰撞球
- `tool0` 不纳入球覆盖目标
- baseline 与 candidate 对比时必须保留原始 evidence
- 在新的可用 baseline 生成并验收前，不得把任何 candidate 视为活动 spheres 文件

## 4. 当前代码现状

### 4.1 自动生成碰撞球的接入路径

- `scripts/plan_rokae_motion.py` 和 `scripts/run_rokae_pipeline.py` 默认启用 `auto_generate_spheres`
- `scripts/rokae_asset_utils.py` 中 `resolve_robot_config_for_workspace()` 负责将 robot config 中的 spheres 来源解析为：
  - 运行时自动生成
  - 或从 YAML 文件加载
- `scripts/generate_rokae_spheres.py` 可将自动生成结果落盘，并输出 metrics

### 4.2 Bubblify 在当前仓库中的角色

Bubblify 当前主要用于：

- 可视化检查球覆盖情况
- 人工辅助复核
- 将手工或导出的结果转为项目使用的 grouped `collision_spheres` YAML

它不是当前默认规划链路的球生成器。

### 4.3 已有验证基础

- `scripts/stress_test_rokae_pipeline.py` 已支持 baseline / candidate 全链路对比
- `README.md` 与 `doc/plan/rokae_curoboV2_auto_spheres_bubblify_validation/bubblify_workflow.md` 已说明推荐命令与切换原则

### 4.4 当前已确认的问题

- 历史 `robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml` 来源于旧 `dahuafuhe` spheres 的 remap seed，不匹配当前 ROKAE mesh。
- 经过可视化复核，已确认该文件无法正确包裹当前 `XMS5-R800-W4G3B4C` 的 mesh，不能继续作为 baseline 或活动 spheres 文件使用。
- 该文件已从仓库移除，后续需要重新生成一份新的可用 baseline，并在验证通过后再固化到同一路径。
- 在新 baseline 落盘前，主规划链路应继续依赖运行时 `auto_generate_spheres`，而不是文件加载模式。

## 5. 验收标准

本次实验完成后，新生成的 baseline / candidate 方案至少应满足以下标准：

1. 结构完整性

- base 与 6 个 collision link 都有球
- 不为 `tool0` 生成球

2. 几何合理性

- 无明显漏包
- 无明显过度膨胀
- 细长 link 由多球覆盖而非单个超大球近似

3. 指标可接受

- `coverage`、`protrusion`、`surface_gap_mean` 无异常 link
- 总球数与各 link 球数处于可解释范围

4. 回归不劣化

- `point_to_point` 与 `joint_target` 无障碍 case 成功数不低于 baseline
- `approach`、`grasp`、带障碍物 case 如有差异，必须可解释并记录
- 合同导出与 MuJoCo 回放不能出现结构性失败

## 6. 分步骤实验计划

## Step 1. 固化实验输入与记录模板

### 工作目标

统一 baseline、candidate、输出目录和记录字段，避免后续样本混乱。

### 需要做的事情

- 明确 baseline 生成策略：
  - 先用 `scripts/generate_rokae_spheres.py` 生成一份新的持久化 baseline
  - baseline 通过几何复核后，再作为后续对比基准
- 明确 baseline 目标路径：
  - `robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml`
- 约定 candidate 输出目录，例如 `/tmp/rokae_auto_spheres_candidates/`
- 建立一份实验记录表，字段至少包含：
  - baseline 文件路径
  - candidate 名称
  - sphere_density
  - 总球数
  - 各 link 球数
  - metrics 摘要
  - 可视化结论
  - 压测摘要路径

### 完成标志

- 所有人使用同一套 baseline / candidate 命名规则和证据路径

## Step 2. 先生成新的 baseline，再生成多档 candidate 碰撞球

### 工作目标

先补齐一个与当前 mesh 匹配的持久化 baseline，再形成多个候选档位。

### 建议档位

- baseline：`density=0.3`
- `density=0.2`
- `density=0.3`
- `density=0.5`

### 需要做的事情

- 先生成新的 baseline 文件，写入目标路径
- 再逐档执行 `scripts/generate_rokae_spheres.py`
- 每档启用 `--compute-metrics`
- candidate 输出到独立文件，禁止在未验收前覆盖 baseline 文件

### 参考命令

```bash
python scripts/generate_rokae_spheres.py --sphere-density 0.3 --compute-metrics --output robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml

python scripts/generate_rokae_spheres.py --sphere-density 0.2 --compute-metrics --output /tmp/rokae_auto_spheres_candidates/rokae_spheres_d02.yml
python scripts/generate_rokae_spheres.py --sphere-density 0.3 --compute-metrics --output /tmp/rokae_auto_spheres_candidates/rokae_spheres_d03.yml
python scripts/generate_rokae_spheres.py --sphere-density 0.5 --compute-metrics --output /tmp/rokae_auto_spheres_candidates/rokae_spheres_d05.yml
```

### 完成标志

- 产出 1 份新的 baseline YAML
- 产出 3 份 candidate YAML
- 每份 candidate 都有控制台 metrics 记录

## Step 3. 做静态指标分析

### 工作目标

先用指标筛掉明显不合理的 candidate，再进入人工可视化与压测。

### 需要做的事情

- 统计每个 candidate 的：
  - 总球数
  - 每个 link 的球数
  - `coverage`
  - `protrusion`
  - `protrusion_dist_mean`
  - `surface_gap_mean`
  - `volume_ratio`
- 标记异常 link：
  - 球数异常少
  - `coverage` 明显偏低
  - `protrusion` 明显偏高

### 完成标志

- 得到一张 candidate 对比表
- 明确 1 到 2 个优先候选

## Step 4. 做几何可视化复核

### 工作目标

验证自动生成球在几何层面是否“看起来正确”。

### 需要做的事情

- 使用 `scripts/generate_rokae_spheres.py --visualize` 检查 baseline 和 candidate 的自动拟合结果
- 使用 Bubblify 打开活动 URDF，对 candidate 结果做人工复核
- 至少检查以下 link：
  - `base`
  - `link2`
  - `link4`
  - `link6`
- 对以下问题截图记录：
  - 漏包
  - 过度膨胀
  - 关节附近空洞
  - 细长杆件覆盖不连续

### 参考命令

```bash
python scripts/generate_rokae_spheres.py --sphere-density 0.3 --visualize --output /tmp/rokae_auto_spheres_candidates/rokae_spheres_d03.yml

bubblify --urdf_path robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf --show_collision
```

### 完成标志

- baseline 与每个候选至少各有一轮人工复核结论
- 有截图或文字证据说明是否存在局部缺陷

## Step 5. 做最小功能烟测

### 工作目标

确认新 baseline 与 candidate 接入主规划链路后，不会在基本功能上直接失效。

### 需要做的事情

- 运行自动生成模式的最小规划
- 运行文件加载模式的最小规划
- 各自至少覆盖：
  - `point_to_point`
  - `joint_target`
  - 一个带障碍物 case
- 检查是否产出：
  - `trajectory.json`
  - `summary.json`
  - 合同 JSON
  - `playback_summary.json`

### 参考命令

```bash
python scripts/run_rokae_pipeline.py --config resource/config/examples/pose_plan_example.yaml --sphere-density 0.3 --no-viewer
python scripts/run_rokae_pipeline.py --config resource/config/examples/joint_plan_complex_viewer.yaml --sphere-density 0.3 --no-viewer
python scripts/run_rokae_pipeline.py --config resource/config/examples/pose_plan_example.yaml --no-auto-generate-spheres --no-viewer
```

### 完成标志

- 自动生成与基于新 baseline 文件加载两条路径都能独立跑通

## Step 6. 做 baseline / candidate 全链路压测

### 工作目标

对新 baseline 与优先 candidate 做正式回归对比，作为最终结论依据。

### 需要做的事情

- 对每个优先 candidate 执行 `scripts/stress_test_rokae_pipeline.py`
- baseline 参数必须指向新生成且通过复核的 spheres 文件
- 收集：
  - `stress_summary.json`
  - `round_records.json`
  - 失败 case 的 stdout/stderr
  - 代表性 case 的 plan / contract / playback 产物

### 参考命令

```bash
python scripts/stress_test_rokae_pipeline.py \
  --baseline-spheres robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml \
  --candidate-spheres /tmp/rokae_auto_spheres_candidates/rokae_spheres_d03.yml
```

### 重点分析项

- `plan_success_count`
- `contract_success_count`
- `playback_success_count`
- 各 mode 的成功数变化
- 失败是否集中在某类模式、某类障碍物或某类速度尺度

### 完成标志

- 至少完成 1 轮“新 baseline / candidate”正式对比
- 失败 case 都有解释或归因

## Step 7. 对问题 link 做局部复查与二次验证

### 工作目标

如果 candidate 暴露局部问题，进行最小范围复查，而不是盲目调全局参数。

### 需要做的事情

- 根据 metrics、截图和压测失败 case 定位问题 link
- 判断采用哪种修正方式：
  - 提高整体 `sphere_density`
  - 使用 `--from-config` + `--refit-link` 定向重拟合
  - 保持自动生成方案不变，仅记录限制
- 对修正后的 candidate 复跑对应 smoke case 或回归 case

### 完成标志

- 每个异常 link 都有处理结论
- 修正动作和复验结果有记录

## Step 8. 形成结论并决定默认方案

### 工作目标

给出可以执行的最终结论，而不是停留在主观印象。

### 需要做的事情

- 明确推荐默认 `sphere_density`
- 明确是否保留 `auto_generate_spheres: true` 作为默认策略
- 明确是否继续保留新 baseline，或将更优 candidate 固化为活动 spheres 文件
- 汇总 evidence 路径、截图路径、压测摘要路径
- 记录已知限制与后续待办

### 完成标志

- 输出一份实验结论
- 能回答“默认用哪一档、为什么、风险是什么”

## 7. 关键风险与注意事项

1. `scripts/generate_rokae_spheres.py --stress-test` 若直接输出到 baseline 路径，baseline 与 candidate 可能指向同一文件，导致对比失效。
2. 主规划默认走“运行时自动生成”，而压测脚本比较的是“持久化 YAML 文件”。两种验证对象必须明确区分。
3. `--refit-link` 的有效使用依赖 `--from-config` 分支，实际操作前需要确认命令路径正确。
4. 若只做可视化、不做压测，无法证明 candidate 对规划行为没有回归。

## 8. 建议产物

本次实验建议至少沉淀以下产物：

- 1 份实验记录表
- 1 份新的 baseline spheres YAML
- 3 份 candidate spheres YAML
- 1 份 candidate 指标对比表
- 1 组 Bubblify / 可视化截图
- 1 组 `stress_summary.json` 与 `round_records.json`
- 1 份最终实验结论文档

## 9. 建议执行顺序

建议按以下顺序推进：

1. 先生成新的 baseline，再生成 3 档 candidate 并记录 metrics
2. 再做可视化复核，筛掉明显不合理的候选
3. 只对 1 到 2 个优先 candidate 做正式压测
4. 如有异常，再做局部 refit 或 density 调整
5. 最后决定默认方案，并确认哪个文件成为新的活动 spheres 文件
