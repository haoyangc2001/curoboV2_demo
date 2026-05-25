# CLAUDE.md

本文件用于说明 **当前项目状态、正在进行的工作、协作时的注意事项**。  
`README.md` 负责介绍项目本身；本文件只关注“现在要完成什么”。

## 项目当前状态

`curoboV2_demo` 当前已经具备以下稳定能力：

- 离线规划入口：`scripts/plan_rokae_motion.py`
- 统一流水线入口：`scripts/run_rokae_pipeline.py`
- 绝对/相对障碍物 world 构建
- `trajectory.json` 为中心的规划输出
- MuJoCo 合同导出与回放链路
- CuRobo V2 自动碰撞球生成

当前活动资产：

- robot config：`robot_assets/ROKAE/robot/xms5_r800_w4g3b4c_robot.yml`
- URDF：`robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf`
- spheres 输出路径：`robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml`

## 当前正在做的工作

当前主任务是：

**验证 CuRobo V2 自动生成碰撞球在当前 ROKAE SR5 模型上的可用性，并确定默认方案。**

对应文档：

- 实验计划：
  `doc/plan/rokae_curoboV2_auto_spheres_bubblify_validation/rokae_curoboV2_auto_spheres_bubblify_validation_plan.md`
- 实验状态 JSON：
  `doc/plan/rokae_curoboV2_auto_spheres_bubblify_validation/rokae_curoboV2_auto_spheres_bubblify_validation_plan.json`
- Bubblify 说明：
  `doc/plan/rokae_curoboV2_auto_spheres_bubblify_validation/bubblify_workflow.md`
- 实验结果目录：
  `doc/experiments/rokae_curoboV2_auto_spheres_bubblify_validation/`

## 已完成的关键工作

- 已确认历史 remap seed spheres 文件不可用。
- 已重建新的 baseline spheres 文件。
- 已分析并修正 `scripts/generate_rokae_spheres.py` 中的毫米/米尺度问题。
- 已确认 `generate_rokae_spheres.py` 使用的是 CuRobo V2 `RobotBuilder.fit_collision_spheres()`。
- 已完成 baseline 的初步 Bubblify 人工复核，当前球形状已恢复到“可继续实验”的状态。
- 已将活动 robot config 名称清理为 `xms5_r800_w4g3b4c_robot.yml`。

## 尚未完成的工作

以下工作默认仍待完成，除非实验 JSON 明确更新为已完成：

- 生成 `density=0.2 / 0.4 / 0.5 / 0.6` 四档 candidate
- 汇总 baseline / candidate metrics 对比
- 使用 Bubblify 对 candidate 做最终人工复核
- 执行自动生成模式 vs 文件加载模式的 smoke test
- 执行 baseline / candidate stress test
- 形成最终默认方案结论

## 当前关键结论

### 1. Bubblify 的角色

Bubblify 在本项目中**只用于几何可视化复核**，不是默认球生成器，也不是主链路真源。

### 2. 自动生成球的来源

`scripts/generate_rokae_spheres.py` 生成的球来自：

1. 当前活动 URDF
2. URDF 中引用的 base/link1..link6 mesh
3. CuRobo V2 `RobotBuilder.fit_collision_spheres()` 拟合结果

不是直接扫描 mesh 目录盲生成。

### 3. 当前已知技术 caveat

当前模型在 CuRobo 这条拟合路径上存在 **mesh scale 传递不完整** 的问题。  
如果不做补救，会生成毫米尺度的超大碰撞球。

当前本地补救方式：

- `scripts/generate_rokae_spheres.py` 已增加自动归一化逻辑
- 若检测到异常大球，会自动从毫米换算到米

### 4. 可视化限制

`generate_rokae_spheres.py --visualize`：

- 适合快速检查球尺度是否异常
- 适合看球是否大致贴住局部 mesh
- **不适合作为最终整机装配正确性的唯一依据**

最终几何复核应优先使用：

```bash
bubblify \
  --urdf_path robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf \
  --spherization_yml robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml \
  --show_collision
```

## 推荐执行顺序

1. 先确认当前 baseline 的 metrics 与 Bubblify 结果
2. 生成 `0.2 / 0.4 / 0.5 / 0.6` 四档 candidate
3. 汇总 metrics 对比，观察 density 变化趋势
4. 做 candidate 的 Bubblify 最终复核
5. 做 smoke test
6. 做 stress test
7. 更新实验 JSON 与最终结论

## 协作注意点

- 不要再把 Bubblify 写成默认球生成主流程。
- 不要把 `--visualize` 的局部调试视图误当作最终装配视图。
- 每次修改项目代码后，必须先验证项目从头到尾是否还能完整执行一遍。
- 每完成一个子阶段，必须对该子阶段做一次压力测试，并把结果写入对应实验记录。
- 修改实验状态时，优先同步更新对应 JSON，而不是只改 Markdown。
- 后续实验生成的 metrics 汇总、截图、对比表、压测摘要和结论文档，统一放到 `doc/experiments/rokae_curoboV2_auto_spheres_bubblify_validation/`。
- 如果某阶段声称完成，必须写清楚：
  - 完成了什么
  - 有什么证据
  - 遇到了什么问题
  - 如何解决

## 常用命令

重新生成 baseline：

```bash
python scripts/generate_rokae_spheres.py \
  --sphere-density 0.3 \
  --compute-metrics \
  --output robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml
```

快速局部检查：

```bash
python scripts/generate_rokae_spheres.py \
  --sphere-density 0.3 \
  --compute-metrics \
  --visualize \
  --output robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml
```

最终几何复核：

```bash
bubblify \
  --urdf_path robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf \
  --spherization_yml robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml \
  --show_collision
```
