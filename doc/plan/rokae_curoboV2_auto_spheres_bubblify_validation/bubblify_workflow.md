# Bubblify 可视化验证说明

本文档用于说明在 `curoboV2_demo` 中如何使用 **Bubblify** 辅助验证自动生成碰撞球。

本文档本身保留在 `doc/plan/rokae_curoboV2_auto_spheres_bubblify_validation/`。
在执行 Bubblify 过程中产出的截图、人工结论和对比记录，应统一归档到 `doc/experiments/rokae_curoboV2_auto_spheres_bubblify_validation/`。

本项目当前的默认策略是：

- 规划运行时优先使用 **CuRobo V2 自动生成碰撞球**
- 目标 spheres 输出路径为 `robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml`
- 当前历史 remap seed 已确认不可用并已删除
- **Bubblify 不是默认球生成器，也不是当前链路的唯一真源**

因此，Bubblify 在本项目中的作用应明确限定为：

- 查看自动生成碰撞球是否覆盖合理
- 辅助人工识别漏包、过度膨胀、局部空洞
- 为实验记录提供截图和人工结论

不应将本文件理解为“手工调球主流程”。

## 1. 验证对象

- 活动 URDF：`robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf`
- 活动 robot config：`robot_assets/ROKAE/robot/xms5_r800_w4g3b4c_robot.yml`
- 目标 spheres 输出路径：`robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml`

本次验证重点关注：

- `XMS5-R800-W4G3B4C_base`
- `XMS5-R800-W4G3B4C_link1`
- `XMS5-R800-W4G3B4C_link2`
- `XMS5-R800-W4G3B4C_link3`
- `XMS5-R800-W4G3B4C_link4`
- `XMS5-R800-W4G3B4C_link5`
- `XMS5-R800-W4G3B4C_link6`

`tool0` 不属于本次碰撞球验证范围。

## 2. 使用场景

建议只在以下场景使用 Bubblify：

1. 自动生成 candidate 后，做人工几何复核。
2. 不同 `sphere_density` 候选之间，做覆盖效果对比。
3. 压测失败后，回看可疑 link 是否存在明显几何问题。

不建议把 Bubblify 作为：

- 默认碰撞球生成入口
- 日常规划运行前置步骤
- 替代 `scripts/generate_rokae_spheres.py` 的主工作流

## 3. 安装

建议在项目使用的 `curoboV2` 环境中安装：

```bash
source /home/tanshan/miniconda3/etc/profile.d/conda.sh
conda activate curoboV2
pip install bubblify
```

## 4. 打开 URDF 做可视化检查

```bash
cd /home/tanshan/rep/curoboV2_demo

bubblify \
  --urdf_path robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf \
  --show_collision
```

如果已经有一份 Bubblify 导出结果或新生成的 baseline / candidate 文件，需要回看球布局，也可以这样打开：

```bash
bubblify \
  --urdf_path robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf \
  --spherization_yml /path/to/previous_bubblify_export.yml \
  --show_collision
```

## 5. 检查重点

使用 Bubblify 时，重点不是“把球调到最好看”，而是判断自动生成结果是否存在明显问题。

建议重点看：

- `base` 是否有大面积漏包
- `link2`、`link4` 这类细长件是否由连续多球覆盖
- `link6` 附近是否有末端漏包或球过大
- 关节附近是否存在明显空洞
- 是否出现单个超大球跨越多个局部结构

建议记录以下结论：

- 是否存在明显漏包
- 是否存在明显过度膨胀
- 哪个 link 最可疑
- 是否需要进入下一步压测或局部复查

## 6. 与自动生成流程的关系

推荐流程应保持为：

1. 用 `scripts/generate_rokae_spheres.py` 生成 candidate
2. 读取控制台 metrics，先做静态筛选
3. 用 Bubblify 做人工可视化验证
4. 再用 `tests/stress_test_rokae_pipeline.py` 做 baseline / candidate 回归对比

也就是说，Bubblify 位于：

- `metrics` 之后
- `stress test` 之前

它是验证环节，不是生产环节。

## 7. 与最终决策的关系

仅凭 Bubblify 可视化结果，不能决定哪个文件应固化为新的活动 spheres 文件。

最终是否切换 candidate，仍应以以下信息综合判断：

- 自动生成 metrics
- Bubblify 人工复核结论
- baseline / candidate 压测结果
- 失败 case 的归因分析

如果只是“看起来包住了”，但规划成功率下降，这个 candidate 仍然不能接受。

## 8. 相关文档

- 本次实验计划：
  `doc/plan/rokae_curoboV2_auto_spheres_bubblify_validation/rokae_curoboV2_auto_spheres_bubblify_validation_plan.md`
- 实验结果目录：
  `doc/experiments/rokae_curoboV2_auto_spheres_bubblify_validation/`
- 自动生成碰撞球脚本：
  `scripts/generate_rokae_spheres.py`
- Bubblify 导出转换脚本：
  `scripts/convert_bubblify_spheres.py`
- 压测脚本：
  `tests/stress_test_rokae_pipeline.py`
