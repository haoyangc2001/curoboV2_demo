# Bubblify Collision Spheres Workflow

本文档说明如何为当前活动的 ROKAE 机器人更新 `collision_spheres`。

## 目标文件

- 活动 URDF  
  `robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf`
- 活动 robot config  
  `robot_assets/ROKAE/robot/xms5_r800_w4g3b4c_dahuafuhe.yml`
- 活动 collision spheres  
  `robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml`

当前 robot config 已改为通过相对路径引用外部 spheres 文件，因此 `ROKAE_SR5_0.9C_spherized.yml` 是唯一真源。

## 1. 安装 Bubblify

建议在项目使用的 `curoboV2` 环境里安装：

```bash
source /home/tanshan/miniconda3/etc/profile.d/conda.sh
conda activate curoboV2
pip install bubblify
```

## 2. 打开当前活动 URDF

```bash
cd /home/tanshan/rep/curoboV2_demo

bubblify \
  --urdf_path robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf \
  --show_collision
```

如果已经有一份 Bubblify 原始导出，可继续编辑：

```bash
bubblify \
  --urdf_path robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf \
  --spherization_yml /path/to/previous_bubblify_export.yml \
  --show_collision
```

## 3. 手工调球范围

只为以下 link 放置 collision spheres：

- `XMS5-R800-W4G3B4C_base`
- `XMS5-R800-W4G3B4C_link1`
- `XMS5-R800-W4G3B4C_link2`
- `XMS5-R800-W4G3B4C_link3`
- `XMS5-R800-W4G3B4C_link4`
- `XMS5-R800-W4G3B4C_link5`
- `XMS5-R800-W4G3B4C_link6`

不要为 `tool0` 生成球。

建议：

- 优先保证自碰撞和障碍物碰撞的保守一致性。
- 不必强求最少球数，允许总球数与旧版 `64` 不同。
- 对细长 link 优先用沿主轴分布的多球近似，而不是单大球。

## 4. 转换为 cuRobo 格式

将 Bubblify 导出的原始 YAML 转成当前项目使用的 grouped `collision_spheres` 结构：

```bash
python scripts/convert_bubblify_spheres.py \
  --input-yaml /path/to/bubblify_export.yml
```

如需写到其他文件：

```bash
python scripts/convert_bubblify_spheres.py \
  --input-yaml /path/to/bubblify_export.yml \
  --output-yaml /tmp/rokae_candidate_spheres.yml
```

转换脚本会做以下校验：

- 所有球的 link 名必须属于当前活动 robot config
- base 和 6 个 collision link 都至少要有 1 个球
- 每个球必须有 3 维位置和正半径

## 5. 执行 baseline / candidate 压测

先保留当前活动 spheres 作为 baseline，再用新的 candidate 文件对比：

```bash
python scripts/stress_test_rokae_pipeline.py \
  --baseline-spheres robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml \
  --candidate-spheres /tmp/rokae_candidate_spheres.yml
```

输出默认落到：

```text
evidence/rokae_bubblify_stress/<timestamp>/
```

每个 case 目录至少包含：

- `plan/summary.json`
- `plan/trajectory.json`
- `contract/playback_contract.json`
- `playback/playback_summary.json`

总表位于：

- `stress_summary.json`

## 6. 候选切换原则

只有在下面条件满足后，才建议用 candidate 覆盖活动 spheres 文件：

- baseline 与 candidate 都能完整产出 evidence
- `point_to_point` 和 `joint_target` 的无障碍 case，candidate 成功数不低于 baseline
- `approach` / `grasp` / 障碍物 case 的差异可解释，并记录在 evidence 中
- 没有结构性错误，例如 link 名不匹配、合同导出失败、回放 JSON 缺失
