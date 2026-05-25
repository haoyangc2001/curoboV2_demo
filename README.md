# curoboV2_demo

面向 **ROKAE SR5** 机械臂的离线 **CuRobo V2** 规划工具。项目提供：

- YAML 配置驱动的离线规划入口
- 绝对/相对障碍物 JSON 到 CuRobo world 的转换
- `trajectory.json` 为中心的统一输出
- MuJoCo 合同导出、离屏回放与 realtime viewer
- 基于 CuRobo V2 的自动碰撞球生成

项目目标不是复刻 ROS 工程，而是提供一套**独立、轻量、可脚本化执行**的规划与回放链路。

## 核心能力

| 能力 | 说明 |
|------|------|
| 规划模式 | `point_to_point` / `joint_target` / `approach` / `grasp` |
| 场景建模 | 支持 `abs.autosave.json` / `rel.autosave.json` 转换为 CuRobo cuboid world |
| 速度与约束 | 支持 `speed_scale`、`hold_vec_weight`、`approach_offset` |
| 回放链路 | `trajectory.json` → `playback_contract.json` → MuJoCo GIF / viewer |
| 碰撞球 | 默认启用 CuRobo V2 自动生成，可导出为独立 YAML |

## 项目结构

```text
curoboV2_demo/
├── scripts/                          # 主规划、世界建模、碰撞球生成
├── playback/                         # 合同导出与 MuJoCo 回放
├── resource/config/examples/         # 示例配置与障碍物
├── robot_assets/ROKAE/               # 当前活动机器人资产
├── doc/experiments/                  # 实验结果、截图、对比表与结论沉淀
├── doc/plan/                         # 计划文档、实验方案与长期升级计划
└── third_party/curobo/               # Vendored CuRobo V2
```

重点文件：

- `scripts/run_rokae_pipeline.py`：推荐主入口
- `scripts/plan_rokae_motion.py`：规划子阶段入口
- `scripts/generate_rokae_spheres.py`：自动生成碰撞球
- `playback/export_rokae_playback_contract.py`：规划结果转回放合同
- `playback/replay_rokae_mujoco.py`：MuJoCo 离屏回放

## 环境

推荐在 `curoboV2` conda 环境中运行：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate curoboV2
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

URDF 统一使用：

```text
robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf
```

## 快速开始

一条命令跑完整链路：

```bash
python scripts/run_rokae_pipeline.py \
  --config resource/config/examples/pose_plan_example.yaml
```

仅执行规划：

```bash
python scripts/plan_rokae_motion.py \
  --config resource/config/examples/pose_plan_example.yaml \
  --output-dir /tmp/rokae_demo/pose_plan
```

规划结果生成 MuJoCo 回放：

```bash
python playback/export_rokae_playback_contract.py \
  --plan-output-dir /tmp/rokae_demo/pose_plan \
  --output-dir /tmp/rokae_demo/contract

MUJOCO_GL=egl python playback/replay_rokae_mujoco.py \
  --contract-json /tmp/rokae_demo/contract/playback_contract.json \
  --output-dir /tmp/rokae_demo/playback
```

## 碰撞球说明

当前默认策略：

- 主规划链路默认启用 `auto_generate_spheres: true`
- `sphere_density` 默认值为 `0.3`
- 自动生成脚本为 `scripts/generate_rokae_spheres.py`

单独生成并保存碰撞球：

```bash
python scripts/generate_rokae_spheres.py \
  --sphere-density 0.3 \
  --compute-metrics \
  --output robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml
```

说明：

- 该脚本读取当前活动 URDF，并基于 URDF 中引用的 link mesh 拟合 collision spheres
- 脚本内已包含针对当前模型的毫米/米尺度归一化补丁
- `--visualize` 适合做快速局部检查；最终几何复核建议使用 Bubblify

## 主要输出

| 文件 | 说明 |
|------|------|
| `trajectory.json` | 统一规划输出，主链路成功后默认保留 |
| `playback_contract.json` | MuJoCo 回放合同 |
| `playback.gif` | 离屏回放动图（按需） |
| `playback_start.png` / `playback_end.png` | 回放首尾帧（按需） |

## 相关文档

- 实验计划：`doc/plan/rokae_curoboV2_auto_spheres_bubblify_validation/`
- 实验产物：`doc/experiments/rokae_curoboV2_auto_spheres_bubblify_validation/`
- 升级方案：`doc/plan/rokae_curobo_planning_upgrade_plan/`
- 资产说明：`robot_assets/ROKAE/README.md`
- 贡献说明：`AGENTS.md`

## 注意事项

- 配置中的 pose 四元数顺序是 `[qx, qy, qz, qw]`，脚本内部会转换为 CuRobo 使用的 `[qw, qx, qy, qz]`
- `speed_scale` 通过 robot config 的 `velocity_scale` 实现，必须在规划器构造时生效
- `generate_rokae_spheres.py --visualize` 的 mesh 叠加显示不是最终整机装配验证视图
- 当前主规划默认走自动生成球；如切换为文件加载，请先确认 spheres 文件已通过几何与回归验证
