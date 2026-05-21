# curoboV2_demo

`curoboV2_demo` 是一个面向 ROKAE 机械臂的独立 CuRobo + MuJoCo 演示仓库，用来在不依赖主项目安装树的前提下，完成本地机器人资产加载、位姿规划、轨迹导出和 MuJoCo 回放验证。

当前仓库的激活机器人资产位于 `robot_assets/ROKAE/`，统一使用的 URDF 为 `robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf`。

## 项目目标

- 在仓库内直接加载本地 vendored 的 `third_party/curobo`
- 使用 `robot_assets/ROKAE/` 中的机器人配置和网格资源完成规划
- 将规划结果导出为可复用的回放合同
- 使用 MuJoCo 进行离屏回放，并按需开启实时 viewer
- 将运行产物沉淀到 `evidence/` 目录，便于复查

## 目录说明

```text
curoboV2_demo/
|-- demo_scripts/
|-- playback/
|-- robot_assets/
|   `-- ROKAE/
|-- evidence/
|-- third_party/
|   `-- curobo/
|-- XMS5-R800-W4G3B4C_description/
|-- ROKAE_migration_notes.md
`-- README.md
```

### `demo_scripts/`

规划和资产相关脚本。

- `rokae_asset_utils.py`：统一管理当前工作区内的 ROKAE 资产路径
- `materialize_rokae_assets.py`：从上游资源整理工作区资产包
- `verify_rokae_assets.py`：检查配置、URDF、mesh 和关节顺序是否一致
- `demo_plan_pose_rokae.py`：执行一次 ROKAE 位姿规划，输出 `summary.json`
- `review_rokae_pose_demo.py`：重复执行规划，用于稳定性复核

### `playback/`

规划结果导出和 MuJoCo 回放脚本。

- `export_rokae_playback_contract.py`：把规划结果导出为回放合同
- `replay_rokae_mujoco.py`：读取合同，生成 MJCF，并离屏回放
- `run_rokae_demo.py`：串联规划、合同导出、离屏回放和可选实时 viewer
- `review_rokae_mujoco_playback.py`：批量复跑完整链路

### `robot_assets/ROKAE/`

当前项目实际使用的机器人资产包。

- `start.launch.yaml`：主项目侧启动配置镜像
- `robot/xms5_r800_w4g3b4c_dahuafuhe.yml`：CuRobo 机器人配置
- `robot/curobo/ROKAE_SR5_0.9C.urdf`：当前激活 URDF
- `robot/curobo/meshes/`：URDF 依赖的 mesh 资源
- `bundle_manifest.json`：资产包清单

### `evidence/`

运行输出目录，保存规划摘要、回放合同、GIF、截图和复核结果。

## 当前主流程

本仓库的主流程分为三步：

1. 资产检查：确认 `robot_assets/ROKAE/` 下的 YAML、URDF 和 mesh 可被当前工作区直接解析。
2. 位姿规划：从当前默认关节位姿出发，生成一次小幅相对位移的末端目标并调用 CuRobo 求解。
3. MuJoCo 回放：将轨迹写成合同，生成最小 MJCF，完成离屏渲染和一致性检查。

## 常用入口

在仓库根目录执行：

```bash
python demo_scripts/verify_rokae_assets.py
python demo_scripts/demo_plan_pose_rokae.py --output-dir evidence/tmp_plan
python playback/export_rokae_playback_contract.py --output-dir evidence/tmp_contract
python playback/run_rokae_demo.py --output-root evidence/tmp_run --no-viewer
```

如果需要实时查看 MuJoCo 回放，可去掉 `--no-viewer`。

## 运行依赖

仓库默认假设本机 Python 环境已经具备以下依赖：

- `torch`
- `mujoco`
- `imageio`
- `pyyaml`

此外还需要可用的 CUDA / 图形环境，以满足 CuRobo 和 MuJoCo 的运行需求。

## 说明

- 根 `README.md` 只描述当前仓库本身，不展开迁移过程
- `ROKAE_migration_notes.md` 保留为迁移记录，不作为主使用文档
- `third_party/curobo/` 是 vendored 依赖，项目脚本会优先从这里加载 CuRobo
