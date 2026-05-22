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

## 环境启动（完整流程）

本项目运行在 LXD 容器 `zhongji-dev-2204` 中，与主项目 `zhongji` 共享 GPU 和 CUDA，但 Python 环境完全隔离。

### 1. 从宿主机进入容器

```bash
# 确认容器状态
sudo lxc list --format csv -c n,s | grep zhongji

# 进入容器（交互式 shell）
sudo lxc exec zhongji-dev-2204 -- bash
```

### 2. 激活 conda 环境

容器内使用 miniforge3 管理 Python 环境，CuroboV2 独立环境名为 `curoboV2`：

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate curoboV2
```

验证环境：

```bash
python -c "import torch; print('torch:', torch.__version__); print('cuda:', torch.cuda.is_available())"
# 期望输出: torch: 2.11.0+cu128  cuda: True
```

### 3. 运行 demo

```bash
cd ~/rep/curoboV2_demo/demo_scripts

# 资产检查
python verify_rokae_assets.py

# 位姿规划（输出 summary.json）
python demo_plan_pose_rokae.py --output-dir /tmp/rokae_demo_output

# MuJoCo 回放（需要图形环境）
python ../playback/run_rokae_demo.py --output-root /tmp/rokae_demo_run --no-viewer
```
如果需要实时查看 MuJoCo 回放，可去掉 `--no-viewer`。


### 4. 非交互方式（从宿主机直接执行）

```bash
sudo lxc exec zhongji-dev-2204 -- bash -c "\
  source ~/miniforge3/etc/profile.d/conda.sh && \
  conda activate curoboV2 && \
  cd ~/rep/curoboV2_demo/demo_scripts && \
  python demo_plan_pose_rokae.py --output-dir /tmp/rokae_demo_output"
```




## 运行依赖

`curoboV2` conda 环境已预装以下依赖：

| 包 | 版本 | 说明 |
|---|---|---|
| `torch` | 2.11.0+cu128 | GPU 加速，CUDA 12.8 |
| `cuda-core` | 1.0.1 | CuRobo V2 后端必需 |
| `cuda-bindings` | 12.9.4 | CUDA Python 绑定 |
| `mujoco` | - | 离屏回放和可视化 |
| `pyyaml` | - | 配置文件解析 |
| `imageio` | - | 图像/GIF 导出 |

容器共享宿主机的 NVIDIA GPU 驱动和 CUDA 12.8 运行时，无需额外安装。

## 环境隔离说明

| 项目 | conda 环境 | Python | CuRobo 版本 |
|---|---|---|---|
| zhongji（主项目） | `zhongji` | 3.10 | V1 |
| curoboV2_demo | `curoboV2` | 3.10 | V2 |

两个环境共享系统级 CUDA 12.8，互不干扰。

## 说明

- 根 `README.md` 只描述当前仓库本身，不展开迁移过程
- `ROKAE_migration_notes.md` 保留为迁移记录，不作为主使用文档
- `third_party/curobo/` 是 vendored 依赖，项目脚本会优先从这里加载 CuRobo
- CuRobo V2 默认不编译 pybind CUDA 扩展；如需启用，设置环境变量 `CUROBO_USE_PYBIND=1`
