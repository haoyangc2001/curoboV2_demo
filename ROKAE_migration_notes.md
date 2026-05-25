# XMS5-R800-W4G3B4C 迁移记录

本文档记录本次在 `curoboV2_demo` 中围绕 `XMS5-R800-W4G3B4C` 机器人模型所做的分析与修改，便于后续回忆和继续处理。

## 1. 本次工作的目标

目标分为两部分：

1. 让 `XMS5-R800-W4G3B4C` 的 URDF 能在本地 `URDF Visualizer` 插件中正常预览。
2. 将项目当前主链路里实际使用的机器人资产，从原来的 `rokae_cr7_dahuafuhe` 切换到 `XMS5-R800-W4G3B4C`。

## 2. 最初发现的问题

### 2.1 URDF Visualizer 无法打开

文件：

- `XMS5-R800-W4G3B4C_description/XMS5-R800-W4G3B4C_description/urdf/XMS5-R800-W4G3B4C.urdf`

原因分析：

- 该 URDF 中 mesh 路径使用的是 `package://XMS5-R800-W4G3B4C_description/...`
- 本地 IDE 的 `URDF Visualizer` 插件通常不会完整走 ROS package 解析流程
- 因此即使文件存在，插件本地直接打开 URDF 时也可能解析失败

结论：

- 对插件预览来说，主要是 `package://` 资源路径问题
- 不是模型文件缺失导致的主问题

### 2.2 launch 文件引用了不存在的 `.xacro`

文件：

- `XMS5-R800-W4G3B4C_description/XMS5-R800-W4G3B4C_description/launch/direct_teach.launch`

问题：

- 原第 3 行引用 `urdf/XMS5-R800-W4G3B4C.xacro`
- 仓库里实际不存在这个文件
- 只存在：
  - `urdf/XMS5-R800-W4G3B4C.urdf`
  - `urdf/materials.xacro`

结论：

- `materials.xacro` 只是材料定义，不是整机入口
- 原 launch 在 ROS/RViz 链路里会因为找不到主 xacro 直接失败

### 2.3 项目当前实际使用的不是 ROS 描述包，而是机器人资产目录

通过全局搜索确认，项目主链路实际在用的是：

- 历史位置为 `robot_assets/dahuafuhe`
- 其中旧主链路曾引用：
  - `robot_assets/dahuafuhe/robot/curobo/rokae_cr7_dahuafuhe.urdf`
  - `robot_assets/dahuafuhe/robot/rokae_cr7_dahuafuhe.yml`
  - `robot_assets/dahuafuhe/robot/spheres/rokae_cr7_dahuafuhe_spherized.yml`
  - `robot_assets/dahuafuhe/start.launch.yaml`

也就是说：

- `XMS5-R800-W4G3B4C_description/...` 是 ROS 描述包
- 当前活动链路已经迁移到 `robot_assets/ROKAE`

## 3. 已做的修改

### 3.1 修复 `URDF Visualizer` 本地预览

修改文件：

- `XMS5-R800-W4G3B4C_description/XMS5-R800-W4G3B4C_description/urdf/XMS5-R800-W4G3B4C.urdf`

修改内容：

- 将所有 mesh 路径从：
  - `package://XMS5-R800-W4G3B4C_description/meshes/...`
- 改为：
  - `../meshes/...`

目的：

- 让本地插件不依赖 ROS package 解析
- 可以直接通过相对路径找到 STL 文件

### 3.2 修复 `direct_teach.launch`

修改文件：

- `XMS5-R800-W4G3B4C_description/XMS5-R800-W4G3B4C_description/launch/direct_teach.launch`

修改内容：

- 将原先通过不存在的 `.xacro` 生成 `robot_description`
- 改为直接通过现有 `.urdf` 加载：
  - `textfile="$(find XMS5-R800-W4G3B4C_description)/urdf/XMS5-R800-W4G3B4C.urdf"`

影响：

- ROS/RViz 启动不再依赖缺失的主 `.xacro`
- 失去 xacro 参数化能力，但当前仓库本身就没有对应主 xacro，因此这是更务实的修法

### 3.3 修复 `XMS5-R800-W4G3B4C.rviz` 中的 joint 名

修改文件：

- `XMS5-R800-W4G3B4C_description/XMS5-R800-W4G3B4C_description/launch/XMS5-R800-W4G3B4C.rviz`

修改内容：

- 将旧的 `xmate_joint_*` 替换为：
  - `XMS5-R800-W4G3B4C_joint_1`
  - `XMS5-R800-W4G3B4C_joint_2`
  - `XMS5-R800-W4G3B4C_joint_3`
  - `XMS5-R800-W4G3B4C_joint_4`
  - `XMS5-R800-W4G3B4C_joint_5`
  - `XMS5-R800-W4G3B4C_joint_6`

目的：

- 避免 RViz 关节显示项与新 URDF 的 joint 名不一致

## 4. 对项目主链路的迁移

### 4.1 新增并切换了 `robot_assets/ROKAE` 的活动模型

新增活动文件：

- `robot_assets/ROKAE/robot/curobo/xms5_r800_w4g3b4c_dahuafuhe.urdf`
- `robot_assets/ROKAE/robot/xms5_r800_w4g3b4c_robot.yml`
- `robot_assets/ROKAE/robot/spheres/xms5_r800_w4g3b4c_spherized.yml`

说明：

- 新 URDF 基于 `XMS5-R800-W4G3B4C.urdf` 拷贝并适配到 `robot_assets/ROKAE/robot/curobo`
- 同时将 mesh 路径改成了该资产目录下的相对路径
- 给末端额外补了一个 `tool0` 固定 link，便于沿用当前 demo 中按 `tool0` 做规划目标的逻辑

### 4.2 已切换 `start.launch.yaml`

修改文件：

- `robot_assets/ROKAE/start.launch.yaml`

已切换内容：

- `robot.urdf` 改为新 URDF
- `rviz.fixed_frame` 改为 `XMS5-R800-W4G3B4C_base`
- `obstacle_panel.frame_id` 改为 `XMS5-R800-W4G3B4C_base`
- `scene_mesh_marker.frame_id` 改为 `XMS5-R800-W4G3B4C_base`
- `robot_driver.joint_name` 改为新 joint 名
- `driver_common.joint_name` 改为新 joint 名
- `sim_joint_state_publisher.joint_names` 改为新 joint 名
- `trajectory_planning.robot_config` 改为新 robot config

### 4.3 已切换路径助手

修改文件：

- `demo_scripts/dahuafuhe_asset_utils.py`

已切换常量：

- `ROBOT_CONFIG_NAME`
- `URDF_NAME`
- `SPHERES_NAME`

现在默认指向：

- `xms5_r800_w4g3b4c_robot.yml`
- `xms5_r800_w4g3b4c_dahuafuhe.urdf`
- `xms5_r800_w4g3b4c_spherized.yml`

### 4.4 已更新说明文件

修改文件：

- `robot_assets/ROKAE/README.md`
- `robot_assets/ROKAE/bundle_manifest.json`

说明：

- 文档中已标记当前活动模型为 `XMS5-R800-W4G3B4C`

## 5. 当前模型切换后的核心文件

当前项目里建议优先关注以下文件：

- ROS 描述包入口  
  `XMS5-R800-W4G3B4C_description/XMS5-R800-W4G3B4C_description/urdf/XMS5-R800-W4G3B4C.urdf`

- ROS launch  
  `XMS5-R800-W4G3B4C_description/XMS5-R800-W4G3B4C_description/launch/direct_teach.launch`

- 项目实际使用的活动 URDF  
  `robot_assets/ROKAE/robot/curobo/xms5_r800_w4g3b4c_dahuafuhe.urdf`

- 项目实际使用的活动 robot config  
  `robot_assets/ROKAE/robot/xms5_r800_w4g3b4c_robot.yml`

- 项目实际使用的活动球碰撞配置  
  `robot_assets/ROKAE/robot/spheres/xms5_r800_w4g3b4c_spherized.yml`

- 项目启动配置  
  `robot_assets/ROKAE/start.launch.yaml`

## 6. 当前仍然存在的风险和限制

### 6.1 球碰撞模型已经切换为 Bubblify 工作流

当前项目已经把 `collision_spheres` 的接入方式切换为**外部 YAML 文件引用**：

- `robot_assets/ROKAE/robot/xms5_r800_w4g3b4c_robot.yml`
  中的 `robot_cfg.kinematics.collision_spheres`
- 现在指向：
  `robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml`

推荐的更新方式不再是依赖本机的 cuRobo `RobotBuilder` 自动拟合，而是：

1. 用 Bubblify 打开当前活动 URDF  
   `robot_assets/ROKAE/robot/curobo/ROKAE_SR5_0.9C.urdf`
2. 在浏览器里手工调整 `XMS5-R800-W4G3B4C_base` 和 `link1..link6` 的球
3. 导出 Bubblify 原始 YAML
4. 用 `scripts/convert_bubblify_spheres.py` 转换成当前项目使用的 cuRobo `collision_spheres` 格式

这样做的好处是：

- spheres 文件成为唯一真源，robot config 不再保留内嵌副本
- 可以在不编辑主 robot config 的情况下迭代球模型
- 转换脚本会对 link 名、半径和缺失 link 做静态校验

仍需注意：

- Bubblify 本身是手工交互工具，球布局质量仍取决于人工调整
- 生成新 spheres 后，必须重新跑规划和 MuJoCo 回放压测

### 6.2 旧资产还保留在仓库中

以下旧文件目前仍保留在历史目录 `robot_assets/dahuafuhe` 中：

- `robot_assets/dahuafuhe/robot/rokae_cr7_dahuafuhe.yml`
- `robot_assets/dahuafuhe/robot/curobo/rokae_cr7_dahuafuhe.urdf`
- `robot_assets/dahuafuhe/robot/spheres/rokae_cr7_dahuafuhe_spherized.yml`

原因：

- 保留旧文件便于回退和对照
- 当前活动链路已经不再默认引用它们

## 7. 后续建议

后续如果要把这套模型真正用于更可信的规划和实机联调，建议按下面顺序继续：

1. 完成一版 Bubblify 手工调球，并通过 `scripts/convert_bubblify_spheres.py` 覆盖活动 spheres 文件。
2. 用 `tests/stress_test_rokae_pipeline.py` 先跑 `baseline`，再跑 `candidate`，保留 evidence。
3. 根据压测结果微调 base/link1..link6 的球布局，重点观察障碍物附近规划与 `grasp` 阶段稳定性。
4. 根据真实末端工具安装关系，确认 `tool0` 的固定变换是否应继续是 `0 0 0`，还是需要设置真实 TCP 偏移。
5. 根据实机驱动要求，确认 `start.launch.yaml` 中使用的新 joint 名是否与下游驱动节点/状态话题完全一致。

## 8. 本次没有做的事情

以下内容本次未执行：

- 未运行测试
- 未执行规划验证
- 未执行 ROS launch 实测
- 未执行实机联调
- 未在本仓库中提交新的 Bubblify 手工调球结果
- 未执行 Bubblify candidate 压测（需要真实导出的 candidate YAML）

本次工作仅完成了：

- 外部 spheres 文件接入
- Bubblify 转换脚本
- 全链路压测脚本
- 文档记录
