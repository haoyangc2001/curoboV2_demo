# 仓库指南

## 项目结构与模块组织

本仓库是面向 **ROKAE SR5 机械臂** 的脚本优先型 **CuRobo V2 工作流**。主要规划逻辑应放在 `scripts/` 中，包括配置加载、世界模型转换、碰撞球生成，以及主入口 `scripts/run_rokae_pipeline.py`。

将最小示例放在 `demo_scripts/` 中，将回放导出和 MuJoCo 回放工具放在 `playback/` 中，将可复用配置放在 `resource/config/examples/` 下。

机器人 URDF、网格模型和碰撞球 YAML 文件应放在 `robot_assets/` 中。

将 `third_party/curobo/` 视为 vendored upstream subtree，也就是外部上游代码的内置副本：避免做无关修改，本地改动应尽量小，并做好文档说明。

## 测试与开发命令

关键工作流：

```bash
python scripts/run_rokae_pipeline.py --config resource/config/examples/pose_plan_example.yaml
python scripts/plan_rokae_motion.py --config resource/config/examples/joint_plan_example.yaml --output-dir /tmp/rokae_demo
python tests/stress_test_rokae_pipeline.py --output-root evidence/rokae_bubblify_stress
```

第一个命令运行完整的规划 / 合约 / 回放流水线；第二个命令只运行规划部分；第三个命令运行可重复的、类似回归测试的压力测试用例。

## 代码风格与命名规范

遵循现有 Python 风格：使用 4 个空格缩进，模块、函数和变量使用 `snake_case`，常量使用 `UPPER_SNAKE_CASE`，并编写简洁的模块级 docstring。

新增的公共辅助函数建议添加类型注解。CLI 参数解析应尽量靠近脚本入口。

新的自动化逻辑通常应放在 `scripts/` 中，而不是 `demo_scripts/`。

当前没有配置仓库级 formatter，因此请匹配周围代码风格，并保持 imports、路径和注释整洁。

## 测试指南

当前没有顶层覆盖率门槛。修改后，应先用最小但真实的脚本运行来验证；如果改动涉及规划、碰撞球或回放行为，再使用 `tests/stress_test_rokae_pipeline.py` 生成更全面的回归验证证据。

如果修改了 vendored CuRobo 代码，请运行 `third_party/curobo/curobo/tests/` 下有针对性的 `pytest` 测试，并在 PR 中说明任何 CUDA 或 MuJoCo 前置要求。

## 提交与 Pull Request 指南

近期 commit 使用简短、祈使句式的摘要，常见为中文，例如：

```text
完成了curoboV2碰撞球生成
Clean generated artifacts and ignore runtime outputs
```

提交标题应聚焦于单一变更。

PR 应包含以下内容：

* 变更目的
* 受影响的工作流，例如 `scripts/`、`playback/`、assets 或 `third_party/`
* 精确的验证命令
* 当 viewer / playback 输出发生变化时，附上截图或 GIF 路径

不要提交来自 `/tmp`、`evidence/` 或缓存回放产物中的生成文件。
