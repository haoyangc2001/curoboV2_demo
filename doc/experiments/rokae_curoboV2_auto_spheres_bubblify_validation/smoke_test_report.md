# Smoke Test 报告

- 生成时间：2026-05-25 06:54 UTC
- 测试目的：验证自动生成模式与文件加载模式在主规划链路中的最小功能可用性

---

## 测试结果总览

| 测试项 | 规划模式 | 球来源 | 结果 | 路径点数 | 耗时 |
|--------|----------|--------|------|----------|------|
| 自动生成 + 点到点 | point_to_point | auto_generate(density=0.6) | PASS | 81 | 30.4s |
| 文件加载 + 点到点 | point_to_point | file_load(candidate_density_0.6_pw10_cw1000.yml) | PASS | 81 | 15.3s |
| 自动生成 + 关节目标 | joint_target | auto_generate(density=0.6) | PASS | 41 | 26.2s |

## 自动生成 + 点到点

- 规划模式：point_to_point
- 球来源：auto_generate(density=0.6)
- 结果：PASS
- 路径点数：81
- 耗时：30.4s
- 轨迹文件：`/tmp/smoke_auto_p2p_z766iqqh/trajectory.json`

## 文件加载 + 点到点

- 规划模式：point_to_point
- 球来源：file_load(candidate_density_0.6_pw10_cw1000.yml)
- 结果：PASS
- 路径点数：81
- 耗时：15.3s
- 轨迹文件：`/tmp/smoke_file_p2p_biuyjazf/trajectory.json`

## 自动生成 + 关节目标

- 规划模式：joint_target
- 球来源：auto_generate(density=0.6)
- 结果：PASS
- 路径点数：41
- 耗时：26.2s
- 轨迹文件：`/tmp/smoke_auto_joint_w2v6josk/trajectory.json`

---

## 结论

**全部 3 项测试通过。**

自动生成模式与文件加载模式均可正常完成规划，碰撞球验证无回归。