# Bubblify 几何复核报告（自动化验证）

- 生成时间：2026-05-25 09:02 UTC
- Baseline：`robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml`
- 候选文件：`doc/experiments/rokae_curoboV2_auto_spheres_bubblify_validation/candidate_morphit_5.yml`, `doc/experiments/rokae_curoboV2_auto_spheres_bubblify_validation/candidate_morphit_5_inflated.yml`

> 本报告由 `tests/validate_spheres_geometry.py` 自动生成，
> 作为 Bubblify 交互式可视化复核的程序化补充。
> 最终几何结论仍建议结合 Bubblify 可视化确认。

---

## 1. Baseline 几何概览

| Link | 球数 | 最小半径 | 最大半径 | 平均半径 | bbox 对角线 | 体积 |
|------|------|----------|----------|----------|-------------|------|
| XMS5-R800-W4G3B4C_base | 5 | 0.0081m | 0.0121m | 0.0104m | 0.1249m | 0.000025m³ |
| XMS5-R800-W4G3B4C_link1 | 30 | 0.0113m | 0.0495m | 0.0215m | 0.2676m | 0.002331m³ |
| XMS5-R800-W4G3B4C_link2 | 31 | 0.0073m | 0.0289m | 0.0182m | 0.4642m | 0.001043m³ |
| XMS5-R800-W4G3B4C_link3 | 31 | 0.0037m | 0.0395m | 0.0169m | 0.2507m | 0.001040m³ |
| XMS5-R800-W4G3B4C_link4 | 34 | 0.0053m | 0.0208m | 0.0147m | 0.2331m | 0.000509m³ |
| XMS5-R800-W4G3B4C_link5 | 31 | 0.0027m | 0.0450m | 0.0203m | 0.1663m | 0.002066m³ |
| XMS5-R800-W4G3B4C_link6 | 30 | 0.0042m | 0.0076m | 0.0051m | 0.1009m | 0.000019m³ |

### Baseline 检测到的异常

- **XMS5-R800-W4G3B4C_link3**：球大小差异过大（max/min = 10.6），可能存在局部不均匀
- **XMS5-R800-W4G3B4C_link5**：球大小差异过大（max/min = 16.8），可能存在局部不均匀

---

## 2. Candidate: candidate_morphit_5

| Link | 球数 | 最小半径 | 最大半径 | 平均半径 | bbox 对角线 | 体积 |
|------|------|----------|----------|----------|-------------|------|
| XMS5-R800-W4G3B4C_base | 2 | 0.0059m | 0.0059m | 0.0059m | 0.0726m | 0.000002m³ |
| XMS5-R800-W4G3B4C_link1 | 5 | 0.0352m | 0.0369m | 0.0359m | 0.2053m | 0.000973m³ |
| XMS5-R800-W4G3B4C_link2 | 5 | 0.0286m | 0.0508m | 0.0376m | 0.3941m | 0.001357m³ |
| XMS5-R800-W4G3B4C_link3 | 5 | 0.0370m | 0.0410m | 0.0387m | 0.1759m | 0.001222m³ |
| XMS5-R800-W4G3B4C_link4 | 5 | 0.0161m | 0.0432m | 0.0351m | 0.1704m | 0.001101m³ |
| XMS5-R800-W4G3B4C_link5 | 5 | 0.0290m | 0.0306m | 0.0302m | 0.0750m | 0.000578m³ |
| XMS5-R800-W4G3B4C_link6 | 5 | 0.0097m | 0.0115m | 0.0105m | 0.0778m | 0.000025m³ |

### candidate_morphit_5 检测到的异常

- **XMS5-R800-W4G3B4C_base**：球数过少（2），可能覆盖不足
- **XMS5-R800-W4G3B4C_link5**：最大球直径(0.0612m)占 bounding box 对角线(0.0750m)的 82%，可能过度膨胀

### candidate_morphit_5 vs Baseline 差异

- **XMS5-R800-W4G3B4C_base**：球数变化 -60%（5 → 2）
- **XMS5-R800-W4G3B4C_base**：总体积变化 -93%
- **XMS5-R800-W4G3B4C_link1**：球数变化 -83%（30 → 5）
- **XMS5-R800-W4G3B4C_link1**：总体积变化 -58%
- **XMS5-R800-W4G3B4C_link2**：球数变化 -84%（31 → 5）
- **XMS5-R800-W4G3B4C_link3**：球数变化 -84%（31 → 5）
- **XMS5-R800-W4G3B4C_link4**：球数变化 -85%（34 → 5）
- **XMS5-R800-W4G3B4C_link4**：总体积变化 +117%
- **XMS5-R800-W4G3B4C_link5**：球数变化 -84%（31 → 5）
- **XMS5-R800-W4G3B4C_link5**：总体积变化 -72%
- **XMS5-R800-W4G3B4C_link6**：球数变化 -83%（30 → 5）

---

## 3. Candidate: candidate_morphit_5_inflated

| Link | 球数 | 最小半径 | 最大半径 | 平均半径 | bbox 对角线 | 体积 |
|------|------|----------|----------|----------|-------------|------|
| XMS5-R800-W4G3B4C_base | 2 | 0.0107m | 0.0107m | 0.0107m | 0.0726m | 0.000010m³ |
| XMS5-R800-W4G3B4C_link1 | 5 | 0.0633m | 0.0664m | 0.0647m | 0.2053m | 0.005675m³ |
| XMS5-R800-W4G3B4C_link2 | 5 | 0.0514m | 0.0915m | 0.0677m | 0.3941m | 0.007914m³ |
| XMS5-R800-W4G3B4C_link3 | 5 | 0.0666m | 0.0737m | 0.0697m | 0.1759m | 0.007126m³ |
| XMS5-R800-W4G3B4C_link4 | 5 | 0.0290m | 0.0778m | 0.0632m | 0.1704m | 0.006424m³ |
| XMS5-R800-W4G3B4C_link5 | 5 | 0.0522m | 0.0551m | 0.0544m | 0.0750m | 0.003370m³ |
| XMS5-R800-W4G3B4C_link6 | 5 | 0.0174m | 0.0206m | 0.0189m | 0.0778m | 0.000144m³ |

### candidate_morphit_5_inflated 检测到的异常

- **XMS5-R800-W4G3B4C_base**：球数过少（2），可能覆盖不足
- **XMS5-R800-W4G3B4C_link3**：最大球直径(0.1475m)占 bounding box 对角线(0.1759m)的 84%，可能过度膨胀
- **XMS5-R800-W4G3B4C_link4**：最大球直径(0.1555m)占 bounding box 对角线(0.1704m)的 91%，可能过度膨胀
- **XMS5-R800-W4G3B4C_link5**：最大球直径(0.1102m)占 bounding box 对角线(0.0750m)的 147%，可能过度膨胀

### candidate_morphit_5_inflated vs Baseline 差异

- **XMS5-R800-W4G3B4C_base**：球数变化 -60%（5 → 2）
- **XMS5-R800-W4G3B4C_base**：总体积变化 -60%
- **XMS5-R800-W4G3B4C_link1**：球数变化 -83%（30 → 5）
- **XMS5-R800-W4G3B4C_link1**：总体积变化 +143%
- **XMS5-R800-W4G3B4C_link2**：球数变化 -84%（31 → 5）
- **XMS5-R800-W4G3B4C_link2**：总体积变化 +659%
- **XMS5-R800-W4G3B4C_link3**：球数变化 -84%（31 → 5）
- **XMS5-R800-W4G3B4C_link3**：总体积变化 +585%
- **XMS5-R800-W4G3B4C_link4**：球数变化 -85%（34 → 5）
- **XMS5-R800-W4G3B4C_link4**：总体积变化 +1163%
- **XMS5-R800-W4G3B4C_link5**：球数变化 -84%（31 → 5）
- **XMS5-R800-W4G3B4C_link5**：总体积变化 +63%
- **XMS5-R800-W4G3B4C_link6**：球数变化 -83%（30 → 5）
- **XMS5-R800-W4G3B4C_link6**：总体积变化 +658%

---

## 总结

共检测到 8 个 link 存在潜在问题，详见上方分析。

建议关注：
- Baseline **XMS5-R800-W4G3B4C_link3**：球大小差异过大（max/min = 10.6），可能存在局部不均匀
- Baseline **XMS5-R800-W4G3B4C_link5**：球大小差异过大（max/min = 16.8），可能存在局部不均匀
- candidate_morphit_5 **XMS5-R800-W4G3B4C_base**：球数过少（2），可能覆盖不足
- candidate_morphit_5 **XMS5-R800-W4G3B4C_link5**：最大球直径(0.0612m)占 bounding box 对角线(0.0750m)的 82%，可能过度膨胀
- candidate_morphit_5_inflated **XMS5-R800-W4G3B4C_base**：球数过少（2），可能覆盖不足
- candidate_morphit_5_inflated **XMS5-R800-W4G3B4C_link3**：最大球直径(0.1475m)占 bounding box 对角线(0.1759m)的 84%，可能过度膨胀
- candidate_morphit_5_inflated **XMS5-R800-W4G3B4C_link4**：最大球直径(0.1555m)占 bounding box 对角线(0.1704m)的 91%，可能过度膨胀
- candidate_morphit_5_inflated **XMS5-R800-W4G3B4C_link5**：最大球直径(0.1102m)占 bounding box 对角线(0.0750m)的 147%，可能过度膨胀

---

### 后续步骤

1. 对报告中标记的异常 link，使用 Bubblify 可视化确认。
2. 如有明显漏包或过度膨胀，考虑调整 density 或局部重新拟合。
3. 通过几何复核的 candidate，进入 smoke test 和 stress test。