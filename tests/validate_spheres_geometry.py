#!/usr/bin/env python3
"""碰撞球几何验证脚本 - 自动化 Bubblify 复核替代方案。

加载 spheres YAML 文件，检测异常球（超大球、漏包、过膨胀），
对比 baseline 与 candidate 的几何差异，生成结构化报告。

用法:
    python tests/validate_spheres_geometry.py \
      --baseline robot_assets/ROKAE/robot/spheres/ROKAE_SR5_0.9C_spherized.yml \
      --candidates doc/experiments/rokae_curoboV2_auto_spheres_bubblify_validation/candidate_density_0.4_pw10_cw1000.yml \
                   doc/experiments/rokae_curoboV2_auto_spheres_bubblify_validation/candidate_density_0.6_pw10_cw1000.yml \
      --output doc/experiments/rokae_curoboV2_auto_spheres_bubblify_validation/bubblify_review_report.md
"""

from __future__ import annotations

import argparse
import math
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


def load_spheres(path: Path) -> dict[str, list[dict]]:
    """加载 spheres YAML 文件，返回 {link_name: [{center, radius}, ...]}。"""
    with open(path) as f:
        data = yaml.safe_load(f)
    return data.get("collision_spheres", data)


def compute_link_stats(spheres: list[dict]) -> dict[str, Any]:
    """计算单个 link 的球体统计信息。"""
    if not spheres:
        return {"count": 0}

    radii = [s["radius"] for s in spheres]
    centers = [s["center"] for s in spheres]

    # 计算 bounding box
    xs = [c[0] for c in centers]
    ys = [c[1] for c in centers]
    zs = [c[2] for c in centers]

    bbox_x = max(xs) - min(xs)
    bbox_y = max(ys) - min(ys)
    bbox_z = max(zs) - min(zs)
    bbox_diag = math.sqrt(bbox_x**2 + bbox_y**2 + bbox_z**2)

    # 球体总体积
    total_vol = sum(4 / 3 * math.pi * r**3 for r in radii)

    # 最大球直径与 bounding box 对角线的比值
    max_diameter = 2 * max(radii)
    diameter_to_bbox = max_diameter / bbox_diag if bbox_diag > 0 else float("inf")

    return {
        "count": len(spheres),
        "radius_min": min(radii),
        "radius_max": max(radii),
        "radius_mean": sum(radii) / len(radii),
        "radius_median": sorted(radii)[len(radii) // 2],
        "bbox": (bbox_x, bbox_y, bbox_z),
        "bbox_diag": bbox_diag,
        "total_volume": total_vol,
        "diameter_to_bbox_ratio": diameter_to_bbox,
    }


def detect_anomalies(
    link_name: str, stats: dict[str, Any], all_stats: dict[str, dict]
) -> list[str]:
    """检测单个 link 的几何异常。"""
    issues = []

    if stats["count"] == 0:
        issues.append("无碰撞球覆盖")
        return issues

    # 检查球数过少
    if stats["count"] < 3:
        issues.append(f"球数过少（{stats['count']}），可能覆盖不足")

    # 检查最大球是否过大（直径超过 bounding box 对角线的 80%）
    if stats["diameter_to_bbox_ratio"] > 0.8:
        issues.append(
            f"最大球直径({stats['radius_max']*2:.4f}m)占 bounding box 对角线"
            f"({stats['bbox_diag']:.4f}m)的 {stats['diameter_to_bbox_ratio']:.0%}，"
            f"可能过度膨胀"
        )

    # 检查最大球半径是否异常（超过所有 link 平均值的 3 倍）
    all_max_radii = [
        s["radius_max"] for s in all_stats.values() if s.get("count", 0) > 0
    ]
    if all_max_radii:
        global_mean_max = sum(all_max_radii) / len(all_max_radii)
        if stats["radius_max"] > global_mean_max * 3:
            issues.append(
                f"最大球半径({stats['radius_max']:.4f}m)远超全局平均"
                f"({global_mean_max:.4f}m)，可能属于异常大球"
            )

    # 检查球大小差异是否过大（最大/最小 > 10）
    if stats["radius_min"] > 0:
        ratio = stats["radius_max"] / stats["radius_min"]
        if ratio > 10:
            issues.append(
                f"球大小差异过大（max/min = {ratio:.1f}），可能存在局部不均匀"
            )

    return issues


def compare_link_stats(
    baseline_stats: dict[str, Any], candidate_stats: dict[str, Any]
) -> list[str]:
    """对比 baseline 与 candidate 的单 link 差异。"""
    diffs = []

    b_count = baseline_stats.get("count", 0)
    c_count = candidate_stats.get("count", 0)
    if b_count > 0 and c_count > 0:
        count_change = (c_count - b_count) / b_count
        if abs(count_change) > 0.5:
            diffs.append(
                f"球数变化 {count_change:+.0%}（{b_count} → {c_count}）"
            )

    b_vol = baseline_stats.get("total_volume", 0)
    c_vol = candidate_stats.get("total_volume", 0)
    if b_vol > 0 and c_vol > 0:
        vol_change = (c_vol - b_vol) / b_vol
        if abs(vol_change) > 0.5:
            diffs.append(
                f"总体积变化 {vol_change:+.0%}"
            )

    return diffs


def generate_report(
    baseline_path: Path,
    candidate_paths: list[Path],
    output_path: Path,
) -> None:
    """生成完整的几何验证报告。"""
    baseline_spheres = load_spheres(baseline_path)
    baseline_stats = {
        link: compute_link_stats(spheres)
        for link, spheres in baseline_spheres.items()
    }

    candidate_data = []
    for cp in candidate_paths:
        spheres = load_spheres(cp)
        stats = {
            link: compute_link_stats(spheres)
            for link, spheres in spheres.items()
        }
        candidate_data.append({"path": cp, "spheres": spheres, "stats": stats})

    # 检测 baseline 异常
    baseline_anomalies = {}
    for link, stats in baseline_stats.items():
        issues = detect_anomalies(link, stats, baseline_stats)
        if issues:
            baseline_anomalies[link] = issues

    # 检测 candidate 异常
    candidate_anomalies = {}
    for cand in candidate_data:
        cand_anom = {}
        for link, stats in cand["stats"].items():
            issues = detect_anomalies(link, stats, cand["stats"])
            if issues:
                cand_anom[link] = issues
        candidate_anomalies[cand["path"].stem] = cand_anom

    # 对比差异
    comparisons = {}
    for cand in candidate_data:
        comp = {}
        all_links = set(baseline_stats.keys()) | set(cand["stats"].keys())
        for link in sorted(all_links):
            b = baseline_stats.get(link, {})
            c = cand["stats"].get(link, {})
            diffs = compare_link_stats(b, c)
            if diffs:
                comp[link] = diffs
        comparisons[cand["path"].stem] = comp

    # 生成 Markdown 报告
    now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    lines = [
        "# Bubblify 几何复核报告（自动化验证）",
        "",
        f"- 生成时间：{now}",
        f"- Baseline：`{baseline_path}`",
        f"- 候选文件：{', '.join(f'`{cp}`' for cp in candidate_paths)}",
        "",
        "> 本报告由 `tests/validate_spheres_geometry.py` 自动生成，",
        "> 作为 Bubblify 交互式可视化复核的程序化补充。",
        "> 最终几何结论仍建议结合 Bubblify 可视化确认。",
        "",
        "---",
        "",
    ]

    # Baseline 概览
    lines.append("## 1. Baseline 几何概览")
    lines.append("")
    lines.append(
        "| Link | 球数 | 最小半径 | 最大半径 | 平均半径 | bbox 对角线 | 体积 |"
    )
    lines.append(
        "|------|------|----------|----------|----------|-------------|------|"
    )
    for link in sorted(baseline_stats.keys()):
        s = baseline_stats[link]
        if s["count"] == 0:
            lines.append(f"| {link} | 0 | - | - | - | - | - |")
        else:
            lines.append(
                f"| {link} | {s['count']} "
                f"| {s['radius_min']:.4f}m "
                f"| {s['radius_max']:.4f}m "
                f"| {s['radius_mean']:.4f}m "
                f"| {s['bbox_diag']:.4f}m "
                f"| {s['total_volume']:.6f}m³ |"
            )
    lines.append("")

    # Baseline 异常
    if baseline_anomalies:
        lines.append("### Baseline 检测到的异常")
        lines.append("")
        for link, issues in baseline_anomalies.items():
            for issue in issues:
                lines.append(f"- **{link}**：{issue}")
        lines.append("")
    else:
        lines.append("### Baseline 未检测到明显异常")
        lines.append("")
    lines.append("---")
    lines.append("")

    # Candidate 详细分析
    for i, cand in enumerate(candidate_data):
        name = cand["path"].stem
        lines.append(f"## {i + 2}. Candidate: {name}")
        lines.append("")
        lines.append(
            "| Link | 球数 | 最小半径 | 最大半径 | 平均半径 | bbox 对角线 | 体积 |"
        )
        lines.append(
            "|------|------|----------|----------|----------|-------------|------|"
        )
        for link in sorted(cand["stats"].keys()):
            s = cand["stats"][link]
            if s["count"] == 0:
                lines.append(f"| {link} | 0 | - | - | - | - | - |")
            else:
                lines.append(
                    f"| {link} | {s['count']} "
                    f"| {s['radius_min']:.4f}m "
                    f"| {s['radius_max']:.4f}m "
                    f"| {s['radius_mean']:.4f}m "
                    f"| {s['bbox_diag']:.4f}m "
                    f"| {s['total_volume']:.6f}m³ |"
                )
        lines.append("")

        # 异常
        anom = candidate_anomalies.get(name, {})
        if anom:
            lines.append(f"### {name} 检测到的异常")
            lines.append("")
            for link, issues in anom.items():
                for issue in issues:
                    lines.append(f"- **{link}**：{issue}")
            lines.append("")
        else:
            lines.append(f"### {name} 未检测到明显异常")
            lines.append("")

        # 与 baseline 对比
        comp = comparisons.get(name, {})
        if comp:
            lines.append(f"### {name} vs Baseline 差异")
            lines.append("")
            for link, diffs in comp.items():
                for diff in diffs:
                    lines.append(f"- **{link}**：{diff}")
            lines.append("")
        else:
            lines.append(f"### {name} vs Baseline 无显著差异")
            lines.append("")

        lines.append("---")
        lines.append("")

    # 总结
    lines.append("## 总结")
    lines.append("")

    total_anomalies = len(baseline_anomalies)
    for name, anom in candidate_anomalies.items():
        total_anomalies += len(anom)

    if total_anomalies == 0:
        lines.append("所有文件均未检测到明显几何异常，球体分布均匀，无超大球或漏包问题。")
    else:
        lines.append(f"共检测到 {total_anomalies} 个 link 存在潜在问题，详见上方分析。")
        lines.append("")
        lines.append("建议关注：")
        for link, issues in baseline_anomalies.items():
            for issue in issues:
                lines.append(f"- Baseline **{link}**：{issue}")
        for name, anom in candidate_anomalies.items():
            for link, issues in anom.items():
                for issue in issues:
                    lines.append(f"- {name} **{link}**：{issue}")

    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("### 后续步骤")
    lines.append("")
    lines.append("1. 对报告中标记的异常 link，使用 Bubblify 可视化确认。")
    lines.append("2. 如有明显漏包或过度膨胀，考虑调整 density 或局部重新拟合。")
    lines.append("3. 通过几何复核的 candidate，进入 smoke test 和 stress test。")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"报告已保存到: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="碰撞球几何验证")
    parser.add_argument(
        "--baseline",
        type=Path,
        required=True,
        help="Baseline spheres YAML 文件路径",
    )
    parser.add_argument(
        "--candidates",
        type=Path,
        nargs="+",
        required=True,
        help="Candidate spheres YAML 文件路径",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("doc/experiments/bubblify_review_report.md"),
        help="输出报告路径",
    )
    args = parser.parse_args()

    generate_report(args.baseline, args.candidates, args.output)


if __name__ == "__main__":
    main()
