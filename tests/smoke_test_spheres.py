#!/usr/bin/env python3
"""碰撞球 Smoke Test — 验证自动生成模式与文件加载模式的最小功能可用性。

测试内容：
1. 自动生成模式（density=0.6）+ 点到点规划
2. 文件加载模式（candidate_density_0.6_pw10_cw1000.yml）+ 点到点规划
3. 自动生成模式（density=0.6）+ 关节目标规划

判定标准：
- 规划返回 success=True
- 输出 trajectory.json 存在且包含 waypoints

用法：
    python tests/smoke_test_spheres.py
    python tests/smoke_test_spheres.py --candidate doc/experiments/.../candidate_density_0.6.yml
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

# 项目根目录
WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE_ROOT))


@dataclass
class SmokeResult:
    """单个 smoke test 的结果。"""
    name: str
    mode: str
    spheres_source: str
    success: bool
    waypoint_count: int = 0
    solve_time_s: float = 0.0
    error: str = ""
    trajectory_path: str = ""


def _make_temp_robot_config(spheres_file: str) -> Path:
    """创建临时 robot config YAML，将 collision_spheres 指向指定文件。

    文件加载模式下，需要通过 robot config 中的 collision_spheres 字段指定球文件路径。
    此函数复制默认 robot config 并修改 collision_spheres 为绝对路径。
    """
    import yaml

    default_config = WORKSPACE_ROOT / "robot_assets" / "ROKAE" / "robot" / "xms5_r800_w4g3b4c_robot.yml"
    with open(default_config) as f:
        cfg_data = yaml.safe_load(f)

    # 将 collision_spheres 设为候选文件的绝对路径
    spheres_abs = str(Path(spheres_file).resolve())
    cfg_data["robot_cfg"]["kinematics"]["collision_spheres"] = spheres_abs

    # 将 urdf_path 和 asset_root_path 也设为绝对路径，避免临时文件在 /tmp/ 下解析失败
    robot_dir = WORKSPACE_ROOT / "robot_assets" / "ROKAE" / "robot"
    kin = cfg_data["robot_cfg"]["kinematics"]
    if "urdf_path" in kin:
        urdf_rel = kin["urdf_path"]
        kin["urdf_path"] = str((robot_dir / urdf_rel).resolve())
    if "asset_root_path" in kin:
        asset_rel = kin["asset_root_path"]
        kin["asset_root_path"] = str((robot_dir / asset_rel).resolve())

    tmp_path = Path(tempfile.mktemp(suffix=".yml", prefix="robot_cfg_smoke_"))
    with open(tmp_path, "w") as f:
        yaml.dump(cfg_data, f, default_flow_style=False, allow_unicode=True)
    return tmp_path


def run_single_test(
    name: str,
    plan_mode: str,
    auto_generate: bool,
    sphere_density: float,
    spheres_file: str | None,
    start_jp: list[float],
    goal,
    output_dir: Path,
) -> SmokeResult:
    """运行单个规划测试。"""
    from scripts.config_utils import PlanningConfig

    cfg = PlanningConfig()
    cfg.mode = plan_mode
    cfg.start.joint_position = start_jp
    if plan_mode == "joint_target":
        cfg.goal.joint_position = goal
        cfg.goal.pose = None
    else:
        cfg.goal.pose = goal
        cfg.goal.joint_position = None
    cfg.output_dir = str(output_dir)
    cfg.auto_generate_spheres = auto_generate
    cfg.sphere_density = sphere_density

    tmp_robot_cfg = None
    # 文件加载模式：创建临时 robot config 指向候选球文件
    if not auto_generate and spheres_file:
        tmp_robot_cfg = _make_temp_robot_config(spheres_file)
        cfg.robot_config = str(tmp_robot_cfg)

    spheres_src = (
        f"auto_generate(density={sphere_density})"
        if auto_generate
        else f"file_load({Path(spheres_file).name})"
    )

    try:
        from scripts.plan_rokae_motion import run

        t0 = time.time()
        result = run(cfg)
        elapsed = time.time() - t0

        success = result.get("success", False)
        wp_count = result.get("waypoint_count", 0)

        # 检查 trajectory.json
        traj_path = output_dir / "trajectory.json"
        traj_exists = traj_path.exists()
        traj_valid = False
        if traj_exists:
            with open(traj_path) as f:
                traj_data = json.load(f)
                traj_valid = len(traj_data.get("waypoints", [])) > 0

        return SmokeResult(
            name=name,
            mode=plan_mode,
            spheres_source=spheres_src,
            success=success and traj_valid,
            waypoint_count=wp_count,
            solve_time_s=elapsed,
            error="" if success else result.get("error", "unknown"),
            trajectory_path=str(traj_path) if traj_exists else "",
        )
    except Exception as e:
        return SmokeResult(
            name=name,
            mode=plan_mode,
            spheres_source=spheres_src,
            success=False,
            error=str(e),
        )
    finally:
        if tmp_robot_cfg and tmp_robot_cfg.exists():
            tmp_robot_cfg.unlink(missing_ok=True)


def generate_report(results: list[SmokeResult], output_path: Path) -> None:
    """生成 smoke test 报告。"""
    from datetime import datetime, timezone

    now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    lines = [
        "# Smoke Test 报告",
        "",
        f"- 生成时间：{now}",
        f"- 测试目的：验证自动生成模式与文件加载模式在主规划链路中的最小功能可用性",
        "",
        "---",
        "",
        "## 测试结果总览",
        "",
        "| 测试项 | 规划模式 | 球来源 | 结果 | 路径点数 | 耗时 |",
        "|--------|----------|--------|------|----------|------|",
    ]

    for r in results:
        status = "PASS" if r.success else "FAIL"
        wp = str(r.waypoint_count) if r.success else "-"
        t = f"{r.solve_time_s:.1f}s" if r.success else "-"
        lines.append(f"| {r.name} | {r.mode} | {r.spheres_source} | {status} | {wp} | {t} |")

    lines.append("")

    # 详细信息
    for r in results:
        lines.append(f"## {r.name}")
        lines.append("")
        lines.append(f"- 规划模式：{r.mode}")
        lines.append(f"- 球来源：{r.spheres_source}")
        lines.append(f"- 结果：{'PASS' if r.success else 'FAIL'}")
        if r.success:
            lines.append(f"- 路径点数：{r.waypoint_count}")
            lines.append(f"- 耗时：{r.solve_time_s:.1f}s")
            lines.append(f"- 轨迹文件：`{r.trajectory_path}`")
        else:
            lines.append(f"- 错误：{r.error}")
        lines.append("")

    # 结论
    lines.append("---")
    lines.append("")
    lines.append("## 结论")
    lines.append("")

    passed = sum(1 for r in results if r.success)
    total = len(results)

    if passed == total:
        lines.append(f"**全部 {total} 项测试通过。**")
        lines.append("")
        lines.append("自动生成模式与文件加载模式均可正常完成规划，碰撞球验证无回归。")
    elif passed > 0:
        lines.append(f"**{passed}/{total} 项测试通过。**")
        lines.append("")
        lines.append("失败的测试项需要排查：")
        for r in results:
            if not r.success:
                lines.append(f"- {r.name}：{r.error}")
    else:
        lines.append(f"**全部 {total} 项测试失败。**")
        lines.append("")
        lines.append("需要排查规划链路是否正常工作。")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"\n报告已保存到: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="碰撞球 Smoke Test")
    parser.add_argument(
        "--candidate",
        type=Path,
        default=WORKSPACE_ROOT
        / "doc/experiments/rokae_curoboV2_auto_spheres_bubblify_validation/candidate_density_0.6_pw10_cw1000.yml",
        help="Candidate spheres 文件路径",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=WORKSPACE_ROOT
        / "doc/experiments/rokae_curoboV2_auto_spheres_bubblify_validation/smoke_test_report.md",
        help="输出报告路径",
    )
    args = parser.parse_args()

    candidate_path = args.candidate.resolve()
    if not candidate_path.exists():
        print(f"错误：candidate 文件不存在: {candidate_path}")
        sys.exit(1)

    # 标准测试位姿
    start_jp = [-1.571, 1.571, 0.0, 1.571, 1.571, 0.0]
    goal_pose = [0.45, 0.0, 0.85, 0.0, 0.707, 0.0, 0.707]
    goal_jp = [-1.0, 1.2, 0.3, 1.0, 1.5, 0.5]

    results = []

    # 测试 1：自动生成模式 + 点到点规划
    print("=" * 60)
    print("测试 1/3：自动生成模式（density=0.6）+ 点到点规划")
    print("=" * 60)
    out1 = Path(tempfile.mkdtemp(prefix="smoke_auto_p2p_"))
    r1 = run_single_test(
        name="自动生成 + 点到点",
        plan_mode="point_to_point",
        auto_generate=True,
        sphere_density=0.6,
        spheres_file=None,
        start_jp=start_jp,
        goal=goal_pose,
        output_dir=out1,
    )
    results.append(r1)
    print(f"结果: {'PASS' if r1.success else 'FAIL'}")

    # 测试 2：文件加载模式 + 点到点规划
    print("\n" + "=" * 60)
    print("测试 2/3：文件加载模式（candidate_density_0.6_pw10_cw1000）+ 点到点规划")
    print("=" * 60)
    out2 = Path(tempfile.mkdtemp(prefix="smoke_file_p2p_"))
    r2 = run_single_test(
        name="文件加载 + 点到点",
        plan_mode="point_to_point",
        auto_generate=False,
        sphere_density=0.3,
        spheres_file=str(candidate_path),
        start_jp=start_jp,
        goal=goal_pose,
        output_dir=out2,
    )
    results.append(r2)
    print(f"结果: {'PASS' if r2.success else 'FAIL'}")

    # 测试 3：自动生成模式 + 关节目标规划
    print("\n" + "=" * 60)
    print("测试 3/3：自动生成模式（density=0.6）+ 关节目标规划")
    print("=" * 60)
    out3 = Path(tempfile.mkdtemp(prefix="smoke_auto_joint_"))
    r3 = run_single_test(
        name="自动生成 + 关节目标",
        plan_mode="joint_target",
        auto_generate=True,
        sphere_density=0.6,
        spheres_file=None,
        start_jp=start_jp,
        goal=goal_jp,
        output_dir=out3,
    )
    results.append(r3)
    print(f"结果: {'PASS' if r3.success else 'FAIL'}")

    # 生成报告
    generate_report(results, args.output)

    # 清理临时目录
    for d in [out1, out2, out3]:
        shutil.rmtree(d, ignore_errors=True)

    # 返回码
    passed = sum(1 for r in results if r.success)
    sys.exit(0 if passed == len(results) else 1)


if __name__ == "__main__":
    main()
