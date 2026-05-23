#!/usr/bin/env python3
"""通用离线规划入口。

使用方式：
  python scripts/plan_rokae_motion.py --config resource/config/examples/pose_plan_example.yaml
  python scripts/plan_rokae_motion.py --config resource/config/examples/joint_plan_example.yaml

支持命令行覆盖关键参数：
  python scripts/plan_rokae_motion.py --config config.yaml --mode joint_target --start-jp 0,0,0,0,0,0
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

# 确保 scripts/ 在 import 路径中
_SCRIPTS_DIR = Path(__file__).resolve().parent
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

from config_utils import PlanningConfig, load_config
from rokae_motion_gen import RokaeMotionGen
from rokae_world_utils import build_world


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ROKAE 离线规划工具")
    _SCRIPTS_DIR = Path(__file__).resolve().parent
    _DEFAULT_CONFIG = str(_SCRIPTS_DIR.parent / "resource" / "config" / "examples" / "pose_plan_example.yaml")
    p.add_argument("--config", default=_DEFAULT_CONFIG, help="输入配置 YAML 路径")
    p.add_argument("--mode", help="覆盖规划模式")
    p.add_argument("--start-jp", help="覆盖起始关节角（逗号分隔）")
    p.add_argument("--goal-pose", help="覆盖目标位姿（逗号分隔，x,y,z,qx,qy,qz,qw）")
    p.add_argument("--goal-jp", help="覆盖目标关节角（逗号分隔）")
    p.add_argument("--output-dir", default="/tmp/rokae_debug", help="覆盖输出目录")
    p.add_argument("--speed-scale", type=float, help="速度缩放 (0, 2.0]")
    p.add_argument("--hold-vec-weight", help="方向保持权重（逗号分隔，x,y,z）")
    p.add_argument("--approach-offset", type=float, help="接近偏移量（米）")
    p.add_argument("--approach-axis", default="z", help="接近轴（x/y/z，默认 z）")
    return p.parse_args()


def apply_overrides(cfg: PlanningConfig, args: argparse.Namespace) -> PlanningConfig:
    """用命令行参数覆盖配置。"""
    if getattr(args, "mode", None):
        cfg.mode = args.mode
    if getattr(args, "start_jp", None):
        cfg.start.joint_position = [float(x) for x in args.start_jp.split(",")]
    if getattr(args, "goal_pose", None):
        vals = [float(x) for x in args.goal_pose.split(",")]
        if len(vals) != 7:
            raise ValueError("--goal-pose 需要 7 个值: x,y,z,qx,qy,qz,qw")
        cfg.goal.pose = vals
        cfg.goal.joint_position = None
    if getattr(args, "goal_jp", None):
        vals = [float(x) for x in args.goal_jp.split(",")]
        cfg.goal.joint_position = vals
        cfg.goal.pose = None
    if getattr(args, "output_dir", None):
        cfg.output_dir = args.output_dir
    if getattr(args, "speed_scale", None) is not None:
        cfg.speed_scale = args.speed_scale
    if getattr(args, "hold_vec_weight", None):
        cfg.hold_vec_weight = [float(x) for x in args.hold_vec_weight.split(",")]
    if getattr(args, "approach_offset", None) is not None:
        cfg.approach_offset = args.approach_offset
    return cfg


def _apply_overrides(cfg: PlanningConfig, args: argparse.Namespace) -> PlanningConfig:
    return apply_overrides(cfg, args)


def _pose_xyzw_to_curobo(pose_xyzw: list[float]) -> list[float]:
    """将 [x,y,z,qx,qy,qz,qw] 转换为 CuRobo 的 [x,y,z,qw,qx,qy,qz]。"""
    x, y, z, qx, qy, qz, qw = pose_xyzw
    return [x, y, z, qw, qx, qy, qz]


def run(cfg: PlanningConfig, config_path: Path | None = None) -> dict:
    """执行规划并返回结果。

    Args:
        cfg: 规划配置。
        config_path: 输入配置文件路径（用于记录来源）。

    Returns:
        规划结果字典。
    """
    print(f"模式: {cfg.mode}")
    print(f"起始关节: {cfg.start.joint_position}")

    # 初始化规划器
    robot_path = Path(cfg.robot_config) if cfg.robot_config else None
    speed = cfg.speed_scale if cfg.speed_scale is not None else 1.0
    gen = RokaeMotionGen(robot_config_path=robot_path, speed_scale=speed)
    print(f"关节名: {gen.joint_names}")
    print(f"工具帧: {gen.tool_frames}")
    if speed != 1.0:
        print(f"速度缩放: {speed}")

    # 加载障碍物
    world_result = None
    if cfg.world.obstacle_json or cfg.world.obstacle_rel_json:
        abs_p = Path(cfg.world.obstacle_json) if cfg.world.obstacle_json else None
        rel_p = Path(cfg.world.obstacle_rel_json) if cfg.world.obstacle_rel_json else None
        world_result = build_world(abs_json_path=abs_p, rel_json_path=rel_p)
        gen.update_world_from_dict(world_result["world_dict"])
        print(f"障碍物: {world_result['world_summary']}")
    else:
        print("障碍物: 无")

    # 执行规划
    t0 = time.monotonic()

    if cfg.mode == "point_to_point":
        if not cfg.goal.pose:
            raise ValueError("point_to_point 模式需要 goal.pose")
        target = _pose_xyzw_to_curobo(cfg.goal.pose)
        result = gen.plan_single(
            cfg.start.joint_position, target,
            hold_vec_weight=cfg.hold_vec_weight,
        )

    elif cfg.mode == "joint_target":
        if not cfg.goal.joint_position:
            raise ValueError("joint_target 模式需要 goal.joint_position")
        result = gen.plan_single_js(cfg.start.joint_position, cfg.goal.joint_position)

    elif cfg.mode == "approach":
        if not cfg.goal.pose:
            raise ValueError("approach 模式需要 goal.pose")
        target = _pose_xyzw_to_curobo(cfg.goal.pose)
        approach_offset = cfg.approach_offset if cfg.approach_offset is not None else -0.1
        approach_axis = cfg.raw.get("approach_axis", "z")
        result = gen.plan_grasp_single(
            cfg.start.joint_position, target,
            approach_axis=approach_axis,
            approach_offset=approach_offset,
            plan_approach=True,
            plan_lift=False,
        )

    elif cfg.mode == "grasp":
        if not cfg.goal.pose:
            raise ValueError("grasp 模式需要 goal.pose")
        target = _pose_xyzw_to_curobo(cfg.goal.pose)
        approach_offset = cfg.approach_offset if cfg.approach_offset is not None else -0.15
        retract_offset = cfg.retract_offset if cfg.retract_offset is not None else -0.15
        approach_axis = cfg.raw.get("approach_axis", "z")
        lift_axis = cfg.raw.get("lift_axis", "z")
        result = gen.plan_grasp_single(
            cfg.start.joint_position, target,
            approach_axis=approach_axis,
            approach_offset=approach_offset,
            lift_axis=lift_axis,
            lift_offset=retract_offset,
            plan_approach=True,
            plan_lift=True,
        )

    else:
        raise NotImplementedError(
            f"模式 '{cfg.mode}' 暂未实现（当前支持 point_to_point、joint_target、approach、grasp）"
        )

    wall_time = time.monotonic() - t0
    result["wall_time_total"] = wall_time

    # 打印结果
    print(f"\n结果: success={result['success']}, status={result['status']}")
    if result["success"]:
        print(f"  路径点数: {result['waypoint_count']}")
        print(f"  求解时间: {result['solve_time']:.4f}s")
        print(f"  总耗时:   {wall_time:.4f}s")

    # 写入输出文件
    if cfg.output_dir:
        out = Path(cfg.output_dir)
        out.mkdir(parents=True, exist_ok=True)

        # summary.json
        summary = {
            "mode": cfg.mode,
            "success": result["success"],
            "status": result["status"],
            "robot_config": cfg.robot_config,
            "tool_frames": gen.tool_frames,
            "joint_names": gen.joint_names,
            "speed_scale": gen.speed_scale,
            "start_joint": cfg.start.joint_position,
            "goal_pose": cfg.goal.pose,
            "goal_joint": cfg.goal.joint_position,
            "hold_vec_weight": cfg.hold_vec_weight,
            "approach_offset": cfg.approach_offset,
            "retract_offset": cfg.retract_offset,
            "solve_time": result.get("solve_time"),
            "total_time": result.get("total_time"),
            "wall_time": wall_time,
            "interpolation_dt": result.get("interpolation_dt"),
            "waypoint_count": result.get("waypoint_count"),
            "input_config": str(config_path) if config_path else None,
        }
        (out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))

        # trajectory.json
        if result["success"]:
            traj = {
                "joint_names": result["joint_names"],
                "waypoints": result["trajectory_points"],
                "sample_period_s": result["interpolation_dt"],
            }
            (out / "trajectory.json").write_text(json.dumps(traj, indent=2, ensure_ascii=False))

        # world_summary.json
        if world_result is not None:
            ws = {
                "abs_json": cfg.world.obstacle_json,
                "rel_json": cfg.world.obstacle_rel_json,
                "summary": world_result["world_summary"],
                "obstacle_names": list(world_result["world_dict"].get("cuboid", {}).keys()),
            }
            (out / "world_summary.json").write_text(json.dumps(ws, indent=2, ensure_ascii=False))

        print(f"输出目录: {out}")

    return result


def collect_plan_artifact_paths(output_dir: Path) -> dict[str, str]:
    """返回规划阶段关键产物路径。"""
    return {
        "plan_output_dir": str(output_dir),
        "summary_json": str(output_dir / "summary.json"),
        "trajectory_json": str(output_dir / "trajectory.json"),
        "world_summary_json": str(output_dir / "world_summary.json"),
    }


def run_to_output_dir(cfg: PlanningConfig, config_path: Path | None = None) -> dict[str, Any]:
    """执行规划并返回结果与关键产物路径。"""
    result = run(cfg, config_path=config_path)
    output_dir = Path(cfg.output_dir) if cfg.output_dir else None
    artifact_paths = collect_plan_artifact_paths(output_dir) if output_dir is not None else {}
    return {
        "result": result,
        "plan_output_dir": str(output_dir) if output_dir is not None else None,
        "artifacts": artifact_paths,
    }


def main() -> None:
    args = _parse_args()
    config_path = Path(args.config)
    cfg = load_config(config_path)
    cfg = apply_overrides(cfg, args)
    run(cfg, config_path=config_path)


if __name__ == "__main__":
    main()
