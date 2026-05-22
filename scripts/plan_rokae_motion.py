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
    p.add_argument("--config", required=True, help="输入配置 YAML 路径")
    p.add_argument("--mode", help="覆盖规划模式")
    p.add_argument("--start-jp", help="覆盖起始关节角（逗号分隔）")
    p.add_argument("--goal-pose", help="覆盖目标位姿（逗号分隔，x,y,z,qx,qy,qz,qw）")
    p.add_argument("--goal-jp", help="覆盖目标关节角（逗号分隔）")
    p.add_argument("--output-dir", help="覆盖输出目录")
    return p.parse_args()


def _apply_overrides(cfg: PlanningConfig, args: argparse.Namespace) -> PlanningConfig:
    """用命令行参数覆盖配置。"""
    if args.mode:
        cfg.mode = args.mode
    if args.start_jp:
        cfg.start.joint_position = [float(x) for x in args.start_jp.split(",")]
    if args.goal_pose:
        vals = [float(x) for x in args.goal_pose.split(",")]
        if len(vals) != 7:
            raise ValueError("--goal-pose 需要 7 个值: x,y,z,qx,qy,qz,qw")
        cfg.goal.pose = vals
        cfg.goal.joint_position = None
    if args.goal_jp:
        vals = [float(x) for x in args.goal_jp.split(",")]
        cfg.goal.joint_position = vals
        cfg.goal.pose = None
    if args.output_dir:
        cfg.output_dir = args.output_dir
    return cfg


def _pose_xyzw_to_curobo(pose_xyzw: list[float]) -> list[float]:
    """将 [x,y,z,qx,qy,qz,qw] 转换为 CuRobo 的 [x,y,z,qw,qx,qy,qz]。"""
    x, y, z, qx, qy, qz, qw = pose_xyzw
    return [x, y, z, qw, qx, qy, qz]


def run(cfg: PlanningConfig) -> dict:
    """执行规划并返回结果。"""
    print(f"模式: {cfg.mode}")
    print(f"起始关节: {cfg.start.joint_position}")

    # 初始化规划器
    robot_path = Path(cfg.robot_config) if cfg.robot_config else None
    gen = RokaeMotionGen(robot_config_path=robot_path)
    print(f"关节名: {gen.joint_names}")
    print(f"工具帧: {gen.tool_frames}")

    # 加载障碍物
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
        result = gen.plan_single(cfg.start.joint_position, target)

    elif cfg.mode == "joint_target":
        if not cfg.goal.joint_position:
            raise ValueError("joint_target 模式需要 goal.joint_position")
        result = gen.plan_single_js(cfg.start.joint_position, cfg.goal.joint_position)

    else:
        raise NotImplementedError(f"模式 '{cfg.mode}' 暂未实现（当前支持 point_to_point、joint_target）")

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
            "start_joint": cfg.start.joint_position,
            "goal_pose": cfg.goal.pose,
            "goal_joint": cfg.goal.joint_position,
            "solve_time": result.get("solve_time"),
            "interpolation_dt": result.get("interpolation_dt"),
            "waypoint_count": result.get("waypoint_count"),
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

        print(f"输出目录: {out}")

    return result


def main() -> None:
    args = _parse_args()
    cfg = load_config(Path(args.config))
    cfg = _apply_overrides(cfg, args)
    run(cfg)


if __name__ == "__main__":
    main()
