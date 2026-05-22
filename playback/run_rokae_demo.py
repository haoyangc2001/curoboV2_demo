#!/usr/bin/env python3
"""大花复合末端一键规划与 MuJoCo 回放入口。

本文件把规划、合同导出、离屏回放和可选实时回放串起来，
用于快速执行完整阶段一演示链路。
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import mujoco
import mujoco.viewer

from export_rokae_playback_contract import build_contract_from_plan_output, export_contract
from replay_rokae_mujoco import (
    _load_contract,
    _resolve_qpos_addresses,
    generate_rokae_stage1_mjcf,
    replay_contract,
)


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]


def _realtime_replay(
    contract_path: Path,
    output_dir: Path,
    *,
    playback_speed: float,
    final_hold_s: float,
    shutdown_wait_s: float = 5.0,
) -> dict[str, Any]:
    """在 MuJoCo viewer 中按真实时间节奏回放合同。

    Args:
        contract_path: 回放合同路径。
        output_dir: 实时回放摘要输出目录。
        playback_speed: 播放速度倍率。
        final_hold_s: 最后一帧停留时长，单位秒。
        shutdown_wait_s: 关闭 viewer 后等待资源释放的最长时间，单位秒。

    Returns:
        包含是否完整播放、耗时和关闭状态的摘要字典。
    """
    contract = _load_contract(contract_path)
    joint_names = list(contract["joint_contract"]["mujoco_expected_joint_names"])
    waypoints = list(contract["trajectory_contract"]["waypoints"])
    dt = float(contract["timing_contract"]["sample_period_s"])
    urdf_path = Path(contract["robot_source"]["urdf_path"])

    model_xml_path = generate_rokae_stage1_mjcf(
        output_dir / "rokae_stage1_realtime.xml",
        urdf_path=urdf_path,
    )
    model = mujoco.MjModel.from_xml_path(str(model_xml_path))
    data = mujoco.MjData(model)
    qpos_mapping = _resolve_qpos_addresses(model, joint_names)

    handle = mujoco.viewer.launch_passive(
        model,
        data,
        show_left_ui=False,
        show_right_ui=False,
    )
    sim_ref = handle._sim

    completed_all_waypoints = True
    start_time = time.perf_counter()
    rendered_waypoints = 0

    try:
        for waypoint in waypoints:
            if not handle.is_running():
                completed_all_waypoints = False
                break

            with handle.lock():
                for target, mapping in zip(waypoint, qpos_mapping):
                    data.qpos[mapping["qpos_adr"]] = target
                mujoco.mj_forward(model, data)

            handle.sync()
            rendered_waypoints += 1
            time.sleep(max(dt / playback_speed, 0.0))

        hold_until = time.perf_counter() + max(final_hold_s, 0.0)
        while handle.is_running() and time.perf_counter() < hold_until:
            handle.sync()
            time.sleep(min(dt / max(playback_speed, 1e-6), 0.05))
    finally:
        elapsed = time.perf_counter() - start_time
        handle.close()
        shutdown_deadline = time.perf_counter() + max(shutdown_wait_s, 0.0)
        while sim_ref() is not None and time.perf_counter() < shutdown_deadline:
            time.sleep(0.01)

    summary = {
        "success": completed_all_waypoints,
        "contract_path": str(contract_path),
        "urdf_path": str(urdf_path),
        "model_xml_path": str(model_xml_path),
        "playback_speed": playback_speed,
        "requested_waypoints": len(waypoints),
        "rendered_waypoints": rendered_waypoints,
        "sample_period_s": dt,
        "final_hold_s": final_hold_s,
        "elapsed_wall_time_s": elapsed,
        "viewer_shutdown_wait_s": shutdown_wait_s,
        "viewer_shutdown_completed": sim_ref() is None,
        "stopped_early": not completed_all_waypoints,
    }
    (output_dir / "realtime_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )
    return summary


def run_all(
    output_root: Path,
    render_every: int,
    goal_delta_xyz: tuple[float, float, float] = (0.12, 0.0, 0.05),
) -> dict[str, Any]:
    """执行完整的大花复合末端离屏演示流程（legacy 模式）。

    Args:
        output_root: 本次运行的根输出目录。
        render_every: 离屏回放渲染步长。
        goal_delta_xyz: 规划目标相对位移。

    Returns:
        汇总规划、合同与回放关键结果的运行摘要字典。
    """
    plan_dir = output_root / "plan"
    contract_dir = output_root / "contract"
    playback_dir = output_root / "playback"
    plan_dir.mkdir(parents=True, exist_ok=True)
    contract_dir.mkdir(parents=True, exist_ok=True)
    playback_dir.mkdir(parents=True, exist_ok=True)

    contract = export_contract(
        contract_dir,
        plan_output_dir=plan_dir,
        goal_delta_xyz=goal_delta_xyz,
    )
    contract_path = contract_dir / "playback_contract.json"
    playback_summary = replay_contract(contract_path, playback_dir, render_every)

    summary = {
        "success": bool(playback_summary["success"]),
        "contract_path": str(contract_path),
        "plan_summary_path": str(plan_dir / "summary.json"),
        "playback_summary_path": str(playback_dir / "playback_summary.json"),
        "contract_waypoint_count": contract["timing_contract"]["sample_count"],
        "contract_sample_period_s": contract["timing_contract"]["sample_period_s"],
        "goal_delta_xyz": list(goal_delta_xyz),
        "offscreen_rendered_frames": playback_summary["render_summary"]["frame_count"],
        "ee_displacement": playback_summary["render_summary"]["ee_displacement"],
        "ee_direction_alignment": playback_summary["render_summary"]["ee_direction_alignment"],
        "checks": playback_summary["checks"],
    }
    (output_root / "run_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )
    return summary


def run_from_plan_output(
    plan_output_dir: Path,
    output_root: Path,
    render_every: int,
) -> dict[str, Any]:
    """从已有规划输出执行 MuJoCo 回放流程。

    Args:
        plan_output_dir: plan_rokae_motion.py 的输出目录（含 summary.json 和 trajectory.json）。
        output_root: 本次运行的根输出目录。
        render_every: 离屏回放渲染步长。

    Returns:
        汇总合同与回放关键结果的运行摘要字典。
    """
    contract_dir = output_root / "contract"
    playback_dir = output_root / "playback"
    contract_dir.mkdir(parents=True, exist_ok=True)
    playback_dir.mkdir(parents=True, exist_ok=True)

    contract = build_contract_from_plan_output(plan_output_dir, contract_dir)
    contract_path = contract_dir / "playback_contract.json"
    playback_summary = replay_contract(contract_path, playback_dir, render_every)

    summary = {
        "success": bool(playback_summary["success"]),
        "contract_path": str(contract_path),
        "plan_output_dir": str(plan_output_dir),
        "playback_summary_path": str(playback_dir / "playback_summary.json"),
        "contract_waypoint_count": contract["timing_contract"]["sample_count"],
        "contract_sample_period_s": contract["timing_contract"]["sample_period_s"],
        "offscreen_rendered_frames": playback_summary["render_summary"]["frame_count"],
        "ee_displacement": playback_summary["render_summary"]["ee_displacement"],
        "checks": playback_summary["checks"],
    }
    (output_root / "run_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )
    return summary


def main() -> None:
    """命令行入口。

    支持两种模式：
    --plan-output-dir 从已有规划输出构建合同并回放（推荐）
    --legacy 执行在线规划后构建合同并回放（兼容）

    Returns:
        无返回值；成功时打印一键流程的核心产物路径。
    """
    parser = argparse.ArgumentParser(
        description="One-click ROKAE planning plus MuJoCo playback"
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help="Optional root directory for this run; defaults to evidence/s6_playback/<timestamp>",
    )
    parser.add_argument(
        "--plan-output-dir",
        type=Path,
        default=None,
        help="Directory containing summary.json and trajectory.json from plan_rokae_motion.py",
    )
    parser.add_argument(
        "--render-every",
        type=int,
        default=1,
        help="Offscreen replay renders every Nth waypoint while always keeping the final frame",
    )
    parser.add_argument("--goal-dx", type=float, default=0.12, help="Goal offset in x (m, legacy mode only)")
    parser.add_argument("--goal-dy", type=float, default=0.0, help="Goal offset in y (m, legacy mode only)")
    parser.add_argument("--goal-dz", type=float, default=0.05, help="Goal offset in z (m, legacy mode only)")
    parser.add_argument(
        "--playback-speed",
        type=float,
        default=1.0,
        help="Realtime viewer speed multiplier; 1.0 means one contract sample every contract dt",
    )
    parser.add_argument(
        "--final-hold-s",
        type=float,
        default=1.0,
        help="How long to keep the final realtime pose on screen before closing the viewer",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="Skip the realtime MuJoCo viewer and only generate offscreen evidence",
    )
    parser.add_argument(
        "--legacy",
        action="store_true",
        help="Use legacy mode: run demo_plan_pose_rokae.run_demo() for online planning",
    )
    args = parser.parse_args()

    if args.render_every <= 0:
        raise SystemExit("--render-every must be positive")
    if args.playback_speed <= 0.0:
        raise SystemExit("--playback-speed must be positive")

    if args.output_root is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_root = WORKSPACE_ROOT / "evidence" / "s6_playback" / stamp
    else:
        output_root = args.output_root

    if args.legacy:
        goal_delta_xyz = (args.goal_dx, args.goal_dy, args.goal_dz)
        summary = run_all(output_root, args.render_every, goal_delta_xyz=goal_delta_xyz)
    else:
        if args.plan_output_dir is None:
            parser.error("--plan-output-dir is required (or use --legacy)")
        summary = run_from_plan_output(args.plan_output_dir, output_root, args.render_every)

    realtime_summary = None
    if not args.no_viewer:
        realtime_dir = output_root / "realtime"
        realtime_dir.mkdir(parents=True, exist_ok=True)
        realtime_summary = _realtime_replay(
            Path(summary["contract_path"]),
            realtime_dir,
            playback_speed=args.playback_speed,
            final_hold_s=args.final_hold_s,
        )

    summary["success"] = bool(summary["success"] and (realtime_summary is None or realtime_summary["success"]))
    summary["viewer_enabled"] = not args.no_viewer
    summary["playback_speed"] = args.playback_speed
    summary["realtime_summary_path"] = (
        str(output_root / "realtime" / "realtime_summary.json") if realtime_summary else None
    )
    (output_root / "run_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )

    print("One-click ROKAE demo completed")
    print(f"Output root: {output_root}")
    print(f"Contract: {output_root / 'contract' / 'playback_contract.json'}")
    print(f"Playback GIF: {output_root / 'playback' / 'playback.gif'}")
    if "goal_delta_xyz" in summary:
        print(f"Goal delta xyz: {summary['goal_delta_xyz']}")
    if "ee_direction_alignment" in summary:
        print(f"Direction alignment: {summary['ee_direction_alignment']:.6f}")
    if realtime_summary is not None:
        print(f"Realtime summary: {output_root / 'realtime' / 'realtime_summary.json'}")


if __name__ == "__main__":
    main()
