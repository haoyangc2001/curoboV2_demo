#!/usr/bin/env python3
"""Minimal standalone pose-planning demo for CuRobo2 stage 1."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import torch

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
LOCAL_CUROBO_ROOT = WORKSPACE_ROOT / "third_party" / "curobo"
if str(LOCAL_CUROBO_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_CUROBO_ROOT))

from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import GoalToolPose, JointState


def build_demo_goal(tool_frames: list[str]) -> GoalToolPose:
    """Create a single reachable goal pose for the official Franka robot."""
    return GoalToolPose(
        tool_frames=tool_frames,
        position=torch.tensor([[[[[0.5, 0.0, 0.3]]]]], device="cuda", dtype=torch.float32),
        quaternion=torch.tensor([[[[[1.0, 0.0, 0.0, 0.0]]]]], device="cuda", dtype=torch.float32),
    )


def run_demo(output_dir: Path | None = None) -> dict:
    """Run a minimal standalone pose-planning flow and return a summary dict."""
    config = MotionPlannerCfg.create(
        robot="franka.yml",
        scene_model="collision_test.yml",
    )
    planner = MotionPlanner(config)
    planner.warmup(enable_graph=True, num_warmup_iterations=5)

    current_state = JointState.from_position(
        planner.default_joint_state.position.unsqueeze(0),
        joint_names=planner.joint_names,
    )
    goal_pose = build_demo_goal(planner.tool_frames)
    result = planner.plan_pose(goal_pose, current_state)

    success = result is not None and result.success.any()
    summary = {
        "success": bool(success),
        "planner_class": type(planner).__name__,
        "robot": "franka.yml",
        "scene_model": "collision_test.yml",
        "warmup_enable_graph": True,
        "warmup_iterations": 5,
        "tool_frames": list(planner.tool_frames),
        "start_joint_names": list(planner.joint_names),
        "goal_position": [0.5, 0.0, 0.3],
        "goal_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        "result_class": type(result).__name__ if result is not None else None,
        "interpolation_dt": float(planner.trajopt_solver.config.interpolation_dt),
    }

    if success:
        interpolated = result.get_interpolated_plan()
        waypoints = int(interpolated.position.shape[-2])
        summary.update(
            {
                "trajectory_waypoints": waypoints,
                "trajectory_duration": round(waypoints * summary["interpolation_dt"], 6),
                "result_total_time": float(result.total_time),
                "interpolated_joint_names": list(interpolated.joint_names),
                "interpolated_position_shape": list(interpolated.position.shape),
            }
        )

        print("Planning succeeded")
        print(f"Trajectory waypoints: {waypoints}")
        print(f"Trajectory duration: {summary['trajectory_duration']:.3f}s")
        print(f"Planner solve time: {summary['result_total_time']:.6f}s")
        print(f"Interpolated joints: {', '.join(summary['interpolated_joint_names'])}")
    else:
        print("Planning failed")

    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
        )

    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Minimal standalone CuRobo2 pose demo")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional directory for summary.json output",
    )
    args = parser.parse_args()

    summary = run_demo(output_dir=args.output_dir)
    raise SystemExit(0 if summary["success"] else 1)


if __name__ == "__main__":
    main()
