#!/usr/bin/env python3
"""重复执行大花复合末端位姿规划，用于阶段一稳定性复核。

本文件通过多次调用单次规划脚本，收集成功率、轨迹长度和误差信息，
生成可归档的批量复核摘要。
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

from rokae_asset_utils import workspace_robot_config_path
from demo_plan_pose_rokae import DEFAULT_GOAL_DELTA_XYZ, run_demo


def main() -> None:
    """命令行入口。

    Returns:
        无返回值；根据全部重复运行是否通过设置退出码。
    """
    parser = argparse.ArgumentParser(description="Repeat the ROKAE pose demo for review")
    parser.add_argument(
        "--repeat-count",
        type=int,
        default=10,
        help="Number of full pose-demo runs to execute",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Directory that receives per-run outputs and the review summary",
    )
    parser.add_argument("--goal-dx", type=float, default=DEFAULT_GOAL_DELTA_XYZ[0], help="Goal offset in x (m)")
    parser.add_argument("--goal-dy", type=float, default=DEFAULT_GOAL_DELTA_XYZ[1], help="Goal offset in y (m)")
    parser.add_argument("--goal-dz", type=float, default=DEFAULT_GOAL_DELTA_XYZ[2], help="Goal offset in z (m)")
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    run_rows = []
    goal_delta_xyz = (args.goal_dx, args.goal_dy, args.goal_dz)

    for run_idx in range(1, args.repeat_count + 1):
        run_dir = args.output_dir / f"run_{run_idx:02d}"
        summary = run_demo(output_dir=run_dir, goal_delta_xyz=goal_delta_xyz)
        run_rows.append(
            {
                "run_index": run_idx,
                "success": summary["success"],
                "trajectory_waypoints": summary.get("trajectory_waypoints"),
                "trajectory_duration": summary.get("trajectory_duration"),
                "result_total_time": summary.get("result_total_time"),
                "goal_position_error_norm": summary.get("goal_position_error_norm"),
                "joint_names": summary.get("interpolated_joint_names"),
            }
        )
        print(f"run_{run_idx:02d}: {'pass' if summary['success'] else 'fail'}")

    review_summary = {
        "stage": "S1-008",
        "generated_at": datetime.now().astimezone().isoformat(),
        "repeat_count": args.repeat_count,
        "passed": all(run["success"] for run in run_rows),
        "goal_delta_xyz": list(goal_delta_xyz),
        "robot_config_path": str(workspace_robot_config_path()),
        "runs": run_rows,
    }
    (args.output_dir / "review_summary.json").write_text(
        json.dumps(review_summary, indent=2, ensure_ascii=False) + "\n"
    )

    raise SystemExit(0 if review_summary["passed"] else 1)


if __name__ == "__main__":
    main()
