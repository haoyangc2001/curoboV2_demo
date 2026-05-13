#!/usr/bin/env python3
"""重复执行大花复合末端规划与 MuJoCo 回放闭环。

本文件通过多次运行一键流程，验证从规划、合同导出到离屏回放的完整链路稳定性。
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

from run_dahuafuhe_demo import run_all


def main() -> None:
    """命令行入口。

    Returns:
        无返回值；根据批量回放复核是否通过设置退出码。
    """
    parser = argparse.ArgumentParser(description="Repeat the dahuafuhe MuJoCo playback loop")
    parser.add_argument(
        "--repeat-count",
        type=int,
        default=10,
        help="Number of full planning plus playback loops to execute",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Directory that receives per-run outputs and the review summary",
    )
    parser.add_argument(
        "--render-every",
        type=int,
        default=1,
        help="Render every Nth waypoint during offscreen playback",
    )
    parser.add_argument("--goal-dx", type=float, default=0.12, help="Goal offset in x (m)")
    parser.add_argument("--goal-dy", type=float, default=0.0, help="Goal offset in y (m)")
    parser.add_argument("--goal-dz", type=float, default=0.05, help="Goal offset in z (m)")
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    runs = []
    goal_delta_xyz = (args.goal_dx, args.goal_dy, args.goal_dz)

    for run_idx in range(1, args.repeat_count + 1):
        run_dir = args.output_dir / f"run_{run_idx:02d}"
        summary = run_all(run_dir, args.render_every, goal_delta_xyz=goal_delta_xyz)
        run_row = {
            "run_index": run_idx,
            "success": summary["success"],
            "contract_waypoint_count": summary["contract_waypoint_count"],
            "contract_sample_period_s": summary["contract_sample_period_s"],
            "offscreen_rendered_frames": summary["offscreen_rendered_frames"],
            "ee_displacement": summary["ee_displacement"],
            "ee_direction_alignment": summary["ee_direction_alignment"],
            "checks": summary["checks"],
        }
        runs.append(run_row)
        print(f"run_{run_idx:02d}: {'pass' if summary['success'] else 'fail'}")

    review_summary = {
        "stage": "S1-009",
        "generated_at": datetime.now().astimezone().isoformat(),
        "repeat_count": args.repeat_count,
        "render_every": args.render_every,
        "goal_delta_xyz": list(goal_delta_xyz),
        "passed": all(run["success"] for run in runs),
        "runs": runs,
    }
    (args.output_dir / "review_summary.json").write_text(
        json.dumps(review_summary, indent=2, ensure_ascii=False) + "\n"
    )

    raise SystemExit(0 if review_summary["passed"] else 1)


if __name__ == "__main__":
    main()
