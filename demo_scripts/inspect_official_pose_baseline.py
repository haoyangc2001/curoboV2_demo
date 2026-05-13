#!/usr/bin/env python3
"""检查官方位姿规划示例的 API 基线。

本文件用于记录官方 Franka 示例在当前工作区中的关键 API 形态，
把规划器、输入输出对象与张量形状落盘到证据目录，便于后续迁移对照。
"""

from __future__ import annotations

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


def main() -> None:
    """执行一次官方示例并输出 API 基线信息。

    Returns:
        无返回值；成功时把基线 JSON 写入 `evidence/s1_003/`。
    """
    config = MotionPlannerCfg.create(
        robot="franka.yml",
        scene_model="collision_test.yml",
    )
    planner = MotionPlanner(config)
    planner.warmup(enable_graph=True, num_warmup_iterations=5)

    q_start = JointState.from_position(
        planner.default_joint_state.position.unsqueeze(0),
        joint_names=planner.joint_names,
    )

    goal_pose = GoalToolPose(
        tool_frames=planner.tool_frames,
        position=torch.tensor([[[[[0.5, 0.0, 0.3]]]]], device="cuda", dtype=torch.float32),
        quaternion=torch.tensor([[[[[1.0, 0.0, 0.0, 0.0]]]]], device="cuda", dtype=torch.float32),
    )

    result = planner.plan_pose(goal_pose, q_start)
    interpolated = result.get_interpolated_plan() if result is not None and result.success.any() else None

    payload = {
        "planner_class": type(planner).__name__,
        "config_class": type(config).__name__,
        "robot_argument": "franka.yml",
        "scene_model_argument": "collision_test.yml",
        "warmup_called": True,
        "warmup_kwargs": {
            "enable_graph": True,
            "num_warmup_iterations": 5,
        },
        "tool_frames": list(planner.tool_frames),
        "joint_names": list(planner.joint_names),
        "goal_pose_type": type(goal_pose).__name__,
        "goal_position_shape": list(goal_pose.position.shape),
        "goal_quaternion_shape": list(goal_pose.quaternion.shape),
        "start_state_type": type(q_start).__name__,
        "start_position_shape": list(q_start.position.shape),
        "result_is_none": result is None,
        "result_success_any": bool(result.success.any().item()) if result is not None else False,
        "result_class": type(result).__name__ if result is not None else None,
        "interpolated_plan_type": type(interpolated).__name__ if interpolated is not None else None,
        "interpolated_joint_names": list(interpolated.joint_names) if interpolated is not None else None,
        "interpolated_position_shape": list(interpolated.position.shape) if interpolated is not None else None,
        "interpolation_dt": float(planner.trajopt_solver.config.interpolation_dt),
        "result_total_time": float(result.total_time) if result is not None else None,
    }

    out_path = WORKSPACE_ROOT / "evidence" / "s1_003" / "official_pose_api_baseline.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    print(out_path)


if __name__ == "__main__":
    main()
