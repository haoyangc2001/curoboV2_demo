#!/usr/bin/env python3
"""大花复合末端阶段一最小位姿规划示例。

本文件基于工作区内已适配的机器人资产，执行一次从当前工具位姿出发的
相对位移规划，并导出便于后续回放和验收的摘要数据。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import torch

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
LOCAL_CUROBO_ROOT = WORKSPACE_ROOT / "third_party" / "curobo"
if str(LOCAL_CUROBO_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_CUROBO_ROOT))
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import GoalToolPose, JointState

from dahuafuhe_asset_utils import resolve_robot_config_for_workspace, workspace_robot_config_path


DEFAULT_GOAL_DELTA_XYZ = (0.12, 0.0, 0.05)


def _to_float(value: Any) -> float | None:
    """把张量或标量安全转换为 `float`。

    Args:
        value: `None`、Python 标量或 Torch 张量。

    Returns:
        转换后的浮点数；若输入为空或空张量则返回 `None`。
    """
    if value is None:
        return None
    if torch.is_tensor(value):
        flat = value.reshape(-1)
        if flat.numel() == 0:
            return None
        return float(flat[0].item())
    return float(value)


def _squeeze_positions(position: torch.Tensor, expected_joint_count: int) -> list[list[float]]:
    """把轨迹张量压缩为二维关节序列。

    Args:
        position: CuRobo 输出的轨迹位置张量。
        expected_joint_count: 每个 waypoint 期望的关节数。

    Returns:
        `[[joint0, joint1, ...], ...]` 形式的关节位置序列。
    """
    cpu = position.detach().to("cpu")
    while cpu.ndim > 2:
        if cpu.shape[0] != 1:
            raise ValueError(f"expected singleton leading dimensions before waypoint data, got {list(cpu.shape)}")
        cpu = cpu[0]
    if cpu.ndim != 2:
        raise ValueError(f"expected 2D waypoint tensor after squeeze, got {list(cpu.shape)}")
    if cpu.shape[1] != expected_joint_count:
        raise ValueError(f"waypoint width {cpu.shape[1]} does not match joint count {expected_joint_count}")
    return cpu.tolist()


def build_demo_goal(
    planner: MotionPlanner,
    current_state: JointState,
    goal_delta_xyz: tuple[float, float, float] = DEFAULT_GOAL_DELTA_XYZ,
) -> tuple[GoalToolPose, dict[str, list[float]]]:
    """基于当前工具位姿构造一个相对平移目标。

    Args:
        planner: 已初始化的运动规划器。
        current_state: 当前关节状态。
        goal_delta_xyz: 相对当前工具位姿的 XYZ 平移增量，单位米。

    Returns:
        二元组：
        1. `GoalToolPose` 目标位姿。
        2. 便于调试记录的起点/目标位姿信息字典。
    """
    kin_state = planner.compute_kinematics(current_state)
    tool_pose = kin_state.tool_poses.get_link_pose(planner.tool_frames[0])

    goal_position = tool_pose.position.clone().reshape(1, 1, 1, 1, 3)
    goal_position[..., 0] += goal_delta_xyz[0]
    goal_position[..., 1] += goal_delta_xyz[1]
    goal_position[..., 2] += goal_delta_xyz[2]
    goal_quaternion = tool_pose.quaternion.clone().reshape(1, 1, 1, 1, 4)

    goal_pose = GoalToolPose(
        tool_frames=planner.tool_frames,
        position=goal_position,
        quaternion=goal_quaternion,
    )
    pose_debug = {
        "start_position": tool_pose.position.detach().cpu().reshape(-1).tolist(),
        "start_quaternion_wxyz": tool_pose.quaternion.detach().cpu().reshape(-1).tolist(),
        "goal_position": goal_position.detach().cpu().reshape(-1).tolist(),
        "goal_quaternion_wxyz": goal_quaternion.detach().cpu().reshape(-1).tolist(),
    }
    return goal_pose, pose_debug


def run_demo(
    output_dir: Path | None = None,
    goal_delta_xyz: tuple[float, float, float] = DEFAULT_GOAL_DELTA_XYZ,
) -> dict[str, Any]:
    """执行一次大花复合末端位姿规划。

    Args:
        output_dir: 可选输出目录；若提供则写出 `summary.json`。
        goal_delta_xyz: 相对起始工具位姿的目标平移增量。

    Returns:
        包含规划结果、轨迹合同、末端误差和调试信息的摘要字典。
    """
    robot_cfg = resolve_robot_config_for_workspace()
    config = MotionPlannerCfg.create(
        robot=robot_cfg,
        scene_model=None,
    )
    planner = MotionPlanner(config)
    planner.warmup(enable_graph=True, num_warmup_iterations=5)

    current_state = JointState.from_position(
        planner.default_joint_state.position.unsqueeze(0),
        joint_names=planner.joint_names,
    )
    goal_pose, pose_debug = build_demo_goal(planner, current_state, goal_delta_xyz=goal_delta_xyz)
    result = planner.plan_pose(goal_pose, current_state)

    success = result is not None and bool(result.success.any().item())
    summary: dict[str, Any] = {
        "success": bool(success),
        "planner_class": type(planner).__name__,
        "robot_config_path": str(workspace_robot_config_path()),
        "robot_config_name": workspace_robot_config_path().name,
        "scene_model": None,
        "warmup_enable_graph": True,
        "warmup_iterations": 5,
        "tool_frames": list(planner.tool_frames),
        "start_joint_names": list(planner.joint_names),
        "start_joint_position": current_state.position.detach().cpu().reshape(-1).tolist(),
        "goal_delta_xyz": list(goal_delta_xyz),
        "start_tool_position": pose_debug["start_position"],
        "start_tool_quaternion_wxyz": pose_debug["start_quaternion_wxyz"],
        "goal_position": pose_debug["goal_position"],
        "goal_quaternion_wxyz": pose_debug["goal_quaternion_wxyz"],
        "result_class": type(result).__name__ if result is not None else None,
        "interpolation_dt": float(planner.trajopt_solver.config.interpolation_dt),
        "notes": [
            "Stage-1 dahuafuhe target pose is selected by applying a small relative translation to the current tool0 pose.",
            "This step stays inside the standalone demo workspace and does not use ROS services or the main project install tree.",
        ],
    }

    if success:
        interpolated = result.get_interpolated_plan()
        interpolated_joint_names = list(interpolated.joint_names)
        waypoints = _squeeze_positions(interpolated.position, len(interpolated_joint_names))
        waypoint_count = len(waypoints)

        end_state = JointState.from_position(
            torch.tensor([waypoints[-1]], device=current_state.position.device, dtype=current_state.position.dtype),
            joint_names=interpolated_joint_names,
        )
        end_pose = planner.compute_kinematics(end_state).tool_poses.get_link_pose(planner.tool_frames[0])
        goal_position_tensor = goal_pose.position.detach().reshape(-1, 3)
        position_error = torch.linalg.norm(end_pose.position - goal_position_tensor, dim=-1)

        summary.update(
            {
                "trajectory_waypoints": waypoint_count,
                "trajectory_duration": round(waypoint_count * summary["interpolation_dt"], 6),
                "result_total_time": float(result.total_time),
                "interpolated_joint_names": interpolated_joint_names,
                "interpolated_position_shape": list(interpolated.position.shape),
                "trajectory_contract": {
                    "format": "joint_position_sequence",
                    "storage_layout": "[waypoint_index][joint_index]",
                    "sample_period_s": summary["interpolation_dt"],
                    "joint_names": interpolated_joint_names,
                    "waypoint_joint_count": len(interpolated_joint_names),
                    "waypoints": waypoints,
                    "first_waypoint": waypoints[0],
                    "last_waypoint": waypoints[-1],
                },
                "final_tool_position": end_pose.position.detach().cpu().reshape(-1).tolist(),
                "final_tool_quaternion_wxyz": end_pose.quaternion.detach().cpu().reshape(-1).tolist(),
                "goal_position_error_norm": float(position_error.reshape(-1)[0].item()),
                "result_solve_time": _to_float(getattr(result, "solve_time", None)),
            }
        )

        print("Planning succeeded")
        print(f"Trajectory waypoints: {waypoint_count}")
        print(f"Trajectory duration: {summary['trajectory_duration']:.3f}s")
        print(f"Planner total time: {summary['result_total_time']:.6f}s")
        print(f"Goal position error norm: {summary['goal_position_error_norm']:.6f}m")
    else:
        print("Planning failed")

    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
        )

    return summary


def main() -> None:
    """命令行入口。

    Returns:
        无返回值；根据规划结果设置退出码。
    """
    parser = argparse.ArgumentParser(description="Minimal standalone CuRobo2 dahuafuhe pose demo")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional directory for summary.json output",
    )
    parser.add_argument("--goal-dx", type=float, default=DEFAULT_GOAL_DELTA_XYZ[0], help="Goal offset in x (m)")
    parser.add_argument("--goal-dy", type=float, default=DEFAULT_GOAL_DELTA_XYZ[1], help="Goal offset in y (m)")
    parser.add_argument("--goal-dz", type=float, default=DEFAULT_GOAL_DELTA_XYZ[2], help="Goal offset in z (m)")
    args = parser.parse_args()

    summary = run_demo(
        output_dir=args.output_dir,
        goal_delta_xyz=(args.goal_dx, args.goal_dy, args.goal_dz),
    )
    raise SystemExit(0 if summary["success"] else 1)


if __name__ == "__main__":
    main()
