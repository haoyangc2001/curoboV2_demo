#!/usr/bin/env python3
"""导出 ROKAE 从 CuRobo 到 MuJoCo 的回放合同。

本文件把规划结果转换为直接可回放的关节序列合同，
明确关节映射、采样周期、轨迹数据和验收断言。

支持两种输入来源：
1. 直接从 plan_rokae_motion.py 的输出目录读取 trajectory.json + summary.json（推荐）
2. 调用旧版 demo_plan_pose_rokae.run_demo() 执行在线规划（兼容）
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEMO_SCRIPTS_ROOT = WORKSPACE_ROOT / "demo_scripts"
SCRIPTS_ROOT = WORKSPACE_ROOT / "scripts"
if str(DEMO_SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(DEMO_SCRIPTS_ROOT))
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from rokae_asset_utils import workspace_robot_config_path, workspace_urdf_path


def _parse_movable_urdf_joint_names(urdf_path: Path) -> list[str]:
    """读取 URDF 中全部非固定关节名称。

    Args:
        urdf_path: 机器人 URDF 路径。

    Returns:
        按 URDF 定义顺序排列的可动关节名称列表。
    """
    root = ET.fromstring(urdf_path.read_text())
    movable = []
    for joint in root.findall("joint"):
        if joint.attrib.get("type", "") != "fixed":
            movable.append(joint.attrib["name"])
    return movable


def _build_joint_mapping(source_joint_names: list[str], target_joint_names: list[str]) -> dict[str, Any]:
    """构造源关节顺序到目标关节顺序的映射。

    Args:
        source_joint_names: 源序列关节名列表。
        target_joint_names: 目标序列关节名列表。

    Returns:
        包含索引映射和是否为恒等映射的字典。
    """
    target_index = {name: idx for idx, name in enumerate(target_joint_names)}
    if len(target_index) != len(target_joint_names):
        raise ValueError("duplicate joint names detected in target joint order")

    source_to_target = []
    for source_idx, name in enumerate(source_joint_names):
        if name not in target_index:
            raise ValueError(f"joint {name} missing from target joint order")
        source_to_target.append(
            {
                "joint_name": name,
                "source_index": source_idx,
                "target_index": target_index[name],
            }
        )
    return {
        "source_joint_names": source_joint_names,
        "target_joint_names": target_joint_names,
        "source_to_target": source_to_target,
        "is_identity": source_joint_names == target_joint_names,
    }


def export_contract(
    output_dir: Path,
    plan_output_dir: Path | None = None,
    goal_delta_xyz: tuple[float, float, float] | None = None,
) -> dict[str, Any]:
    """导出一次可供 MuJoCo 使用的回放合同。

    Args:
        output_dir: 合同与复核摘要的输出目录。
        plan_output_dir: 可选规划输出目录；为空时使用 `output_dir/plan`。
        goal_delta_xyz: 可选目标相对位移；为空时复用规划脚本默认值。

    Returns:
        包含轨迹、关节映射、时序约束和来源信息的合同字典。
    """
    if plan_output_dir is None:
        plan_output_dir = output_dir / "plan"
    if goal_delta_xyz is None:
        goal_delta_xyz = run_demo.__defaults__[1]

    plan_summary = run_demo(output_dir=plan_output_dir, goal_delta_xyz=goal_delta_xyz)
    if not plan_summary["success"]:
        raise RuntimeError("failed to obtain a successful ROKAE planning result for S1-009 review")

    urdf_path = workspace_urdf_path()
    urdf_movable_joint_names = _parse_movable_urdf_joint_names(urdf_path)
    interpolated_joint_names = list(plan_summary["trajectory_contract"]["joint_names"])
    if interpolated_joint_names != urdf_movable_joint_names:
        raise ValueError(
            "ROKAE interpolated joint order does not match movable URDF joint order"
        )

    waypoints = list(plan_summary["trajectory_contract"]["waypoints"])
    sample_period_s = float(plan_summary["trajectory_contract"]["sample_period_s"])
    source_mapping = _build_joint_mapping(interpolated_joint_names, urdf_movable_joint_names)

    contract = {
        "contract_name": "curobo_v2_stage1_rokae_mujoco_playback_contract",
        "contract_version": "1.0.0",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "stage": "S1-009",
        "workspace_root": str(WORKSPACE_ROOT),
        "source": {
            "planner_class": plan_summary["planner_class"],
            "result_class": plan_summary["result_class"],
            "robot_config_path": plan_summary["robot_config_path"],
            "tool_frames": plan_summary["tool_frames"],
            "goal_delta_xyz": plan_summary["goal_delta_xyz"],
            "start_tool_position": plan_summary["start_tool_position"],
            "start_tool_quaternion_wxyz": plan_summary["start_tool_quaternion_wxyz"],
            "goal_position": plan_summary["goal_position"],
            "goal_quaternion_wxyz": plan_summary["goal_quaternion_wxyz"],
            "final_tool_position": plan_summary["final_tool_position"],
            "final_tool_quaternion_wxyz": plan_summary["final_tool_quaternion_wxyz"],
            "goal_position_error_norm": plan_summary["goal_position_error_norm"],
            "warmup_enable_graph": plan_summary["warmup_enable_graph"],
            "warmup_iterations": plan_summary["warmup_iterations"],
        },
        "robot_source": {
            "robot_config_path": str(workspace_robot_config_path()),
            "urdf_path": str(urdf_path),
            "ee_body_name": "tool0",
        },
        "joint_contract": {
            "planner_active_joint_names": list(plan_summary["start_joint_names"]),
            "interpolated_joint_names": interpolated_joint_names,
            "urdf_movable_joint_names": urdf_movable_joint_names,
            "mujoco_expected_joint_names": urdf_movable_joint_names,
            "mapping_policy": (
                "Resolve MuJoCo joints strictly by joint name. Build the target qpos vector in "
                "MuJoCo joint order, and fail fast on any missing or duplicate names."
            ),
            "interpolated_to_mujoco_mapping": source_mapping,
        },
        "timing_contract": {
            "sample_period_s": sample_period_s,
            "sample_count": len(waypoints),
            "trajectory_span_s": round(max(0, len(waypoints) - 1) * sample_period_s, 6),
            "final_hold_s": sample_period_s,
            "playback_policy": (
                "Replay the exported waypoint sequence in order. Apply one waypoint every "
                "sample_period_s seconds without ROS topics or additional interpolation."
            ),
        },
        "trajectory_contract": {
            "format": "joint_position_sequence",
            "storage_layout": "[waypoint_index][joint_index]",
            "waypoint_joint_count": len(interpolated_joint_names),
            "waypoints": waypoints,
            "first_waypoint": waypoints[0],
            "last_waypoint": waypoints[-1],
        },
        "review_assertions": [
            "The playback input is the interpolated CuRobo joint sequence extracted from the minimal ROKAE pose demo.",
            "MuJoCo joint mapping must be resolved by joint name instead of relying on positional identity.",
            "The direct MuJoCo playback script must stay independent from ROS topics and main-project launch files.",
            "The direct playback must preserve the ROKAE URDF mesh scale values when generating MJCF.",
        ],
        "notes": [
            "The stage-1 ROKAE contract is derived from the workspace-local adapted robot bundle created in S1-007.",
            "The first validation target is a small relative translation from the start tool0 pose, chosen for stability.",
        ],
        "runtime": {
            "result_total_time_s": plan_summary["result_total_time"],
            "solve_time_s": plan_summary.get("result_solve_time"),
        },
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "playback_contract.json").write_text(
        json.dumps(contract, indent=2, ensure_ascii=False) + "\n"
    )

    review_summary = {
        "stage": "S1-009",
        "passed": True,
        "checks": {
            "real_planning_result_inspected": True,
            "interpolated_joint_order_matches_urdf_movable_joints": True,
            "mujoco_mapping_defined_without_ros_topics": True,
            "fixed_dt_policy_defined": True,
        },
        "artifacts": {
            "plan_summary_path": str(plan_output_dir / "summary.json"),
            "contract_path": str(output_dir / "playback_contract.json"),
        },
    }
    (output_dir / "review_summary.json").write_text(
        json.dumps(review_summary, indent=2, ensure_ascii=False) + "\n"
    )
    return contract


def build_contract_from_plan_output(
    plan_output_dir: Path,
    output_dir: Path,
) -> dict[str, Any]:
    """从 plan_rokae_motion.py 的输出目录构建 MuJoCo 回放合同。

    直接读取 plan 输出中的 trajectory.json 和 summary.json，
    无需重新执行规划。

    Args:
        plan_output_dir: plan_rokae_motion.py 的输出目录（含 summary.json 和 trajectory.json）。
        output_dir: 合同与复核摘要的输出目录。

    Returns:
        包含轨迹、关节映射、时序约束和来源信息的合同字典。
    """
    summary_path = plan_output_dir / "summary.json"
    trajectory_path = plan_output_dir / "trajectory.json"
    if not summary_path.exists():
        raise FileNotFoundError(f"summary.json not found in {plan_output_dir}")
    if not trajectory_path.exists():
        raise FileNotFoundError(f"trajectory.json not found in {plan_output_dir}")

    plan_summary = json.loads(summary_path.read_text())
    traj_data = json.loads(trajectory_path.read_text())

    if not plan_summary.get("success"):
        raise RuntimeError(
            f"plan in {plan_output_dir} was not successful (status={plan_summary.get('status')})"
        )

    urdf_path = workspace_urdf_path()
    urdf_movable_joint_names = _parse_movable_urdf_joint_names(urdf_path)

    interpolated_joint_names = list(traj_data["joint_names"])
    if interpolated_joint_names != urdf_movable_joint_names:
        raise ValueError(
            "ROKAE interpolated joint order does not match movable URDF joint order"
        )

    waypoints = list(traj_data["waypoints"])
    sample_period_s = float(traj_data["sample_period_s"])
    source_mapping = _build_joint_mapping(interpolated_joint_names, urdf_movable_joint_names)

    contract = {
        "contract_name": "curobo_v2_rokae_mujoco_playback_contract",
        "contract_version": "2.0.0",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "stage": "S6",
        "workspace_root": str(WORKSPACE_ROOT),
        "source": {
            "planner_class": "MotionPlanner",
            "result_class": "TrajOptSolverResult",
            "robot_config_path": plan_summary.get("robot_config"),
            "tool_frames": plan_summary.get("tool_frames"),
            "mode": plan_summary.get("mode"),
            "start_joint": plan_summary.get("start_joint"),
            "goal_pose": plan_summary.get("goal_pose"),
            "goal_joint": plan_summary.get("goal_joint"),
            "joint_names": plan_summary.get("joint_names"),
            "input_config": plan_summary.get("input_config"),
        },
        "robot_source": {
            "robot_config_path": str(workspace_robot_config_path()),
            "urdf_path": str(urdf_path),
            "ee_body_name": "tool0",
        },
        "joint_contract": {
            "planner_active_joint_names": interpolated_joint_names,
            "interpolated_joint_names": interpolated_joint_names,
            "urdf_movable_joint_names": urdf_movable_joint_names,
            "mujoco_expected_joint_names": urdf_movable_joint_names,
            "mapping_policy": (
                "Resolve MuJoCo joints strictly by joint name. Build the target qpos vector in "
                "MuJoCo joint order, and fail fast on any missing or duplicate names."
            ),
            "interpolated_to_mujoco_mapping": source_mapping,
        },
        "timing_contract": {
            "sample_period_s": sample_period_s,
            "sample_count": len(waypoints),
            "trajectory_span_s": round(max(0, len(waypoints) - 1) * sample_period_s, 6),
            "final_hold_s": sample_period_s,
            "playback_policy": (
                "Replay the exported waypoint sequence in order. Apply one waypoint every "
                "sample_period_s seconds without ROS topics or additional interpolation."
            ),
        },
        "trajectory_contract": {
            "format": "joint_position_sequence",
            "storage_layout": "[waypoint_index][joint_index]",
            "waypoint_joint_count": len(interpolated_joint_names),
            "waypoints": waypoints,
            "first_waypoint": waypoints[0],
            "last_waypoint": waypoints[-1],
        },
        "review_assertions": [
            "The playback input is the interpolated CuRobo joint sequence from plan_rokae_motion.py.",
            "MuJoCo joint mapping must be resolved by joint name instead of relying on positional identity.",
            "The direct MuJoCo playback script must stay independent from ROS topics and main-project launch files.",
            "The direct playback must preserve the ROKAE URDF mesh scale values when generating MJCF.",
        ],
        "notes": [
            "Contract v2.0 is built from plan_rokae_motion.py output (trajectory.json + summary.json).",
            "Supports both point_to_point and joint_target planning modes.",
        ],
        "runtime": {
            "solve_time_s": plan_summary.get("solve_time"),
            "total_time_s": plan_summary.get("total_time"),
            "wall_time_s": plan_summary.get("wall_time"),
        },
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "playback_contract.json").write_text(
        json.dumps(contract, indent=2, ensure_ascii=False) + "\n"
    )

    review_summary = {
        "stage": "S6",
        "passed": True,
        "checks": {
            "trajectory_json_read_successfully": True,
            "summary_json_read_successfully": True,
            "interpolated_joint_order_matches_urdf_movable_joints": True,
            "mujoco_mapping_defined_without_ros_topics": True,
            "fixed_dt_policy_defined": True,
        },
        "artifacts": {
            "plan_summary_path": str(plan_output_dir / "summary.json"),
            "plan_trajectory_path": str(plan_output_dir / "trajectory.json"),
            "contract_path": str(output_dir / "playback_contract.json"),
        },
    }
    (output_dir / "review_summary.json").write_text(
        json.dumps(review_summary, indent=2, ensure_ascii=False) + "\n"
    )
    return contract


def main() -> None:
    """命令行入口。

    支持两种模式：
    --plan-output-dir 直接从已有规划输出构建合同（推荐）
    --legacy 调用旧版 run_demo 执行在线规划后构建合同（兼容）

    Returns:
        无返回值；成功时输出合同路径与轨迹采样信息。
    """
    parser = argparse.ArgumentParser(
        description="Export a ROKAE MuJoCo playback contract from planning output"
    )
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory for contract artifacts")
    parser.add_argument(
        "--plan-output-dir",
        type=Path,
        default=None,
        help="Directory containing summary.json and trajectory.json from plan_rokae_motion.py",
    )
    parser.add_argument(
        "--legacy",
        action="store_true",
        help="Use legacy mode: run demo_plan_pose_rokae.run_demo() for online planning",
    )
    parser.add_argument("--goal-dx", type=float, default=0.12, help="Goal offset in x (m, legacy mode only)")
    parser.add_argument("--goal-dy", type=float, default=0.0, help="Goal offset in y (m, legacy mode only)")
    parser.add_argument("--goal-dz", type=float, default=0.05, help="Goal offset in z (m, legacy mode only)")
    args = parser.parse_args()

    if args.legacy:
        # Legacy mode: import and run demo
        from demo_plan_pose_rokae import run_demo
        if args.plan_output_dir is None:
            args.plan_output_dir = args.output_dir / "plan"
        contract = export_contract(
            args.output_dir,
            plan_output_dir=args.plan_output_dir,
            goal_delta_xyz=(args.goal_dx, args.goal_dy, args.goal_dz),
        )
    else:
        if args.plan_output_dir is None:
            parser.error("--plan-output-dir is required (or use --legacy)")
        contract = build_contract_from_plan_output(
            args.plan_output_dir,
            args.output_dir,
        )

    print("Contract export succeeded")
    print(f"Contract: {args.output_dir / 'playback_contract.json'}")
    print(f"Waypoints: {contract['timing_contract']['sample_count']}")
    print(f"Sample period: {contract['timing_contract']['sample_period_s']:.6f}s")


if __name__ == "__main__":
    main()
