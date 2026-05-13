#!/usr/bin/env python3
"""导出官方 Franka 的 MuJoCo 回放合同。

本文件基于真实的 CuRobo 规划结果生成关节轨迹合同，
用于定义官方 Franka 阶段一的关节顺序、时序和回放输入格式。
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

import mujoco
import torch
import yaml

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
LOCAL_CUROBO_ROOT = WORKSPACE_ROOT / "third_party" / "curobo"
if str(LOCAL_CUROBO_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_CUROBO_ROOT))

from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import GoalToolPose, JointState


FRANKA_CFG_PATH = LOCAL_CUROBO_ROOT / "curobo" / "content" / "configs" / "robot" / "franka.yml"


def build_demo_goal(tool_frames: list[str]) -> GoalToolPose:
    """构造官方 Franka 示例目标位姿。

    Args:
        tool_frames: 规划器使用的末端工具坐标系名称列表。

    Returns:
        固定目标位姿对象。
    """
    return GoalToolPose(
        tool_frames=tool_frames,
        position=torch.tensor([[[[[0.5, 0.0, 0.3]]]]], device="cuda", dtype=torch.float32),
        quaternion=torch.tensor(
            [[[[[1.0, 0.0, 0.0, 0.0]]]]], device="cuda", dtype=torch.float32
        ),
    )


def _to_float(value: Any) -> float | None:
    """把张量或标量安全转换为浮点数。

    Args:
        value: `None`、Python 标量或 Torch 张量。

    Returns:
        转换后的浮点数；无法取值时返回 `None`。
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
    """把轨迹张量压缩为二维 waypoint 列表。

    Args:
        position: 规划结果位置张量。
        expected_joint_count: 每个 waypoint 的目标关节数。

    Returns:
        二维关节位置序列。
    """
    cpu = position.detach().to("cpu")
    while cpu.ndim > 2:
        if cpu.shape[0] != 1:
            raise ValueError(
                f"expected only singleton leading dimensions before waypoint data, got {list(cpu.shape)}"
            )
        cpu = cpu[0]

    if cpu.ndim != 2:
        raise ValueError(f"expected a 2D waypoint tensor after squeeze, got {list(cpu.shape)}")
    if cpu.shape[1] != expected_joint_count:
        raise ValueError(
            f"waypoint width {cpu.shape[1]} does not match joint count {expected_joint_count}"
        )

    return cpu.tolist()


def _load_franka_cfg() -> dict[str, Any]:
    """读取官方 Franka 的 CuRobo 配置。

    Returns:
        `franka.yml` 解析后的字典。
    """
    return yaml.safe_load(FRANKA_CFG_PATH.read_text())


def _resolve_franka_urdf_path(franka_cfg: dict[str, Any]) -> Path:
    """从 CuRobo 配置推导官方 Franka URDF 路径。

    Args:
        franka_cfg: `franka.yml` 解析结果。

    Returns:
        对应 URDF 的绝对路径。
    """
    rel = franka_cfg["robot_cfg"]["kinematics"]["urdf_path"]
    asset_root = franka_cfg["robot_cfg"]["kinematics"]["asset_root_path"]
    return LOCAL_CUROBO_ROOT / "curobo" / "content" / "assets" / asset_root / Path(rel).name


def _parse_movable_urdf_joint_names(urdf_path: Path) -> list[str]:
    """读取 URDF 中的可动关节名称列表。

    Args:
        urdf_path: URDF 文件路径。

    Returns:
        按文件顺序排列的非固定关节名称列表。
    """
    root = ET.fromstring(urdf_path.read_text())
    movable = []
    for joint in root.findall("joint"):
        joint_type = joint.attrib.get("type", "")
        if joint_type != "fixed":
            movable.append(joint.attrib["name"])
    return movable


def _build_joint_mapping(source_joint_names: list[str], target_joint_names: list[str]) -> dict[str, Any]:
    """构造源关节顺序到目标关节顺序的映射描述。

    Args:
        source_joint_names: 源序列关节名列表。
        target_joint_names: 目标序列关节名列表。

    Returns:
        含索引对应关系与身份映射标记的字典。
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


def export_contract(output_dir: Path) -> dict[str, Any]:
    """导出官方 Franka 回放合同。

    Args:
        output_dir: 合同与复核文件输出目录。

    Returns:
        包含规划来源、关节映射、时序策略和轨迹序列的合同字典。
    """
    franka_cfg = _load_franka_cfg()
    urdf_path = _resolve_franka_urdf_path(franka_cfg)
    cspace_joint_names = list(franka_cfg["robot_cfg"]["kinematics"]["cspace"]["joint_names"])
    urdf_movable_joint_names = _parse_movable_urdf_joint_names(urdf_path)

    if urdf_movable_joint_names != cspace_joint_names:
        raise ValueError(
            "franka.yml cspace joint order does not match movable Franka URDF joint order"
        )

    planner_cfg = MotionPlannerCfg.create(robot="franka.yml", scene_model="collision_test.yml")
    planner = MotionPlanner(planner_cfg)
    planner.warmup(enable_graph=True, num_warmup_iterations=5)

    current_state = JointState.from_position(
        planner.default_joint_state.position.unsqueeze(0),
        joint_names=planner.joint_names,
    )
    result = planner.plan_pose(build_demo_goal(planner.tool_frames), current_state)

    success = result is not None and bool(result.success.any().item())
    if not success:
        raise RuntimeError("failed to obtain a successful real planning result for S1-005 review")

    interpolated = result.get_interpolated_plan()
    interpolated_joint_names = list(interpolated.joint_names)
    waypoints = _squeeze_positions(interpolated.position, len(interpolated_joint_names))
    interpolation_dt = _to_float(planner.trajopt_solver.config.interpolation_dt)
    if interpolation_dt is None:
        raise ValueError("planner interpolation_dt is unavailable")

    if interpolated_joint_names != cspace_joint_names:
        raise ValueError(
            "interpolated joint order does not match Franka cspace order; contract would be ambiguous"
        )

    source_mapping = _build_joint_mapping(interpolated_joint_names, urdf_movable_joint_names)

    contract = {
        "contract_name": "curobo_v2_stage1_mujoco_playback_contract",
        "contract_version": "1.0.0",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "stage": "S1-005",
        "workspace_root": str(WORKSPACE_ROOT),
        "source": {
            "planner_class": type(planner).__name__,
            "result_class": type(result).__name__,
            "robot_config": "franka.yml",
            "scene_model": "collision_test.yml",
            "tool_frames": list(planner.tool_frames),
            "goal_position": [0.5, 0.0, 0.3],
            "goal_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "warmup_enable_graph": True,
            "warmup_iterations": 5,
            "mujoco_version": mujoco.__version__,
        },
        "joint_contract": {
            "planner_active_joint_names": list(planner.joint_names),
            "interpolated_joint_names": interpolated_joint_names,
            "franka_cspace_joint_names": cspace_joint_names,
            "urdf_movable_joint_names": urdf_movable_joint_names,
            "locked_cspace_joints": franka_cfg["robot_cfg"]["kinematics"].get("lock_joints", {}),
            "mujoco_expected_joint_names": urdf_movable_joint_names,
            "mapping_policy": (
                "Resolve MuJoCo joints strictly by joint name. Build the target qpos vector in "
                "MuJoCo joint order, and fail fast on any missing or duplicate names."
            ),
            "interpolated_to_mujoco_mapping": source_mapping,
        },
        "timing_contract": {
            "sample_period_s": interpolation_dt,
            "sample_count": len(waypoints),
            "trajectory_span_s": round(max(0, len(waypoints) - 1) * interpolation_dt, 6),
            "final_hold_s": interpolation_dt,
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
            "The playback input is the interpolated CuRobo joint sequence, not planner.joint_names alone.",
            "Finger joints stay in the stage-1 contract because the real interpolated Franka output contains 9 joints.",
            "The direct MuJoCo playback script must not subscribe to ROS topics to recover joint order or timing.",
            "Fixed playback dt is taken directly from the CuRobo interpolated plan sample period.",
        ],
        "notes": [
            (
                "Official Franka URDF movable joint order matches franka.yml cspace joint order, "
                "so the stage-1 official mapping is identity after name resolution."
            ),
            (
                "MuJoCo playback should still resolve joints by name at runtime instead of assuming "
                "positional identity, so model-side omissions or reorderings fail loudly."
            ),
        ],
        "runtime": {
            "result_total_time_s": _to_float(getattr(result, "total_time", None)),
            "solve_time_s": _to_float(getattr(result, "solve_time", None)),
        },
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "playback_contract.json").write_text(
        json.dumps(contract, indent=2, ensure_ascii=False) + "\n"
    )

    review_summary = {
        "stage": "S1-005",
        "passed": True,
        "checks": {
            "real_planning_result_inspected": True,
            "interpolated_joint_order_matches_cspace": True,
            "cspace_joint_order_matches_urdf_movable_joints": True,
            "mujoco_mapping_defined_without_ros_topics": True,
            "fixed_dt_policy_defined": True,
        },
        "artifacts": {
            "contract_path": str(output_dir / "playback_contract.json"),
        },
    }
    (output_dir / "review_summary.json").write_text(
        json.dumps(review_summary, indent=2, ensure_ascii=False) + "\n"
    )
    return contract


def main() -> None:
    """命令行入口。

    Returns:
        无返回值；成功时打印合同的关节顺序与采样信息。
    """
    parser = argparse.ArgumentParser(description="Export the stage-1 MuJoCo playback contract")
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory for evidence output")
    args = parser.parse_args()

    contract = export_contract(args.output_dir)
    print("Playback contract exported")
    print(f"Output dir: {args.output_dir}")
    print(
        "Joint order: "
        + ", ".join(contract["joint_contract"]["mujoco_expected_joint_names"])
    )
    print(f"Sample count: {contract['timing_contract']['sample_count']}")
    print(f"Sample period: {contract['timing_contract']['sample_period_s']:.6f}s")


if __name__ == "__main__":
    main()
