#!/usr/bin/env python3
"""生成阶段一演示所需的大花复合末端资产包。

本文件负责从源项目复制最小必要资产到当前工作区，
同时改写 URDF 网格路径、规整 CuRobo 配置，并生成资产清单。
"""

from __future__ import annotations

import argparse
import json
import shutil
from datetime import datetime
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml

from rokae_asset_utils import (
    HAND_MESH_NAME,
    SOURCE_CUROBO_ROBOT_ASSET_ROOT,
    WORKSPACE_DAHUAFUHE_CUROBO_ROOT,
    WORKSPACE_DAHUAFUHE_MESH_ROOT,
    WORKSPACE_DAHUAFUHE_ROOT,
    WORKSPACE_DAHUAFUHE_ROBOT_ROOT,
    source_hand_mesh_path,
    source_robot_config_path,
    source_spheres_path,
    source_start_launch_path,
    source_urdf_path,
    workspace_manifest_path,
    workspace_robot_config_path,
    workspace_spheres_path,
    workspace_start_launch_path,
    workspace_urdf_path,
)


ROKAE_MESH_DIR_NAME = "rokae_cr7_meshes"


def _copy_file(src: Path, dst: Path) -> None:
    """复制单个文件并自动创建目标目录。

    Args:
        src: 源文件路径。
        dst: 目标文件路径。

    Returns:
        无返回值。
    """
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def _rewrite_and_copy_urdf(src: Path, dst: Path) -> dict[str, str]:
    """复制 URDF 并重写其中的网格文件路径。

    Args:
        src: 源 URDF 路径。
        dst: 目标 URDF 路径。

    Returns:
        原始网格路径到工作区内新路径的映射字典。
    """
    dst.parent.mkdir(parents=True, exist_ok=True)
    root = ET.fromstring(src.read_text())
    rewritten_meshes: dict[str, str] = {}

    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if not filename:
            continue

        source_name = Path(filename).name
        if source_name == HAND_MESH_NAME:
            new_filename = f"meshes/{HAND_MESH_NAME}"
        else:
            new_filename = f"meshes/{ROKAE_MESH_DIR_NAME}/{source_name}"
        rewritten_meshes[filename] = new_filename
        mesh.attrib["filename"] = new_filename

    tree = ET.ElementTree(root)
    tree.write(dst, encoding="utf-8", xml_declaration=True)
    return rewritten_meshes


def _write_adapted_robot_config() -> dict:
    """生成适配当前工作区的机器人配置文件。

    Returns:
        写入磁盘后的机器人配置字典。
    """
    source_robot_cfg = yaml.safe_load(source_robot_config_path().read_text())
    source_start_launch = yaml.safe_load(source_start_launch_path().read_text())
    source_spheres = yaml.safe_load(source_spheres_path().read_text())

    source_kinematics = source_robot_cfg["robot_cfg"]["kinematics"]
    source_cspace = source_kinematics["cspace"]
    initial_joint_position = source_start_launch["launch"]["nodes"]["trajectory_planning"][
        "initial_joint_position"
    ]

    adapted_cspace = {
        "joint_names": source_cspace["joint_names"],
        "default_joint_position": initial_joint_position,
        "cspace_distance_weight": source_cspace.get("cspace_distance_weight"),
        "null_space_weight": source_cspace.get("null_space_weight"),
        "max_jerk": source_cspace.get("max_jerk", 500.0),
        "max_acceleration": source_cspace.get("max_acceleration", 10.0),
        "position_limit_clip": source_cspace.get("position_limit_clip", 0.0),
    }
    for optional_key in (
        "null_space_maximum_distance",
        "velocity_scale",
        "acceleration_scale",
        "jerk_scale",
    ):
        if optional_key in source_cspace:
            adapted_cspace[optional_key] = source_cspace[optional_key]

    adapted_robot_cfg = {
        "robot_cfg": {
            "kinematics": {
                "format_version": 2.0,
                "base_link": source_kinematics["base_link"],
                "tool_frames": [source_kinematics["ee_link"]],
                "urdf_path": str(workspace_urdf_path()),
                "asset_root_path": str(WORKSPACE_DAHUAFUHE_CUROBO_ROOT),
                "collision_link_names": source_kinematics["collision_link_names"],
                "collision_spheres": source_spheres["collision_spheres"],
                "collision_sphere_buffer": source_kinematics.get("collision_sphere_buffer", 0.0),
                "self_collision_ignore": source_kinematics.get("self_collision_ignore", {}),
                "self_collision_buffer": source_kinematics.get("self_collision_buffer", {}),
                "use_global_cumul": source_kinematics.get("use_global_cumul", True),
                "mesh_link_names": source_kinematics.get("mesh_link_names", []),
                "lock_joints": source_kinematics.get("lock_joints"),
                "extra_links": source_kinematics.get("extra_links"),
                "cspace": adapted_cspace,
            }
        }
    }

    workspace_robot_config_path().parent.mkdir(parents=True, exist_ok=True)
    workspace_robot_config_path().write_text(
        yaml.safe_dump(adapted_robot_cfg, sort_keys=False, allow_unicode=False)
    )
    return adapted_robot_cfg


def materialize_bundle() -> dict:
    """物化完整的阶段一资产包。

    Returns:
        描述复制结果、路径归一化信息与产物范围的清单字典。
    """
    rokae_mesh_src_root = SOURCE_CUROBO_ROBOT_ASSET_ROOT / "meshes" / ROKAE_MESH_DIR_NAME
    rokae_mesh_dst_root = WORKSPACE_DAHUAFUHE_MESH_ROOT / ROKAE_MESH_DIR_NAME

    _copy_file(source_start_launch_path(), workspace_start_launch_path())
    _copy_file(source_spheres_path(), workspace_spheres_path())
    _copy_file(source_hand_mesh_path(), WORKSPACE_DAHUAFUHE_MESH_ROOT / HAND_MESH_NAME)

    rokae_mesh_dst_root.mkdir(parents=True, exist_ok=True)
    copied_meshes = []
    for src_mesh in sorted(rokae_mesh_src_root.glob("*.stl")):
        dst_mesh = rokae_mesh_dst_root / src_mesh.name
        shutil.copy2(src_mesh, dst_mesh)
        copied_meshes.append(str(dst_mesh))

    rewritten_meshes = _rewrite_and_copy_urdf(source_urdf_path(), workspace_urdf_path())
    adapted_robot_cfg = _write_adapted_robot_config()

    manifest = {
        "bundle_name": "rokae_stage1_asset_bundle",
        "generated_at": datetime.now().astimezone().isoformat(),
        "workspace_root": str(WORKSPACE_DAHUAFUHE_ROOT),
        "copied_files": {
            "start_launch": str(workspace_start_launch_path()),
            "robot_config": str(workspace_robot_config_path()),
            "urdf": str(workspace_urdf_path()),
            "collision_spheres": str(workspace_spheres_path()),
            "hand_mesh": str(WORKSPACE_DAHUAFUHE_MESH_ROOT / HAND_MESH_NAME),
            "rokae_meshes": copied_meshes,
        },
        "path_normalization": {
            "urdf_mesh_rewrites": rewritten_meshes,
            "robot_config_style_adaptation": {
                "added_tool_frames_from_ee_link": adapted_robot_cfg["robot_cfg"]["kinematics"]["tool_frames"],
                "inlined_collision_spheres": True,
                "removed_legacy_keys": [
                    "usd_path",
                    "usd_robot_root",
                    "isaac_usd_path",
                    "usd_flip_joints",
                    "usd_flip_joint_limits",
                    "ee_link",
                ],
                "default_joint_position": adapted_robot_cfg["robot_cfg"]["kinematics"]["cspace"][
                    "default_joint_position"
                ],
            },
            "note": (
                "The copied URDF no longer points to tashan_robot/src/robot_description. "
                "All mesh paths now resolve within curobo2_demo_ws/robot_assets/ROKAE."
            ),
        },
        "scope_note": "Stage-1 bundle only supports the current ROKAE robot target.",
    }
    workspace_manifest_path().write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    return manifest


def main() -> None:
    """命令行入口。

    Returns:
        无返回值；标准输出打印生成的资产包清单。
    """
    parser = argparse.ArgumentParser(description="Materialize the stage-1 ROKAE asset bundle")
    parser.parse_args()

    manifest = materialize_bundle()
    print(json.dumps(manifest, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
