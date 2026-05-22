#!/usr/bin/env python3
"""ROKAE 资产包路径与配置辅助函数（scripts 独立版）。

从 demo_scripts/rokae_asset_utils.py 复制并清理，移除了对 tashan_robot 的所有引用，
保证 curoboV2_demo 可独立运行。
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]

WORKSPACE_ROKAE_ROOT = WORKSPACE_ROOT / "robot_assets" / "ROKAE"
WORKSPACE_ROBOT_ROOT = WORKSPACE_ROKAE_ROOT / "robot"
WORKSPACE_CUROBO_ROOT = WORKSPACE_ROBOT_ROOT / "curobo"
WORKSPACE_MESH_ROOT = WORKSPACE_CUROBO_ROOT / "meshes"

START_LAUNCH_NAME = "start.launch.yaml"
ROBOT_CONFIG_NAME = "xms5_r800_w4g3b4c_dahuafuhe.yml"
URDF_NAME = "ROKAE_SR5_0.9C.urdf"
SPHERES_NAME = "ROKAE_SR5_0.9C_spherized.yml"
HAND_MESH_NAME = "dahuafuhe_v2.stl"


def load_yaml(path: Path) -> Any:
    return yaml.safe_load(path.read_text())


def workspace_start_launch_path() -> Path:
    return WORKSPACE_ROKAE_ROOT / START_LAUNCH_NAME


def workspace_robot_config_path() -> Path:
    return WORKSPACE_ROBOT_ROOT / ROBOT_CONFIG_NAME


def workspace_urdf_path() -> Path:
    return WORKSPACE_CUROBO_ROOT / URDF_NAME


def workspace_spheres_path() -> Path:
    return WORKSPACE_ROBOT_ROOT / "spheres" / SPHERES_NAME


def workspace_manifest_path() -> Path:
    return WORKSPACE_ROKAE_ROOT / "bundle_manifest.json"


def resolve_robot_config_for_workspace(
    robot_config_path: Path | None = None,
) -> dict[str, Any]:
    """加载并归一化机器人配置。

    Args:
        robot_config_path: 可选的机器人配置 YAML 路径。为 None 时使用默认路径。

    Returns:
        归一化后的机器人配置字典（路径已转绝对，碰撞球已内联）。
    """
    config_path = robot_config_path or workspace_robot_config_path()
    robot_cfg = copy.deepcopy(load_yaml(config_path))
    kinematics_cfg = robot_cfg["robot_cfg"]["kinematics"]

    for key in ("urdf_path", "asset_root_path"):
        value = kinematics_cfg.get(key)
        if not isinstance(value, str) or not value or "://" in value:
            continue
        path = Path(value)
        if not path.is_absolute():
            kinematics_cfg[key] = str((config_path.parent / path).resolve())

    collision_spheres = kinematics_cfg.get("collision_spheres")
    if isinstance(collision_spheres, str):
        collision_spheres_path = Path(collision_spheres)
        if not collision_spheres_path.is_absolute():
            collision_spheres_path = (config_path.parent / collision_spheres_path).resolve()
        kinematics_cfg["collision_spheres"] = load_yaml(collision_spheres_path)[
            "collision_spheres"
        ]

    return robot_cfg
