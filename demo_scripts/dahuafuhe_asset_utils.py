#!/usr/bin/env python3
"""Helpers for stage-1 dahuafuhe asset adaptation inside the demo workspace."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = WORKSPACE_ROOT.parent / "tashan_robot"

SOURCE_DAHUAFUHE_ROOT = PROJECT_ROOT / "resource" / "config" / "dahuafuhe"
SOURCE_ROBOT_ROOT = SOURCE_DAHUAFUHE_ROOT / "robot"
SOURCE_CUROBO_ROBOT_ASSET_ROOT = (
    PROJECT_ROOT / "src" / "robot_description" / "curobo" / "content" / "assets" / "robot" / "rokae"
)

WORKSPACE_DAHUAFUHE_ROOT = WORKSPACE_ROOT / "robot_assets" / "dahuafuhe"
WORKSPACE_DAHUAFUHE_ROBOT_ROOT = WORKSPACE_DAHUAFUHE_ROOT / "robot"
WORKSPACE_DAHUAFUHE_CUROBO_ROOT = WORKSPACE_DAHUAFUHE_ROBOT_ROOT / "curobo"
WORKSPACE_DAHUAFUHE_MESH_ROOT = WORKSPACE_DAHUAFUHE_CUROBO_ROOT / "meshes"

START_LAUNCH_NAME = "start.launch.yaml"
ROBOT_CONFIG_NAME = "rokae_cr7_dahuafuhe.yml"
URDF_NAME = "rokae_cr7_dahuafuhe.urdf"
SPHERES_NAME = "rokae_cr7_dahuafuhe_spherized.yml"
HAND_MESH_NAME = "dahuafuhe_v2.stl"


def load_yaml(path: Path) -> Any:
    return yaml.safe_load(path.read_text())


def workspace_start_launch_path() -> Path:
    return WORKSPACE_DAHUAFUHE_ROOT / START_LAUNCH_NAME


def workspace_robot_config_path() -> Path:
    return WORKSPACE_DAHUAFUHE_ROBOT_ROOT / ROBOT_CONFIG_NAME


def workspace_urdf_path() -> Path:
    return WORKSPACE_DAHUAFUHE_CUROBO_ROOT / URDF_NAME


def workspace_spheres_path() -> Path:
    return WORKSPACE_DAHUAFUHE_ROBOT_ROOT / "spheres" / SPHERES_NAME


def workspace_manifest_path() -> Path:
    return WORKSPACE_DAHUAFUHE_ROOT / "bundle_manifest.json"


def source_start_launch_path() -> Path:
    return SOURCE_DAHUAFUHE_ROOT / START_LAUNCH_NAME


def source_robot_config_path() -> Path:
    return SOURCE_ROBOT_ROOT / ROBOT_CONFIG_NAME


def source_urdf_path() -> Path:
    return SOURCE_ROBOT_ROOT / "curobo" / URDF_NAME


def source_spheres_path() -> Path:
    return SOURCE_ROBOT_ROOT / "spheres" / SPHERES_NAME


def source_hand_mesh_path() -> Path:
    return SOURCE_ROBOT_ROOT / "curobo" / "meshes" / HAND_MESH_NAME


def resolve_robot_config_for_workspace() -> dict[str, Any]:
    """Load the copied robot config and normalize file paths to workspace-absolute paths."""
    robot_cfg = copy.deepcopy(load_yaml(workspace_robot_config_path()))
    kinematics_cfg = robot_cfg["robot_cfg"]["kinematics"]

    for key in ("urdf_path", "asset_root_path"):
        value = kinematics_cfg.get(key)
        if not isinstance(value, str) or not value or "://" in value:
            continue
        path = Path(value)
        if not path.is_absolute():
            kinematics_cfg[key] = str((workspace_robot_config_path().parent / path).resolve())

    collision_spheres = kinematics_cfg.get("collision_spheres")
    if isinstance(collision_spheres, str):
        collision_spheres_path = Path(collision_spheres)
        if not collision_spheres_path.is_absolute():
            collision_spheres_path = (workspace_robot_config_path().parent / collision_spheres_path).resolve()
        kinematics_cfg["collision_spheres"] = load_yaml(collision_spheres_path)["collision_spheres"]

    return robot_cfg


def is_within_workspace(path: Path) -> bool:
    try:
        path.resolve().relative_to(WORKSPACE_ROOT.resolve())
        return True
    except ValueError:
        return False
