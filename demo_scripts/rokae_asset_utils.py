#!/usr/bin/env python3
"""ROKAE 资产包路径与配置辅助函数。

本文件负责统一管理演示工作区内当前启用的 ROKAE 机械臂资产路径，
并提供 YAML 读取、路径解析、机器人配置归一化等基础能力。
"""

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

WORKSPACE_DAHUAFUHE_ROOT = WORKSPACE_ROOT / "robot_assets" / "ROKAE"
WORKSPACE_DAHUAFUHE_ROBOT_ROOT = WORKSPACE_DAHUAFUHE_ROOT / "robot"
WORKSPACE_DAHUAFUHE_CUROBO_ROOT = WORKSPACE_DAHUAFUHE_ROBOT_ROOT / "curobo"
WORKSPACE_DAHUAFUHE_MESH_ROOT = WORKSPACE_DAHUAFUHE_CUROBO_ROOT / "meshes"

START_LAUNCH_NAME = "start.launch.yaml"
ROBOT_CONFIG_NAME = "xms5_r800_w4g3b4c_dahuafuhe.yml"
URDF_NAME = "ROKAE_SR5_0.9C.urdf"
SPHERES_NAME = "ROKAE_SR5_0.9C_spherized.yml"
HAND_MESH_NAME = "dahuafuhe_v2.stl"


def load_yaml(path: Path) -> Any:
    """读取 YAML 文件。

    Args:
        path: YAML 文件路径。

    Returns:
        解析后的 Python 对象，通常为 `dict` 或 `list`。
    """
    return yaml.safe_load(path.read_text())


def workspace_start_launch_path() -> Path:
    """返回工作区内 `start.launch.yaml` 的绝对路径。

    Returns:
        工作区启动配置文件路径。
    """
    return WORKSPACE_DAHUAFUHE_ROOT / START_LAUNCH_NAME


def workspace_robot_config_path() -> Path:
    """返回工作区内 CuRobo 机器人配置文件路径。

    Returns:
        工作区机器人配置 YAML 的绝对路径。
    """
    return WORKSPACE_DAHUAFUHE_ROBOT_ROOT / ROBOT_CONFIG_NAME


def workspace_urdf_path() -> Path:
    """返回工作区内 URDF 文件路径。

    Returns:
        工作区 URDF 的绝对路径。
    """
    return WORKSPACE_DAHUAFUHE_CUROBO_ROOT / URDF_NAME


def workspace_spheres_path() -> Path:
    """返回工作区内碰撞球配置路径。

    Returns:
        工作区球近似碰撞配置 YAML 的绝对路径。
    """
    return WORKSPACE_DAHUAFUHE_ROBOT_ROOT / "spheres" / SPHERES_NAME


def workspace_manifest_path() -> Path:
    """返回工作区资产包清单文件路径。

    Returns:
        `bundle_manifest.json` 的绝对路径。
    """
    return WORKSPACE_DAHUAFUHE_ROOT / "bundle_manifest.json"


def source_start_launch_path() -> Path:
    """返回源项目中的启动配置路径。

    Returns:
        原始 `start.launch.yaml` 的绝对路径。
    """
    return SOURCE_DAHUAFUHE_ROOT / START_LAUNCH_NAME


def source_robot_config_path() -> Path:
    """返回源项目中的机器人配置路径。

    Returns:
        原始机器人配置 YAML 的绝对路径。
    """
    return SOURCE_ROBOT_ROOT / ROBOT_CONFIG_NAME


def source_urdf_path() -> Path:
    """返回源项目中的 URDF 路径。

    Returns:
        原始 URDF 文件的绝对路径。
    """
    return SOURCE_ROBOT_ROOT / "curobo" / URDF_NAME


def source_spheres_path() -> Path:
    """返回源项目中的碰撞球配置路径。

    Returns:
        原始球近似碰撞配置 YAML 的绝对路径。
    """
    return SOURCE_ROBOT_ROOT / "spheres" / SPHERES_NAME


def source_hand_mesh_path() -> Path:
    """返回源项目中的末端工具网格路径。

    Returns:
        手爪或工具 STL 文件的绝对路径。
    """
    return SOURCE_ROBOT_ROOT / "curobo" / "meshes" / HAND_MESH_NAME


def resolve_robot_config_for_workspace() -> dict[str, Any]:
    """加载并归一化工作区内的机器人配置。

    该函数会把相对路径改写为工作区绝对路径，并将碰撞球文件内容内联到配置中，
    方便后续直接交给 CuRobo 使用。

    Returns:
        归一化后的机器人配置字典。
    """
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
    """判断路径是否位于当前演示工作区内。

    Args:
        path: 待检查路径。

    Returns:
        若路径解析后属于 `WORKSPACE_ROOT` 子路径则返回 `True`，否则返回 `False`。
    """
    try:
        path.resolve().relative_to(WORKSPACE_ROOT.resolve())
        return True
    except ValueError:
        return False
