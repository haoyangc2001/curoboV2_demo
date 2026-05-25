#!/usr/bin/env python3
"""ROKAE 资产包路径与配置辅助函数（scripts 独立版）。

从 demo_scripts/rokae_asset_utils.py 复制并清理，移除了对 tashan_robot 的所有引用，
保证 curoboV2_demo 可独立运行。

支持两种碰撞球来源：
1. 从 YAML 文件加载（传统方式）
2. 使用 CuRobo V2 RobotBuilder 自动生成（推荐）
"""

from __future__ import annotations

import copy
import sys
from pathlib import Path
from typing import Any

import yaml

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]

# CuRobo V2 路径（用于自动生成）
CUROBO_PATH = WORKSPACE_ROOT / "third_party" / "curobo"

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


def generate_collision_spheres(
    urdf_path: Path | None = None,
    asset_path: Path | None = None,
    sphere_density: float = 1.0,
    num_collision_samples: int = 1000,
    compute_metrics: bool = False,
) -> dict[str, list[dict[str, Any]]]:
    """使用 CuRobo V2 RobotBuilder 自动生成碰撞球。

    Args:
        urdf_path: URDF 文件路径，默认使用 ROKAE SR5 URDF。
        asset_path: mesh 资产目录，默认使用 ROKAE 资产目录。
        sphere_density: 球密度倍数（默认 1.0，越大球越多）。
        num_collision_samples: 碰撞裁剪采样数。
        compute_metrics: 是否计算拟合质量指标。

    Returns:
        碰撞球字典，格式为 {link_name: [{"center": [x,y,z], "radius": r}, ...]}
    """
    # 确保 CuRobo V2 在 path 中
    if str(CUROBO_PATH) not in sys.path:
        sys.path.insert(0, str(CUROBO_PATH))

    from curobo.robot_builder import RobotBuilder

    urdf = urdf_path or workspace_urdf_path()
    asset = asset_path or WORKSPACE_CUROBO_ROOT

    print(f"Generating collision spheres using CuRobo V2 RobotBuilder...")
    print(f"  URDF: {urdf}")
    print(f"  Asset path: {asset}")

    # 创建 builder
    builder = RobotBuilder(
        urdf_path=str(urdf),
        asset_path=str(asset),
        tool_frames=["tool0"],
    )

    # 拟合碰撞球
    print("\nFitting collision spheres...")
    builder.fit_collision_spheres(
        sphere_density=sphere_density,
        compute_metrics=compute_metrics,
    )

    print(f"Fitted {builder.num_spheres} spheres across {len(builder.collision_link_names)} links")

    # 计算碰撞矩阵
    print("\nComputing collision matrix...")
    builder.compute_collision_matrix(
        num_samples=num_collision_samples,
    )

    print(f"Created collision ignore matrix with {len(builder.collision_matrix)} entries")

    # 输出质量指标
    if compute_metrics and builder.link_metrics:
        print(f"\n{'Link':<35s} {'n_sph':>5s} {'cover%':>7s} {'protr%':>7s}")
        print("-" * 60)
        for link_name, m in builder.link_metrics.items():
            print(
                f"{link_name:<35s} {m.num_spheres:5d} "
                f"{m.coverage * 100:6.1f}% {m.protrusion * 100:6.1f}%"
            )

    # 修正单位问题
    # URDF 中 mesh 使用 scale="0.001" (毫米转米)，但 RobotBuilder 可能没有正确应用
    # 检查第一个球的坐标，如果过大则自动缩放
    raw_spheres = builder.collision_spheres
    if raw_spheres:
        first_link = list(raw_spheres.keys())[0]
        if raw_spheres[first_link]:
            first_sphere = raw_spheres[first_link][0]
            center = first_sphere["center"]
            radius = first_sphere["radius"]
            # 正常的碰撞球坐标应该在 1 米以内，如果超过 10 则认为是毫米单位
            max_coord = max(abs(c) for c in center) if center else 0
            if max_coord > 10.0 or radius > 10.0:
                print(f"\nDetected millimeter units (max_coord={max_coord:.2f}), converting to meters...")
                for link_name in raw_spheres:
                    for sphere in raw_spheres[link_name]:
                        sphere["center"] = [c / 1000.0 for c in sphere["center"]]
                        sphere["radius"] = sphere["radius"] / 1000.0

    return raw_spheres


def resolve_robot_config_for_workspace(
    robot_config_path: Path | None = None,
    auto_generate_spheres: bool = True,
    sphere_density: float = 0.3,
) -> dict[str, Any]:
    """加载并归一化机器人配置。

    Args:
        robot_config_path: 可选的机器人配置 YAML 路径。为 None 时使用默认路径。
        auto_generate_spheres: 如果为 True，使用 CuRobo V2 自动生成碰撞球（而非从文件加载）。
        sphere_density: 自动生成时的球密度倍数。

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

    if auto_generate_spheres:
        # 使用 CuRobo V2 自动生成
        print("Using CuRobo V2 to generate collision spheres...")
        urdf_path = Path(kinematics_cfg.get("urdf_path", workspace_urdf_path()))
        asset_path = Path(kinematics_cfg.get("asset_root_path", WORKSPACE_CUROBO_ROOT))

        kinematics_cfg["collision_spheres"] = generate_collision_spheres(
            urdf_path=urdf_path,
            asset_path=asset_path,
            sphere_density=sphere_density,
        )
    elif isinstance(collision_spheres, str):
        # 从文件加载（传统方式）
        collision_spheres_path = Path(collision_spheres)
        if not collision_spheres_path.is_absolute():
            collision_spheres_path = (config_path.parent / collision_spheres_path).resolve()

        if not collision_spheres_path.exists():
            # 文件不存在时，自动生成
            print(f"Spheres file not found: {collision_spheres_path}")
            print("Auto-generating using CuRobo V2...")

            urdf_path = Path(kinematics_cfg.get("urdf_path", workspace_urdf_path()))
            asset_path = Path(kinematics_cfg.get("asset_root_path", WORKSPACE_CUROBO_ROOT))

            kinematics_cfg["collision_spheres"] = generate_collision_spheres(
                urdf_path=urdf_path,
                asset_path=asset_path,
                sphere_density=sphere_density,
            )
        else:
            kinematics_cfg["collision_spheres"] = load_yaml(collision_spheres_path)[
                "collision_spheres"
            ]

    return robot_cfg
