#!/usr/bin/env python3
"""在 MuJoCo 中直接回放大花复合末端阶段一合同。

本文件负责把工作区内的 URDF 动态转换为 MJCF，加载合同中的关节轨迹，
完成离屏渲染，并输出末端运动一致性检查结果。
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

import imageio.v2 as imageio
import mujoco
import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROKAE_URDF_PATH = (
    WORKSPACE_ROOT / "robot_assets" / "ROKAE" / "robot" / "curobo" / "ROKAE_SR5_0.9C.urdf"
)


@dataclass
class CollisionMeshInfo:
    """记录单个碰撞网格的路径和缩放信息。"""

    path: Path
    scale: tuple[float, float, float]


@dataclass
class LinkInfo:
    """描述 URDF 中一个 link 的碰撞与惯性信息。"""

    name: str
    collision_meshes: list[CollisionMeshInfo]
    mass: float
    diaginertia: tuple[float, float, float]


@dataclass
class JointInfo:
    """描述 URDF 中一个 joint 的拓扑与运动学属性。"""

    name: str
    joint_type: str
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    limit: tuple[float, float] | None
    damping: float


def _parse_triplet(text: str | None, default: tuple[float, float, float]) -> tuple[float, float, float]:
    """把 URDF/MJCF 中的三元字符串解析为浮点三元组。

    Args:
        text: 类似 `"x y z"` 的字符串；为空时使用默认值。
        default: 默认三元组。

    Returns:
        长度为 3 的浮点数元组。
    """
    if text is None:
        return default
    values = [float(x) for x in text.split()]
    if len(values) != 3:
        raise ValueError(f"expected three floats, got {text!r}")
    return values[0], values[1], values[2]


def _format_triplet(values: tuple[float, float, float]) -> str:
    """把浮点三元组格式化为 MJCF 属性字符串。

    Args:
        values: 三元浮点数。

    Returns:
        以空格分隔的字符串表示。
    """
    return " ".join(f"{v:.9g}" for v in values)


def _mesh_asset_name(mesh_path: Path) -> str:
    """为网格文件生成稳定的 MJCF asset 名称。

    Args:
        mesh_path: 网格文件路径。

    Returns:
        基于文件名生成的 mesh asset 名称。
    """
    return f"{mesh_path.stem}_mesh"


def _load_urdf_model(
    urdf_path: Path,
) -> tuple[dict[str, LinkInfo], dict[str, JointInfo], dict[str, list[str]], str]:
    """解析 URDF，提取 link/joint 树结构。

    Args:
        urdf_path: 待转换的 URDF 文件路径。

    Returns:
        四元组：
        1. link 名到 `LinkInfo` 的映射。
        2. joint 名到 `JointInfo` 的映射。
        3. 父 link 到子 joint 名列表的映射。
        4. 根 link 名称。
    """
    root = ET.fromstring(urdf_path.read_text())

    links: dict[str, LinkInfo] = {}
    for link in root.findall("link"):
        name = link.attrib["name"]
        collision_meshes = []
        for collision in link.findall("collision"):
            mesh = collision.find("./geometry/mesh")
            if mesh is None:
                continue
            collision_meshes.append(
                CollisionMeshInfo(
                    path=(urdf_path.parent / mesh.attrib["filename"]).resolve(),
                    scale=_parse_triplet(mesh.attrib.get("scale"), (1.0, 1.0, 1.0)),
                )
            )

        inertial = link.find("inertial")
        mass = 0.1
        diaginertia = (1e-3, 1e-3, 1e-3)
        if inertial is not None:
            mass_tag = inertial.find("mass")
            inertia_tag = inertial.find("inertia")
            if mass_tag is not None:
                mass = max(float(mass_tag.attrib["value"]), 1e-4)
            if inertia_tag is not None:
                diaginertia = (
                    max(float(inertia_tag.attrib.get("ixx", 0.0)), 1e-6),
                    max(float(inertia_tag.attrib.get("iyy", 0.0)), 1e-6),
                    max(float(inertia_tag.attrib.get("izz", 0.0)), 1e-6),
                )

        links[name] = LinkInfo(
            name=name,
            collision_meshes=collision_meshes,
            mass=mass,
            diaginertia=diaginertia,
        )

    joints: dict[str, JointInfo] = {}
    children_by_parent: dict[str, list[str]] = {}
    child_links: set[str] = set()
    for joint in root.findall("joint"):
        parent_tag = joint.find("parent")
        child_tag = joint.find("child")
        if parent_tag is None or child_tag is None:
            raise ValueError(f"joint {joint.attrib.get('name')} is missing parent or child")
        origin_tag = joint.find("origin")
        axis_tag = joint.find("axis")
        limit_tag = joint.find("limit")
        parent = parent_tag.attrib["link"]
        child = child_tag.attrib["link"]
        limit = None
        if limit_tag is not None and "lower" in limit_tag.attrib and "upper" in limit_tag.attrib:
            limit = (float(limit_tag.attrib["lower"]), float(limit_tag.attrib["upper"]))

        joint_info = JointInfo(
            name=joint.attrib["name"],
            joint_type=joint.attrib["type"],
            parent=parent,
            child=child,
            xyz=_parse_triplet(origin_tag.attrib.get("xyz") if origin_tag is not None else None, (0.0, 0.0, 0.0)),
            rpy=_parse_triplet(origin_tag.attrib.get("rpy") if origin_tag is not None else None, (0.0, 0.0, 0.0)),
            axis=_parse_triplet(axis_tag.attrib.get("xyz") if axis_tag is not None else None, (0.0, 0.0, 1.0)),
            limit=limit,
            damping=1.0,
        )
        joints[joint_info.name] = joint_info
        children_by_parent.setdefault(parent, []).append(joint_info.name)
        child_links.add(child)

    root_links = set(links) - child_links
    if len(root_links) != 1:
        raise ValueError(f"expected exactly one root link, got {sorted(root_links)}")
    root_link = next(iter(root_links))
    return links, joints, children_by_parent, root_link


def _build_link_body(
    body_parent: ET.Element,
    link_name: str,
    links: dict[str, LinkInfo],
    joints: dict[str, JointInfo],
    children_by_parent: dict[str, list[str]],
) -> None:
    """递归把 URDF link 树写成 MJCF body 树。

    Args:
        body_parent: 当前要写入子节点的 XML 元素。
        link_name: 当前父 link 名称。
        links: link 信息映射。
        joints: joint 信息映射。
        children_by_parent: 父 link 到子 joint 的映射。

    Returns:
        无返回值；结果直接写入 XML 树。
    """
    for joint_name in children_by_parent.get(link_name, []):
        joint = joints[joint_name]
        child_body = ET.SubElement(
            body_parent,
            "body",
            name=joint.child,
            pos=_format_triplet(joint.xyz),
            euler=_format_triplet(joint.rpy),
        )
        link_info = links[joint.child]
        ET.SubElement(
            child_body,
            "inertial",
            pos="0 0 0",
            mass=f"{link_info.mass:.9g}",
            diaginertia=_format_triplet(link_info.diaginertia),
        )

        if joint.joint_type != "fixed":
            joint_attrib = {
                "name": joint.name,
                "type": "hinge" if joint.joint_type == "revolute" else "slide",
                "axis": _format_triplet(joint.axis),
                "damping": f"{max(joint.damping, 1e-3):.9g}",
            }
            if joint.limit is not None:
                joint_attrib["range"] = f"{joint.limit[0]:.9g} {joint.limit[1]:.9g}"
            ET.SubElement(child_body, "joint", joint_attrib)

        for mesh_info in link_info.collision_meshes:
            ET.SubElement(
                child_body,
                "geom",
                name=f"{joint.child}_{mesh_info.path.stem}_geom",
                type="mesh",
                mesh=_mesh_asset_name(mesh_info.path),
                rgba="0.77 0.79 0.84 1",
                contype="0",
                conaffinity="0",
                group="1",
            )

        if joint.child == "tool0":
            ET.SubElement(
                child_body,
                "site",
                name="tool0_site",
                pos="0 0 0",
                size="0.012",
                rgba="1 0.2 0.2 1",
            )

        _build_link_body(child_body, joint.child, links, joints, children_by_parent)


def generate_rokae_stage1_mjcf(output_xml_path: Path, urdf_path: Path | None = None) -> Path:
    """根据 URDF 生成可直接加载的 MJCF 文件。

    Args:
        output_xml_path: 输出 MJCF 文件路径。
        urdf_path: 可选 URDF 路径；为空时使用默认大花复合末端 URDF。

    Returns:
        生成后的 MJCF 文件路径。
    """
    if urdf_path is None:
        urdf_path = DEFAULT_ROKAE_URDF_PATH
    links, joints, children_by_parent, root_link = _load_urdf_model(urdf_path)

    model = ET.Element("mujoco", model="rokae_stage1_playback")
    ET.SubElement(model, "compiler", angle="radian", autolimits="true", balanceinertia="true")
    ET.SubElement(model, "option", timestep="0.002", gravity="0 0 -9.81")
    visual = ET.SubElement(model, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="960")

    asset = ET.SubElement(model, "asset")
    unique_meshes: dict[Path, tuple[float, float, float]] = {}
    for link in links.values():
        for mesh_info in link.collision_meshes:
            unique_meshes.setdefault(mesh_info.path, mesh_info.scale)

    for mesh_path in sorted(unique_meshes):
        ET.SubElement(
            asset,
            "mesh",
            name=_mesh_asset_name(mesh_path),
            file=str(mesh_path),
            scale=_format_triplet(unique_meshes[mesh_path]),
        )

    worldbody = ET.SubElement(model, "worldbody")
    ET.SubElement(worldbody, "light", pos="1.5 -0.5 2.5", dir="-1 0 -1")
    ET.SubElement(worldbody, "geom", name="ground", type="plane", size="2 2 0.1", rgba="0.94 0.94 0.96 1")

    if root_link == "world":
        _build_link_body(worldbody, root_link, links, joints, children_by_parent)
    else:
        root_body = ET.SubElement(worldbody, "body", name=root_link, pos="0 0 0")
        ET.SubElement(root_body, "inertial", pos="0 0 0", mass="0.1", diaginertia="0.001 0.001 0.001")
        _build_link_body(root_body, root_link, links, joints, children_by_parent)

    tree = ET.ElementTree(model)
    output_xml_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_xml_path, encoding="utf-8", xml_declaration=True)
    return output_xml_path


def _load_contract(contract_path: Path) -> dict[str, Any]:
    """读取回放合同 JSON。

    Args:
        contract_path: 合同文件路径。

    Returns:
        解析后的合同字典。
    """
    return json.loads(contract_path.read_text())


def _resolve_qpos_addresses(model: mujoco.MjModel, joint_names: list[str]) -> list[dict[str, Any]]:
    """解析合同关节名在 MuJoCo `qpos` 中的地址。

    Args:
        model: 已加载的 MuJoCo 模型。
        joint_names: 合同要求的关节名列表。

    Returns:
        每个关节对应的 joint id、qpos 地址和类型信息列表。
    """
    mapping = []
    for joint_name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise ValueError(f"joint {joint_name} is missing from MuJoCo model")
        mapping.append(
            {
                "joint_name": joint_name,
                "joint_id": int(joint_id),
                "qpos_adr": int(model.jnt_qposadr[joint_id]),
                "joint_type": int(model.jnt_type[joint_id]),
            }
        )
    return mapping


def _camera_for_rokae(model: mujoco.MjModel) -> mujoco.MjvCamera:
    """创建适合大花复合末端回放视角的相机。

    Args:
        model: MuJoCo 模型。

    Returns:
        已配置好的自由相机对象。
    """
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.azimuth = 60.0
    camera.elevation = -20.0
    camera.distance = 1.9
    camera.lookat[:] = np.array([0.0, -0.45, 0.45])
    return camera


def _render_frames(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos_mapping: list[dict[str, Any]],
    waypoints: list[list[float]],
    output_dir: Path,
    dt: float,
    render_every: int,
    *,
    ee_body_name: str,
    expected_start_ee: list[float] | None,
    expected_end_ee: list[float] | None,
    expected_goal_delta_xyz: list[float] | None,
) -> dict[str, Any]:
    """按合同轨迹渲染回放帧并保存 GIF/首尾图。

    Args:
        model: MuJoCo 模型。
        data: MuJoCo 仿真数据。
        qpos_mapping: 关节名到 qpos 地址的映射。
        waypoints: 关节轨迹序列。
        output_dir: 图像输出目录。
        dt: 合同采样周期。
        render_every: 每隔多少个 waypoint 渲染一帧。
        ee_body_name: 末端 body 名称。
        expected_start_ee: 可选合同起始末端位置。
        expected_end_ee: 可选合同结束末端位置。
        expected_goal_delta_xyz: 可选合同目标位移，用于方向一致性检查。

    Returns:
        包含渲染文件路径、末端轨迹和一致性指标的摘要字典。
    """
    renderer = mujoco.Renderer(model, height=720, width=960)
    camera = _camera_for_rokae(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name)
    if body_id < 0:
        raise ValueError(f"{ee_body_name} body is missing from MuJoCo model")

    frames = []
    ee_positions = []
    first_frame = None
    last_frame = None

    for waypoint_index, waypoint in enumerate(waypoints):
        if len(waypoint) != len(qpos_mapping):
            raise ValueError(
                f"waypoint {waypoint_index} has {len(waypoint)} joints, expected {len(qpos_mapping)}"
            )
        for target, mapping in zip(waypoint, qpos_mapping):
            data.qpos[mapping["qpos_adr"]] = target

        mujoco.mj_forward(model, data)
        ee_positions.append(data.xpos[body_id].copy().tolist())

        if waypoint_index % render_every != 0 and waypoint_index != len(waypoints) - 1:
            continue

        renderer.update_scene(data, camera=camera)
        pixels = renderer.render().copy()
        if first_frame is None:
            first_frame = pixels
        last_frame = pixels
        frames.append(pixels)

    renderer.close()

    if not frames or first_frame is None or last_frame is None:
        raise ValueError("no rendered frames were produced")

    start_png = output_dir / "playback_start.png"
    end_png = output_dir / "playback_end.png"
    gif_path = output_dir / "playback.gif"

    imageio.imwrite(start_png, first_frame)
    imageio.imwrite(end_png, last_frame)
    imageio.mimsave(gif_path, frames, duration=max(dt * render_every, 1e-3), loop=0)

    ee_start = ee_positions[0]
    ee_end = ee_positions[-1]
    ee_displacement = math.dist(ee_start, ee_end)

    observed_delta = [ee_end[i] - ee_start[i] for i in range(3)]
    direction_alignment = None
    if expected_goal_delta_xyz is not None:
        expected_norm = math.sqrt(sum(v * v for v in expected_goal_delta_xyz))
        observed_norm = math.sqrt(sum(v * v for v in observed_delta))
        if expected_norm > 0.0 and observed_norm > 0.0:
            dot = sum(a * b for a, b in zip(observed_delta, expected_goal_delta_xyz))
            direction_alignment = dot / (expected_norm * observed_norm)

    expected_start_error = (
        math.dist(ee_start, expected_start_ee) if expected_start_ee is not None else None
    )
    expected_end_error = (
        math.dist(ee_end, expected_end_ee) if expected_end_ee is not None else None
    )

    return {
        "frame_count": len(frames),
        "render_every": render_every,
        "ee_positions": ee_positions,
        "ee_start": ee_start,
        "ee_end": ee_end,
        "ee_displacement": ee_displacement,
        "ee_delta_xyz": observed_delta,
        "ee_direction_alignment": direction_alignment,
        "expected_start_ee_error_norm": expected_start_error,
        "expected_end_ee_error_norm": expected_end_error,
        "gif_path": str(gif_path),
        "start_png": str(start_png),
        "end_png": str(end_png),
    }


def replay_contract(contract_path: Path, output_dir: Path, render_every: int) -> dict[str, Any]:
    """执行一次 MuJoCo 离屏回放。

    Args:
        contract_path: 回放合同 JSON 路径。
        output_dir: 回放产物输出目录。
        render_every: 每隔多少个 waypoint 渲染一帧。

    Returns:
        包含模型路径、回放统计、检查项和渲染摘要的结果字典。
    """
    contract = _load_contract(contract_path)
    joint_names = list(contract["joint_contract"]["mujoco_expected_joint_names"])
    waypoints = list(contract["trajectory_contract"]["waypoints"])
    dt = float(contract["timing_contract"]["sample_period_s"])
    ee_body_name = str(contract["robot_source"]["ee_body_name"])
    urdf_path = Path(contract["robot_source"]["urdf_path"])

    model_xml_path = generate_rokae_stage1_mjcf(
        output_dir / "rokae_stage1_playback.xml",
        urdf_path=urdf_path,
    )
    model = mujoco.MjModel.from_xml_path(str(model_xml_path))
    data = mujoco.MjData(model)

    qpos_mapping = _resolve_qpos_addresses(model, joint_names)
    render_summary = _render_frames(
        model,
        data,
        qpos_mapping,
        waypoints,
        output_dir,
        dt,
        render_every,
        ee_body_name=ee_body_name,
        expected_start_ee=contract["source"].get("start_tool_position"),
        expected_end_ee=contract["source"].get("final_tool_position"),
        expected_goal_delta_xyz=contract["source"].get("goal_delta_xyz"),
    )

    checks = {
        "all_contract_joints_resolved": True,
        "waypoint_width_matches_joint_count": True,
        "ee_displacement_gt_0_02m": render_summary["ee_displacement"] > 0.02,
        "ee_direction_alignment_gt_0_95": (
            render_summary["ee_direction_alignment"] is not None
            and render_summary["ee_direction_alignment"] > 0.95
        ),
        "start_ee_matches_contract_lt_1e_4m": (
            render_summary["expected_start_ee_error_norm"] is not None
            and render_summary["expected_start_ee_error_norm"] < 1e-4
        ),
        "end_ee_matches_contract_lt_1e_4m": (
            render_summary["expected_end_ee_error_norm"] is not None
            and render_summary["expected_end_ee_error_norm"] < 1e-4
        ),
    }

    summary = {
        "success": all(checks.values()),
        "contract_path": str(contract_path),
        "urdf_path": str(urdf_path),
        "model_xml_path": str(model_xml_path),
        "mujoco_version": mujoco.__version__,
        "joint_names": joint_names,
        "qpos_mapping": qpos_mapping,
        "waypoint_count": len(waypoints),
        "sample_period_s": dt,
        "trajectory_span_s": float(contract["timing_contract"]["trajectory_span_s"]),
        "render_summary": render_summary,
        "checks": checks,
    }
    (output_dir / "playback_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )
    return summary


def main() -> None:
    """命令行入口。

    Returns:
        无返回值；成功时打印回放统计信息。
    """
    parser = argparse.ArgumentParser(description="Replay a ROKAE playback contract in MuJoCo")
    parser.add_argument("--contract-json", type=Path, required=True, help="Path to playback_contract.json")
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory for playback evidence")
    parser.add_argument(
        "--render-every",
        type=int,
        default=1,
        help="Render every Nth waypoint while always keeping the final frame",
    )
    args = parser.parse_args()

    if args.render_every <= 0:
        raise SystemExit("--render-every must be positive")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary = replay_contract(args.contract_json, args.output_dir, args.render_every)
    print("MuJoCo playback succeeded")
    print(f"Output dir: {args.output_dir}")
    print(f"Waypoints: {summary['waypoint_count']}")
    print(f"Sample period: {summary['sample_period_s']:.6f}s")
    print(f"Rendered frames: {summary['render_summary']['frame_count']}")
    print(f"End-effector displacement: {summary['render_summary']['ee_displacement']:.6f}m")
    print(f"Direction alignment: {summary['render_summary']['ee_direction_alignment']:.6f}")


if __name__ == "__main__":
    main()
