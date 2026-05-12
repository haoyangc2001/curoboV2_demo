#!/usr/bin/env python3
"""Replay a stage-1 Franka playback contract directly in MuJoCo."""

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
CUROBO_FRANKA_DIR = (
    WORKSPACE_ROOT
    / "third_party"
    / "curobo"
    / "curobo"
    / "content"
    / "assets"
    / "robot"
    / "franka_description"
)
FRANKA_URDF_PATH = CUROBO_FRANKA_DIR / "franka_panda.urdf"


@dataclass
class LinkInfo:
    name: str
    collision_meshes: list[Path]
    mass: float
    diaginertia: tuple[float, float, float]


@dataclass
class JointInfo:
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
    if text is None:
        return default
    values = [float(x) for x in text.split()]
    if len(values) != 3:
        raise ValueError(f"expected three floats, got {text!r}")
    return values[0], values[1], values[2]


def _format_triplet(values: tuple[float, float, float]) -> str:
    return " ".join(f"{v:.9g}" for v in values)


def _load_urdf_model(urdf_path: Path) -> tuple[dict[str, LinkInfo], dict[str, JointInfo], dict[str, list[str]], str]:
    root = ET.fromstring(urdf_path.read_text())

    links: dict[str, LinkInfo] = {}
    for link in root.findall("link"):
        name = link.attrib["name"]
        collision_meshes = []
        for collision in link.findall("collision"):
            mesh = collision.find("./geometry/mesh")
            if mesh is None:
                continue
            collision_meshes.append((urdf_path.parent / mesh.attrib["filename"]).resolve())

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
        dynamics_tag = joint.find("dynamics")
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
            damping=float(dynamics_tag.attrib.get("damping", 1.0)) if dynamics_tag is not None else 1.0,
        )
        joints[joint_info.name] = joint_info
        children_by_parent.setdefault(parent, []).append(joint_info.name)
        child_links.add(child)

    root_links = set(links) - child_links
    if len(root_links) != 1:
        raise ValueError(f"expected exactly one root link, got {sorted(root_links)}")
    root_link = next(iter(root_links))
    return links, joints, children_by_parent, root_link


def _mesh_asset_name(mesh_path: Path) -> str:
    return f"{mesh_path.stem}_mesh"


def _build_link_body(
    body_parent: ET.Element,
    link_name: str,
    links: dict[str, LinkInfo],
    joints: dict[str, JointInfo],
    children_by_parent: dict[str, list[str]],
) -> None:
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

        for mesh_path in link_info.collision_meshes:
            ET.SubElement(
                child_body,
                "geom",
                name=f"{joint.child}_{mesh_path.stem}_geom",
                type="mesh",
                mesh=_mesh_asset_name(mesh_path),
                rgba="0.77 0.79 0.84 1",
                contype="0",
                conaffinity="0",
                group="1",
            )

        if joint.child == "panda_hand":
            ET.SubElement(
                child_body,
                "site",
                name="panda_hand_site",
                pos="0 0 0",
                size="0.012",
                rgba="1 0.2 0.2 1",
            )

        _build_link_body(child_body, joint.child, links, joints, children_by_parent)


def generate_franka_stage1_mjcf(output_xml_path: Path) -> Path:
    links, joints, children_by_parent, root_link = _load_urdf_model(FRANKA_URDF_PATH)

    model = ET.Element("mujoco", model="franka_stage1_playback")
    ET.SubElement(model, "compiler", angle="radian", autolimits="true", balanceinertia="true")
    ET.SubElement(model, "option", timestep="0.002", gravity="0 0 -9.81")
    visual = ET.SubElement(model, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="960")

    asset = ET.SubElement(model, "asset")
    unique_mesh_paths = {
        mesh_path
        for link in links.values()
        for mesh_path in link.collision_meshes
    }
    for mesh_path in sorted(unique_mesh_paths):
        ET.SubElement(
            asset,
            "mesh",
            name=_mesh_asset_name(mesh_path),
            file=str(mesh_path),
            scale="1 1 1",
        )

    worldbody = ET.SubElement(model, "worldbody")
    ET.SubElement(worldbody, "light", pos="1.5 0 2.5", dir="-1 0 -1")
    ET.SubElement(worldbody, "geom", name="ground", type="plane", size="2 2 0.1", rgba="0.94 0.94 0.96 1")

    root_body = ET.SubElement(worldbody, "body", name=root_link, pos="0 0 0")
    ET.SubElement(root_body, "inertial", pos="0 0 0", mass="0.1", diaginertia="0.001 0.001 0.001")
    _build_link_body(root_body, root_link, links, joints, children_by_parent)

    tree = ET.ElementTree(model)
    output_xml_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_xml_path, encoding="utf-8", xml_declaration=True)
    return output_xml_path


def _load_contract(contract_path: Path) -> dict[str, Any]:
    return json.loads(contract_path.read_text())


def _resolve_qpos_addresses(model: mujoco.MjModel, joint_names: list[str]) -> list[dict[str, Any]]:
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


def _camera_for_franka(model: mujoco.MjModel) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.azimuth = 135.0
    camera.elevation = -25.0
    camera.distance = 2.2
    camera.lookat[:] = np.array([0.0, 0.0, 0.6])
    return camera


def _render_frames(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos_mapping: list[dict[str, Any]],
    waypoints: list[list[float]],
    output_dir: Path,
    dt: float,
    render_every: int,
) -> dict[str, Any]:
    renderer = mujoco.Renderer(model, height=720, width=960)
    camera = _camera_for_franka(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "panda_hand")
    if body_id < 0:
        raise ValueError("panda_hand body is missing from MuJoCo model")

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

    return {
        "frame_count": len(frames),
        "render_every": render_every,
        "ee_positions": ee_positions,
        "ee_start": ee_start,
        "ee_end": ee_end,
        "ee_displacement": ee_displacement,
        "gif_path": str(gif_path),
        "start_png": str(start_png),
        "end_png": str(end_png),
    }


def replay_contract(contract_path: Path, output_dir: Path, render_every: int) -> dict[str, Any]:
    contract = _load_contract(contract_path)
    joint_names = list(contract["joint_contract"]["mujoco_expected_joint_names"])
    waypoints = list(contract["trajectory_contract"]["waypoints"])
    dt = float(contract["timing_contract"]["sample_period_s"])

    model_xml_path = generate_franka_stage1_mjcf(output_dir / "franka_stage1_playback.xml")
    model = mujoco.MjModel.from_xml_path(str(model_xml_path))
    data = mujoco.MjData(model)

    qpos_mapping = _resolve_qpos_addresses(model, joint_names)
    render_summary = _render_frames(model, data, qpos_mapping, waypoints, output_dir, dt, render_every)

    summary = {
        "success": True,
        "contract_path": str(contract_path),
        "model_xml_path": str(model_xml_path),
        "mujoco_version": mujoco.__version__,
        "joint_names": joint_names,
        "qpos_mapping": qpos_mapping,
        "waypoint_count": len(waypoints),
        "sample_period_s": dt,
        "trajectory_span_s": float(contract["timing_contract"]["trajectory_span_s"]),
        "render_summary": render_summary,
        "checks": {
            "all_contract_joints_resolved": True,
            "waypoint_width_matches_joint_count": True,
            "ee_displacement_gt_0_05m": render_summary["ee_displacement"] > 0.05,
        },
    }
    (output_dir / "playback_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay an official Franka playback contract in MuJoCo")
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


if __name__ == "__main__":
    main()
