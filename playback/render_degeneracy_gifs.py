#!/usr/bin/env python3
"""
渲染 PointToPoint 退化路径的 MuJoCo GIF 动画。

用法（容器内 curoboV2 环境）:
    cd /home/tanshan/rep/curoboV2_demo
    conda activate curoboV2
    export MUJOCO_GL=egl
    python playback/render_degeneracy_gifs.py \
        --input /home/tanshan/rep/tashan_robot/degeneracy_data/degeneracy_paths.json \
        --output-dir /home/tanshan/rep/tashan_robot/degeneracy_gifs

输出:
    每条退化路径的 GIF 动画和首尾 PNG
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import imageio.v2 as imageio
import mujoco
import numpy as np


def _parse_triplet(text: str | None, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if text is None:
        return default
    values = [float(x) for x in text.split()]
    return (values[0], values[1], values[2])


def _format_triplet(values: tuple[float, float, float]) -> str:
    return " ".join(f"{v:.9g}" for v in values)


def _load_urdf_model(urdf_path: Path):
    """加载 URDF 模型，返回 links、joints 等信息。"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    links = {}
    joints = {}
    children_by_parent = {}
    child_links = set()

    for link_tag in root.findall("link"):
        name = link_tag.attrib["name"]
        links[name] = {'name': name, 'mass': 0.1, 'diaginertia': (0.001, 0.001, 0.001)}

    for joint_tag in root.findall("joint"):
        name = joint_tag.attrib["name"]
        joint_type = joint_tag.attrib.get("type", "revolute")
        parent = joint_tag.find("parent").attrib["link"]
        child = joint_tag.find("child").attrib["link"]

        origin_tag = joint_tag.find("origin")
        axis_tag = joint_tag.find("axis")
        limit_tag = joint_tag.find("limit")

        limit = None
        if limit_tag is not None:
            limit = (float(limit_tag.attrib["lower"]), float(limit_tag.attrib["upper"]))

        joint_info = {
            'name': name,
            'joint_type': joint_type,
            'parent': parent,
            'child': child,
            'xyz': _parse_triplet(origin_tag.attrib.get("xyz") if origin_tag is not None else None, (0.0, 0.0, 0.0)),
            'rpy': _parse_triplet(origin_tag.attrib.get("rpy") if origin_tag is not None else None, (0.0, 0.0, 0.0)),
            'axis': _parse_triplet(axis_tag.attrib.get("xyz") if axis_tag is not None else None, (0.0, 0.0, 1.0)),
            'limit': limit,
            'damping': 1.0,
        }
        joints[name] = joint_info
        children_by_parent.setdefault(parent, []).append(name)
        child_links.add(child)

    root_links = set(links) - child_links
    if len(root_links) != 1:
        raise ValueError(f"expected exactly one root link, got {sorted(root_links)}")
    root_link = next(iter(root_links))
    return links, joints, children_by_parent, root_link


def _build_link_body(body_parent, link_name, links, joints, children_by_parent):
    """递归构建 MJCF body 树。"""
    for joint_name in children_by_parent.get(link_name, []):
        joint = joints[joint_name]
        child_body = ET.SubElement(
            body_parent,
            "body",
            name=joint['child'],
            pos=_format_triplet(joint['xyz']),
            euler=_format_triplet(joint['rpy']),
        )

        ET.SubElement(
            child_body,
            "inertial",
            pos="0 0 0",
            mass="0.1",
            diaginertia="0.001 0.001 0.001",
        )

        if joint['joint_type'] != "fixed":
            joint_attrib = {
                "name": joint['name'],
                "type": "hinge" if joint['joint_type'] == "revolute" else "slide",
                "axis": _format_triplet(joint['axis']),
                "damping": "1.0",
            }
            if joint['limit'] is not None:
                joint_attrib["range"] = f"{joint['limit'][0]:.9g} {joint['limit'][1]:.9g}"
            ET.SubElement(child_body, "joint", joint_attrib)

        # 添加简单几何体用于可视化
        ET.SubElement(
            child_body,
            "geom",
            name=f"{joint['child']}_geom",
            type="capsule",
            size="0.03",
            fromto="0 0 0 0 0 0.1",
            rgba="0.77 0.79 0.84 1",
            contype="0",
            conaffinity="0",
        )

        if joint['child'] == "tool0" or joint['child'] == "link6":
            ET.SubElement(
                child_body,
                "site",
                name="tool0_site",
                pos="0 0 0",
                size="0.012",
                rgba="1 0.2 0.2 1",
            )

        _build_link_body(child_body, joint['child'], links, joints, children_by_parent)


def generate_mjcf(output_xml_path: Path, urdf_path: Path) -> Path:
    """从 URDF 生成 MJCF 文件。"""
    links, joints, children_by_parent, root_link = _load_urdf_model(urdf_path)

    model = ET.Element("mujoco", model="rokae_cr7_degeneracy")
    ET.SubElement(model, "compiler", angle="radian", autolimits="true", balanceinertia="true")
    ET.SubElement(model, "option", timestep="0.002", gravity="0 0 -9.81")
    visual = ET.SubElement(model, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="960")

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


def render_trajectory_gif(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joint_names: list[str],
    waypoints: list[list[float]],
    output_dir: Path,
    path_name: str,
    render_every: int = 2,
) -> dict:
    """渲染单条轨迹的 GIF 动画。"""
    renderer = mujoco.Renderer(model, height=720, width=960)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.azimuth = 60.0
    camera.elevation = -20.0
    camera.distance = 1.9
    camera.lookat[:] = np.array([0.0, -0.45, 0.45])

    # 获取关节地址
    qpos_mapping = []
    for joint_name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            print(f"  警告: 关节 {joint_name} 未找到，跳过")
            continue
        qpos_mapping.append({
            'joint_id': joint_id,
            'qpos_adr': model.jnt_qposadr[joint_id],
        })

    if not qpos_mapping:
        renderer.close()
        return {'error': 'no valid joints found'}

    frames = []
    first_frame = None
    last_frame = None

    for wp_idx, waypoint in enumerate(waypoints):
        # 设置关节位置
        for i, mapping in enumerate(qpos_mapping):
            if i < len(waypoint):
                data.qpos[mapping['qpos_adr']] = waypoint[i]

        mujoco.mj_forward(model, data)

        # 渲染帧
        if wp_idx % render_every == 0 or wp_idx == len(waypoints) - 1:
            renderer.update_scene(data, camera=camera)
            pixels = renderer.render().copy()
            if first_frame is None:
                first_frame = pixels
            last_frame = pixels
            frames.append(pixels)

    renderer.close()

    if not frames:
        return {'error': 'no frames rendered'}

    # 保存文件
    output_dir.mkdir(parents=True, exist_ok=True)
    gif_path = output_dir / f"{path_name}.gif"
    start_png = output_dir / f"{path_name}_start.png"
    end_png = output_dir / f"{path_name}_end.png"

    imageio.imwrite(start_png, first_frame)
    imageio.imwrite(end_png, last_frame)
    imageio.mimsave(gif_path, frames, duration=0.05, loop=0)

    return {
        'gif_path': str(gif_path),
        'start_png': str(start_png),
        'end_png': str(end_png),
        'frame_count': len(frames),
    }


def main():
    parser = argparse.ArgumentParser(description="渲染退化路径 GIF")
    parser.add_argument("--input", required=True, help="退化路径 JSON 文件")
    parser.add_argument("--output-dir", required=True, help="GIF 输出目录")
    parser.add_argument("--render-every", type=int, default=2, help="每隔几个 waypoint 渲染一帧")
    args = parser.parse_args()

    input_path = Path(args.input)
    output_dir = Path(args.output_dir)

    # 加载退化路径数据
    with open(input_path) as f:
        data = json.load(f)

    urdf_path = Path(data['urdf_path'])
    joint_names = data['joint_names']
    paths = data['paths']

    print(f"加载了 {len(paths)} 条退化路径")
    print(f"URDF: {urdf_path}")

    # 生成 MJCF
    mjcf_path = generate_mjcf(output_dir / "rokae_cr7.xml", urdf_path)
    print(f"MJCF: {mjcf_path}")

    # 加载 MuJoCo 模型
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    mj_data = mujoco.MjData(model)

    # 渲染每条路径
    results = []
    for i, path in enumerate(paths):
        path_name = path['name']
        waypoints = path['waypoints']
        j1_range = path['j1_range']
        j1_crosses_pi = path['j1_crosses_pi']

        print(f"\n[{i+1}/{len(paths)}] {path_name}  "
              f"J1={j1_range:.2f}rad  "
              f"{'★J1跨π' if j1_crosses_pi else ''}  "
              f"{len(waypoints)} waypoints")

        result = render_trajectory_gif(
            model, mj_data,
            joint_names, waypoints,
            output_dir, path_name,
            render_every=args.render_every,
        )

        if 'error' in result:
            print(f"  错误: {result['error']}")
        else:
            print(f"  GIF: {result['gif_path']}  ({result['frame_count']} 帧)")
            results.append({
                'name': path_name,
                'j1_range': j1_range,
                'j1_crosses_pi': j1_crosses_pi,
                **result,
            })

    # 保存汇总
    summary_path = output_dir / "summary.json"
    with open(summary_path, 'w') as f:
        json.dump({
            'total_paths': len(paths),
            'rendered': len(results),
            'results': results,
        }, f, indent=2)

    print(f"\n完成! 渲染了 {len(results)} 条路径")
    print(f"汇总: {summary_path}")


if __name__ == "__main__":
    main()
