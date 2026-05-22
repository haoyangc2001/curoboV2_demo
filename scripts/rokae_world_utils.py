#!/usr/bin/env python3
"""障碍物 world 工具模块。

读取 abs.autosave.json / rel.autosave.json 格式的障碍物配置，
将相对障碍物变换到绝对坐标，并生成 CuRobo cuboid world dict。

JSON 格式约定（与 tashan_robot/dahuafuhe 保持一致）：
  {
    "frame_id": "Link_00",
    "boxes": [
      {"id": 0, "size": [x,y,z], "position": [x,y,z], "orientation": [w,qx,qy,qz]},
      ...
    ],
    "base_pose": [x,y,z,w,qx,qy,qz]   # rel 文件独有
  }

四元数顺序统一为 [w, x, y, z]。

# adapted from tashan_robot/src/trajectory_planning/trajectory_planning/main.py
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any


# ---------------------------------------------------------------------------
# 四元数工具（四元数顺序 [w, x, y, z]）
# ---------------------------------------------------------------------------


def _quaternion_multiply(q1: list[float], q2: list[float]) -> list[float]:
    """Hamilton 乘积 q1 * q2。"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]


def _rotate_vector_by_quaternion(q: list[float], v: list[float]) -> list[float]:
    """用四元数 q 旋转向量 v。q = [w, x, y, z]。"""
    qv = [0.0, v[0], v[1], v[2]]
    q_conj = [q[0], -q[1], -q[2], -q[3]]
    tmp = _quaternion_multiply(q, qv)
    result = _quaternion_multiply(tmp, q_conj)
    return [result[1], result[2], result[3]]


# ---------------------------------------------------------------------------
# JSON 加载
# ---------------------------------------------------------------------------


def load_abs_obstacles(json_path: Path) -> list[dict[str, Any]]:
    """读取绝对障碍物 JSON，返回 box 列表。

    每个 box: {"name": str, "dims": [x,y,z], "pose": [x,y,z,w,qx,qy,qz]}
    """
    data = json.loads(Path(json_path).read_text())
    boxes: list[dict[str, Any]] = []
    for item in data.get("boxes", []):
        boxes.append({
            "name": f"obstacle_{item['id']}",
            "dims": list(item["size"]),
            "pose": list(item["position"]) + list(item["orientation"]),
        })
    return boxes


def load_rel_obstacles(json_path: Path) -> list[dict[str, Any]]:
    """读取相对障碍物 JSON，变换到绝对坐标后返回 box 列表。

    如果缺少 base_pose 则返回空列表。
    """
    data = json.loads(Path(json_path).read_text())
    base_pose = data.get("base_pose")
    if not base_pose or len(base_pose) != 7:
        return []

    base_pos = base_pose[:3]
    base_quat = base_pose[3:]   # [w, x, y, z]

    boxes: list[dict[str, Any]] = []
    for item in data.get("boxes", []):
        rel_pos = list(item["position"])
        rel_quat = list(item["orientation"])

        rotated = _rotate_vector_by_quaternion(base_quat, rel_pos)
        abs_pos = [
            base_pos[0] + rotated[0],
            base_pos[1] + rotated[1],
            base_pos[2] + rotated[2],
        ]
        abs_quat = _quaternion_multiply(base_quat, rel_quat)

        boxes.append({
            "name": f"obstacle_rel_{item['id']}",
            "dims": list(item["size"]),
            "pose": abs_pos + abs_quat,
        })
    return boxes


# ---------------------------------------------------------------------------
# World 构建
# ---------------------------------------------------------------------------


def make_world_from_boxes(boxes: list[dict[str, Any]]) -> dict[str, Any]:
    """将 box 列表转换为 CuRobo cuboid world dict。

    返回: {"cuboid": {"name": {"dims": [...], "pose": [...]}, ...}}
    """
    cuboids: dict[str, Any] = {}
    for box in boxes:
        cuboids[box["name"]] = {
            "dims": box["dims"],
            "pose": box["pose"],
        }
    return {"cuboid": cuboids}


def build_world(
    abs_json_path: Path | None = None,
    rel_json_path: Path | None = None,
) -> dict[str, Any]:
    """从障碍物 JSON 文件构建 CuRobo world dict。

    Args:
        abs_json_path: 绝对障碍物 JSON 路径（可选）。
        rel_json_path: 相对障碍物 JSON 路径（可选）。

    Returns:
        {
            "boxes": [...],          # 合并后的 box 列表
            "world_dict": {...},     # CuRobo cuboid world dict
            "world_summary": {       # 摘要统计
                "abs_count": int,
                "rel_count": int,
                "total_count": int,
            },
        }
    """
    abs_boxes: list[dict[str, Any]] = []
    rel_boxes: list[dict[str, Any]] = []

    if abs_json_path is not None:
        abs_path = Path(abs_json_path)
        if abs_path.exists():
            abs_boxes = load_abs_obstacles(abs_path)

    if rel_json_path is not None:
        rel_path = Path(rel_json_path)
        if rel_path.exists():
            rel_boxes = load_rel_obstacles(rel_path)

    all_boxes = abs_boxes + rel_boxes
    world_dict = make_world_from_boxes(all_boxes)

    return {
        "boxes": all_boxes,
        "world_dict": world_dict,
        "world_summary": {
            "abs_count": len(abs_boxes),
            "rel_count": len(rel_boxes),
            "total_count": len(all_boxes),
        },
    }
