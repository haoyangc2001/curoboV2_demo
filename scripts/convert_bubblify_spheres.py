#!/usr/bin/env python3
"""Convert Bubblify sphere exports into the cuRobo collision_spheres format.

The project keeps the active sphere definition in a standalone YAML file so that
the robot config can point at it by path. This script validates a Bubblify-style
export and rewrites it into the grouped format used by cuRobo robot configs.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROBOT_CONFIG = WORKSPACE_ROOT / "robot_assets" / "ROKAE" / "robot" / "xms5_r800_w4g3b4c_dahuafuhe.yml"
DEFAULT_OUTPUT = WORKSPACE_ROOT / "robot_assets" / "ROKAE" / "robot" / "spheres" / "ROKAE_SR5_0.9C_spherized.yml"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert Bubblify YAML into cuRobo collision_spheres format")
    parser.add_argument("--input-yaml", type=Path, required=True, help="Raw Bubblify export YAML path")
    parser.add_argument(
        "--robot-config",
        type=Path,
        default=DEFAULT_ROBOT_CONFIG,
        help="Robot config YAML used to validate expected link names",
    )
    parser.add_argument(
        "--output-yaml",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Destination path for the grouped collision_spheres YAML",
    )
    return parser.parse_args()


def _load_yaml(path: Path) -> Any:
    if not path.exists():
        raise FileNotFoundError(f"YAML file not found: {path}")
    return yaml.safe_load(path.read_text())


def _normalize_bubblify_items(raw: Any) -> list[dict[str, Any]]:
    if isinstance(raw, dict):
        if isinstance(raw.get("spheres"), list):
            return raw["spheres"]
        if isinstance(raw.get("collision_spheres"), dict):
            items: list[dict[str, Any]] = []
            for link_name, spheres in raw["collision_spheres"].items():
                for sphere in spheres or []:
                    items.append(
                        {
                            "link": link_name,
                            "position": sphere.get("center"),
                            "radius": sphere.get("radius"),
                        }
                    )
            return items
    raise ValueError("Unsupported Bubblify YAML shape; expected top-level 'spheres' or 'collision_spheres'")


def _load_expected_links(robot_config_path: Path) -> tuple[list[str], str]:
    robot_cfg = _load_yaml(robot_config_path)
    kin = robot_cfg["robot_cfg"]["kinematics"]
    base_link = kin["base_link"]
    collision_links = list(kin.get("collision_link_names", []))
    return [base_link] + collision_links, kin["urdf_path"]


def _validate_and_group(
    items: list[dict[str, Any]],
    expected_links: list[str],
) -> dict[str, list[dict[str, Any]]]:
    expected = set(expected_links)
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)

    for index, item in enumerate(items):
        if not isinstance(item, dict):
            raise ValueError(f"sphere item #{index} must be a mapping")
        link_name = item.get("link")
        position = item.get("position", item.get("center"))
        radius = item.get("radius")
        if link_name not in expected:
            raise ValueError(f"unknown link in Bubblify export: {link_name}")
        if not isinstance(position, list) or len(position) != 3:
            raise ValueError(f"link {link_name} sphere #{index} must have a 3-element position")
        if radius is None or float(radius) <= 0.0:
            raise ValueError(f"link {link_name} sphere #{index} must have radius > 0")

        grouped[link_name].append(
            {
                "center": [float(v) for v in position],
                "radius": float(radius),
            }
        )

    missing = [link for link in expected_links if not grouped.get(link)]
    if missing:
        raise ValueError(f"Bubblify export is missing spheres for required links: {missing}")

    return {link: grouped[link] for link in expected_links}


def _build_output(
    grouped: dict[str, list[dict[str, Any]]],
    *,
    input_yaml: Path,
    robot_config: Path,
    urdf_path: str,
) -> dict[str, Any]:
    total_spheres = sum(len(spheres) for spheres in grouped.values())
    return {
        "collision_spheres": grouped,
        "metadata": {
            "source": "bubblify",
            "input_yaml": str(input_yaml),
            "robot_config": str(robot_config),
            "urdf_path": urdf_path,
            "total_spheres": total_spheres,
            "links": list(grouped.keys()),
            "exported_at_utc": datetime.now(timezone.utc).isoformat(),
            "notes": [
                "Generated from a Bubblify export for the active workspace ROKAE URDF.",
                "tool0 is intentionally excluded; only the base and six collision links are required.",
            ],
        },
    }


def main() -> None:
    args = _parse_args()
    raw = _load_yaml(args.input_yaml)
    items = _normalize_bubblify_items(raw)
    expected_links, urdf_path = _load_expected_links(args.robot_config)
    grouped = _validate_and_group(items, expected_links)
    output = _build_output(
        grouped,
        input_yaml=args.input_yaml.resolve(),
        robot_config=args.robot_config.resolve(),
        urdf_path=urdf_path,
    )
    args.output_yaml.parent.mkdir(parents=True, exist_ok=True)
    args.output_yaml.write_text(yaml.safe_dump(output, sort_keys=False, allow_unicode=True))
    print(f"Wrote cuRobo collision_spheres YAML to {args.output_yaml}")
    print(f"Links: {len(grouped)}")
    print(f"Total spheres: {output['metadata']['total_spheres']}")


if __name__ == "__main__":
    main()
