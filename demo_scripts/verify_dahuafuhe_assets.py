#!/usr/bin/env python3
"""Repeatedly verify the self-contained dahuafuhe asset bundle for stage 1."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from xml.etree import ElementTree as ET

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
LOCAL_CUROBO_ROOT = WORKSPACE_ROOT / "third_party" / "curobo"
if str(LOCAL_CUROBO_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_CUROBO_ROOT))

from curobo._src.types.robot import RobotCfg

from dahuafuhe_asset_utils import (
    WORKSPACE_ROOT as DEMO_WORKSPACE_ROOT,
    is_within_workspace,
    load_yaml,
    resolve_robot_config_for_workspace,
    workspace_manifest_path,
    workspace_robot_config_path,
    workspace_spheres_path,
    workspace_start_launch_path,
    workspace_urdf_path,
)


def _parse_urdf(urdf_path: Path) -> dict:
    root = ET.fromstring(urdf_path.read_text())
    link_names = [link.attrib["name"] for link in root.findall("link")]
    movable_joint_names = []
    all_joint_names = []
    mesh_paths = []

    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        all_joint_names.append(name)
        if joint.attrib.get("type") != "fixed":
            movable_joint_names.append(name)

    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if filename is None:
            continue
        mesh_abs_path = (urdf_path.parent / filename).resolve()
        mesh_paths.append(
            {
                "filename": filename,
                "absolute_path": str(mesh_abs_path),
                "exists": mesh_abs_path.exists(),
                "within_workspace": is_within_workspace(mesh_abs_path),
            }
        )

    return {
        "link_names": link_names,
        "all_joint_names": all_joint_names,
        "movable_joint_names": movable_joint_names,
        "mesh_paths": mesh_paths,
    }


def _collision_sphere_counts(spheres_payload: dict) -> dict[str, int]:
    collision_spheres = spheres_payload.get("collision_spheres", {})
    return {str(link_name): len(link_spheres) for link_name, link_spheres in collision_spheres.items()}


def verify_once() -> dict:
    manifest_path = workspace_manifest_path()
    if not manifest_path.exists():
        raise FileNotFoundError(
            "dahuafuhe asset bundle is missing. Run materialize_dahuafuhe_assets.py first."
        )

    start_launch = load_yaml(workspace_start_launch_path())
    robot_cfg = load_yaml(workspace_robot_config_path())
    resolved_robot_cfg = resolve_robot_config_for_workspace()
    kinematics_cfg = resolved_robot_cfg["robot_cfg"]["kinematics"]
    urdf_path = Path(kinematics_cfg["urdf_path"])
    asset_root_path = Path(kinematics_cfg["asset_root_path"])
    spheres_path = workspace_spheres_path()

    start_launch_robot_rel = Path(start_launch["launch"]["robot"]["urdf"])
    start_launch_robot_abs = (workspace_start_launch_path().parent / start_launch_robot_rel).resolve()

    urdf_summary = _parse_urdf(urdf_path)
    spheres_payload = load_yaml(spheres_path)
    sphere_counts = _collision_sphere_counts(spheres_payload)
    config_joint_names = list(robot_cfg["robot_cfg"]["kinematics"]["cspace"]["joint_names"])
    collision_link_names = list(robot_cfg["robot_cfg"]["kinematics"]["collision_link_names"])
    mesh_link_names = list(robot_cfg["robot_cfg"]["kinematics"]["mesh_link_names"])

    curobo_robot_cfg = RobotCfg.create(resolved_robot_cfg)
    curobo_joint_names = list(curobo_robot_cfg.kinematics.cspace.joint_names)
    curobo_link_name_count = int(len(curobo_robot_cfg.kinematics.kinematics_config.link_name_to_idx_map))

    required_paths = {
        "start_launch_robot_abs": start_launch_robot_abs,
        "resolved_urdf_path": urdf_path,
        "resolved_asset_root_path": asset_root_path,
        "copied_collision_spheres_path": spheres_path,
    }
    required_paths_within_workspace = {
        key: is_within_workspace(path)
        for key, path in required_paths.items()
    }

    mesh_checks = urdf_summary["mesh_paths"]
    missing_meshes = [row for row in mesh_checks if not row["exists"]]
    external_meshes = [row for row in mesh_checks if not row["within_workspace"]]
    missing_collision_links = [name for name in collision_link_names if name not in sphere_counts]

    passed = (
        all(required_paths_within_workspace.values())
        and start_launch_robot_abs.exists()
        and urdf_path.exists()
        and asset_root_path.exists()
        and spheres_path.exists()
        and not missing_meshes
        and not external_meshes
        and not missing_collision_links
        and config_joint_names == urdf_summary["movable_joint_names"]
        and config_joint_names == curobo_joint_names
    )

    return {
        "passed": passed,
        "start_launch_robot_path": str(start_launch_robot_abs),
        "required_paths_within_workspace": required_paths_within_workspace,
        "urdf": {
            "path": str(urdf_path),
            "link_count": len(urdf_summary["link_names"]),
            "joint_count": len(urdf_summary["all_joint_names"]),
            "movable_joint_names": urdf_summary["movable_joint_names"],
            "mesh_count": len(mesh_checks),
            "mesh_checks": mesh_checks,
        },
        "robot_config": {
            "path": str(workspace_robot_config_path()),
            "base_link": robot_cfg["robot_cfg"]["kinematics"]["base_link"],
            "tool_frames": list(robot_cfg["robot_cfg"]["kinematics"]["tool_frames"]),
            "joint_names": config_joint_names,
            "collision_link_names": collision_link_names,
            "mesh_link_names": mesh_link_names,
        },
        "collision_spheres": {
            "path": str(spheres_path),
            "link_count": len(sphere_counts),
            "sphere_counts_by_link": sphere_counts,
            "missing_collision_links": missing_collision_links,
        },
        "curobo_load": {
            "joint_names": curobo_joint_names,
            "link_name_count": curobo_link_name_count,
            "total_spheres": int(curobo_robot_cfg.kinematics.kinematics_config.total_spheres),
        },
        "workspace_independence": {
            "workspace_root": str(DEMO_WORKSPACE_ROOT),
            "disallowed_external_prefix": str(DEMO_WORKSPACE_ROOT.parent / "tashan_robot"),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Verify the stage-1 dahuafuhe asset bundle")
    parser.add_argument(
        "--repeat-count",
        type=int,
        default=5,
        help="Number of repeated load checks to run",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Directory that will receive per-run summaries and a review summary",
    )
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    runs = []

    for run_idx in range(1, args.repeat_count + 1):
        run_dir = args.output_dir / f"run_{run_idx:02d}"
        run_dir.mkdir(parents=True, exist_ok=True)
        summary = verify_once()
        summary["run_index"] = run_idx
        (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
        runs.append(summary)
        print(f"run_{run_idx:02d}: {'pass' if summary['passed'] else 'fail'}")

    review_summary = {
        "stage": "S1-007",
        "generated_at": datetime.now().astimezone().isoformat(),
        "repeat_count": args.repeat_count,
        "passed": all(run["passed"] for run in runs),
        "bundle_manifest_path": str(workspace_manifest_path()),
        "robot_config_path": str(workspace_robot_config_path()),
        "urdf_path": str(workspace_urdf_path()),
        "collision_spheres_path": str(workspace_spheres_path()),
        "runs": [
            {
                "run_index": run["run_index"],
                "passed": run["passed"],
                "joint_names": run["robot_config"]["joint_names"],
                "movable_joint_names": run["urdf"]["movable_joint_names"],
                "mesh_count": run["urdf"]["mesh_count"],
                "sphere_link_count": run["collision_spheres"]["link_count"],
            }
            for run in runs
        ],
    }
    (args.output_dir / "review_summary.json").write_text(
        json.dumps(review_summary, indent=2, ensure_ascii=False) + "\n"
    )

    raise SystemExit(0 if review_summary["passed"] else 1)


if __name__ == "__main__":
    main()
