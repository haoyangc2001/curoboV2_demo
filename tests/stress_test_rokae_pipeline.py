#!/usr/bin/env python3
"""Run a repeatable planning -> contract -> playback stress suite for ROKAE."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = WORKSPACE_ROOT / "scripts"
PLAYBACK_DIR = WORKSPACE_ROOT / "playback"
ROBOT_CONFIG_PATH = WORKSPACE_ROOT / "robot_assets" / "ROKAE" / "robot" / "xms5_r800_w4g3b4c_robot.yml"
ACTIVE_SPHERES_PATH = WORKSPACE_ROOT / "robot_assets" / "ROKAE" / "robot" / "spheres" / "ROKAE_SR5_0.9C_spherized.yml"
DEFAULT_OUTPUT_ROOT = WORKSPACE_ROOT / "evidence" / "rokae_bubblify_stress"
SIMPLE_WORLD_PATH = WORKSPACE_ROOT / "resource" / "config" / "examples" / "obstacles" / "simple_test.json"
MUJOCO_ENV = {"MUJOCO_GL": "egl"}
DEFAULT_START_JOINT = [-1.571, 1.571, 0.0, 1.571, 1.571, 0.0]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Stress-test the ROKAE planning and playback pipeline")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help="Root directory for stress evidence",
    )
    parser.add_argument(
        "--candidate-spheres",
        type=Path,
        default=ACTIVE_SPHERES_PATH,
        help="Candidate spheres file used for the candidate round",
    )
    parser.add_argument(
        "--baseline-spheres",
        type=Path,
        default=ACTIVE_SPHERES_PATH,
        help="Baseline spheres file used for the baseline round",
    )
    parser.add_argument(
        "--skip-candidate",
        action="store_true",
        help="Only run the baseline round",
    )
    return parser.parse_args()


def _run_command(cmd: list[str], *, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    merged_env = os.environ.copy()
    if env:
        merged_env.update(env)
    return subprocess.run(
        cmd,
        cwd=str(WORKSPACE_ROOT),
        text=True,
        capture_output=True,
        env=merged_env,
        check=False,
    )


def _case_name(spec: dict[str, Any]) -> str:
    parts = [spec["mode"], spec["label"]]
    if spec.get("world_name"):
        parts.append(spec["world_name"])
    if spec.get("speed_scale") is not None:
        parts.append(f"spd{spec['speed_scale']}")
    if spec.get("hold_vec_weight") is not None:
        parts.append("hold" + "_".join(str(v) for v in spec["hold_vec_weight"]))
    if spec.get("approach_offset") is not None:
        parts.append(f"app{spec['approach_offset']}")
    if spec.get("retract_offset") is not None:
        parts.append(f"ret{spec['retract_offset']}")
    return "__".join(parts).replace("/", "_")


def _build_matrix() -> list[dict[str, Any]]:
    worlds = [
        {"name": "no_world", "path": None},
        {"name": "simple_world", "path": SIMPLE_WORLD_PATH},
    ]
    speed_scales = [0.5, 1.0, 1.5]
    hold_weights = [None, [0.0, 0.0, 1.0]]
    pose_targets = [
        [0.45, 0.0, 0.85, 0.0, 0.707, 0.0, 0.707],
        [0.38, 0.20, 0.72, 0.0, 0.707, 0.0, 0.707],
        [0.50, -0.15, 0.68, 0.5, 0.5, 0.5, 0.5],
    ]
    joint_targets = [
        [-1.0, 1.2, 0.3, 1.0, 1.5, 0.5],
        [-1.3, 1.0, -0.1, 1.2, 1.2, 0.2],
        [-0.8, 1.4, 0.5, 0.8, 1.4, -0.2],
    ]
    approach_targets = [
        [0.45, 0.0, 0.55, 0.0, 0.707, 0.0, 0.707],
        [0.40, -0.1, 0.60, 0.0, 0.707, 0.0, 0.707],
    ]
    grasp_targets = [
        [0.45, 0.0, 0.55, 0.0, 0.707, 0.0, 0.707],
        [0.42, -0.08, 0.58, 0.0, 0.707, 0.0, 0.707],
    ]

    specs: list[dict[str, Any]] = []
    for pose_index, pose in enumerate(pose_targets, start=1):
        for speed in speed_scales:
            for world in worlds:
                for hold in hold_weights:
                    specs.append(
                        {
                            "mode": "point_to_point",
                            "label": f"pose{pose_index}",
                            "goal_pose": pose,
                            "speed_scale": speed,
                            "world_name": world["name"],
                            "world_path": world["path"],
                            "hold_vec_weight": hold,
                        }
                    )
    for joint_index, joint in enumerate(joint_targets, start=1):
        for speed in speed_scales:
            for world in worlds:
                specs.append(
                    {
                        "mode": "joint_target",
                        "label": f"joint{joint_index}",
                        "goal_joint": joint,
                        "speed_scale": speed,
                        "world_name": world["name"],
                        "world_path": world["path"],
                    }
                )
    for pose_index, pose in enumerate(approach_targets, start=1):
        for approach_offset in (-0.10, -0.15):
            for world in worlds:
                specs.append(
                    {
                        "mode": "approach",
                        "label": f"approach{pose_index}",
                        "goal_pose": pose,
                        "world_name": world["name"],
                        "world_path": world["path"],
                        "approach_offset": approach_offset,
                    }
                )
    for pose_index, pose in enumerate(grasp_targets, start=1):
        for approach_offset in (-0.10, -0.15):
            for retract_offset in (-0.10, -0.15):
                for world in worlds:
                    specs.append(
                        {
                            "mode": "grasp",
                            "label": f"grasp{pose_index}",
                            "goal_pose": pose,
                            "world_name": world["name"],
                            "world_path": world["path"],
                            "approach_offset": approach_offset,
                            "retract_offset": retract_offset,
                        }
                    )
    return specs


def _write_case_config(spec: dict[str, Any], case_dir: Path, plan_dir: Path) -> Path:
    payload: dict[str, Any] = {
        "mode": spec["mode"],
        "robot_config": str(ROBOT_CONFIG_PATH),
        "start": {"joint_position": DEFAULT_START_JOINT},
        "output_dir": str(plan_dir),
        "world": {},
    }
    if spec["mode"] in {"point_to_point", "approach", "grasp"}:
        payload["goal"] = {"pose": spec["goal_pose"]}
    else:
        payload["goal"] = {"joint_position": spec["goal_joint"]}
    if spec.get("world_path") is not None:
        payload["world"]["obstacle_json"] = str(spec["world_path"])
    if spec.get("speed_scale") is not None:
        payload["speed_scale"] = spec["speed_scale"]
    if spec.get("hold_vec_weight") is not None:
        payload["hold_vec_weight"] = spec["hold_vec_weight"]
    if spec.get("approach_offset") is not None:
        payload["approach_offset"] = spec["approach_offset"]
        payload["approach_axis"] = "z"
    if spec.get("retract_offset") is not None:
        payload["retract_offset"] = spec["retract_offset"]
        payload["lift_axis"] = "z"

    config_path = case_dir / "case_config.yaml"
    config_path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True))
    return config_path


def _build_plan_command(config_path: Path, plan_dir: Path) -> list[str]:
    return [
        sys.executable,
        str(SCRIPTS_DIR / "plan_rokae_motion.py"),
        "--config",
        str(config_path),
        "--output-dir",
        str(plan_dir),
    ]


def _switch_round_spheres(round_name: str, source_path: Path) -> None:
    if not source_path.exists():
        raise FileNotFoundError(f"{round_name} spheres file not found: {source_path}")
    if source_path.resolve() == ACTIVE_SPHERES_PATH.resolve():
        return
    shutil.copyfile(source_path, ACTIVE_SPHERES_PATH)


def _restore_active_spheres(original_bytes: bytes) -> None:
    ACTIVE_SPHERES_PATH.write_bytes(original_bytes)


def _run_case(round_dir: Path, spec: dict[str, Any]) -> dict[str, Any]:
    case_dir = round_dir / _case_name(spec)
    plan_dir = case_dir / "plan"
    contract_dir = case_dir / "contract"
    playback_dir = case_dir / "playback"
    plan_dir.mkdir(parents=True, exist_ok=True)
    config_path = _write_case_config(spec, case_dir, plan_dir)

    plan_cmd = _build_plan_command(config_path, plan_dir)
    plan_result = _run_command(plan_cmd)
    record: dict[str, Any] = {
        "case": _case_name(spec),
        "mode": spec["mode"],
        "spec": spec,
        "config_path": str(config_path),
        "plan_command": plan_cmd,
        "plan_returncode": plan_result.returncode,
        "plan_stdout": plan_result.stdout,
        "plan_stderr": plan_result.stderr,
        "plan_success": False,
        "contract_success": False,
        "playback_success": False,
        "failed_stage": "plan",
    }

    summary_path = plan_dir / "summary.json"
    if not summary_path.exists():
        return record

    plan_summary = json.loads(summary_path.read_text())
    record["plan_summary"] = plan_summary
    record["plan_success"] = bool(plan_summary.get("success"))
    if not record["plan_success"]:
        return record

    contract_dir.mkdir(parents=True, exist_ok=True)
    contract_cmd = [
        sys.executable,
        str(PLAYBACK_DIR / "export_rokae_playback_contract.py"),
        "--plan-output-dir",
        str(plan_dir),
        "--output-dir",
        str(contract_dir),
    ]
    contract_result = _run_command(contract_cmd)
    record["contract_command"] = contract_cmd
    record["contract_returncode"] = contract_result.returncode
    record["contract_stdout"] = contract_result.stdout
    record["contract_stderr"] = contract_result.stderr
    contract_path = contract_dir / "playback_contract.json"
    if contract_result.returncode != 0 or not contract_path.exists():
        record["failed_stage"] = "contract"
        return record

    record["contract_success"] = True
    playback_dir.mkdir(parents=True, exist_ok=True)
    playback_cmd = [
        sys.executable,
        str(PLAYBACK_DIR / "replay_rokae_mujoco.py"),
        "--contract-json",
        str(contract_path),
        "--output-dir",
        str(playback_dir),
        "--render-every",
        "4",
    ]
    playback_result = _run_command(playback_cmd, env=MUJOCO_ENV)
    record["playback_command"] = playback_cmd
    record["playback_returncode"] = playback_result.returncode
    record["playback_stdout"] = playback_result.stdout
    record["playback_stderr"] = playback_result.stderr
    playback_summary_path = playback_dir / "playback_summary.json"
    if playback_result.returncode != 0 or not playback_summary_path.exists():
        record["failed_stage"] = "playback"
        return record

    record["playback_success"] = True
    record["failed_stage"] = None
    record["playback_summary"] = json.loads(playback_summary_path.read_text())
    return record


def _summarize_round(records: list[dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "case_count": len(records),
        "plan_success_count": sum(1 for r in records if r["plan_success"]),
        "plan_failure_count": sum(1 for r in records if not r["plan_success"]),
        "contract_success_count": sum(1 for r in records if r["contract_success"]),
        "playback_success_count": sum(1 for r in records if r["playback_success"]),
        "by_mode": {},
        "failures": [],
    }
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        grouped[record["mode"]].append(record)
        if record["failed_stage"] is not None:
            summary["failures"].append(
                {
                    "case": record["case"],
                    "mode": record["mode"],
                    "failed_stage": record["failed_stage"],
                }
            )
    for mode, mode_records in grouped.items():
        summary["by_mode"][mode] = {
            "total": len(mode_records),
            "plan_success_count": sum(1 for r in mode_records if r["plan_success"]),
            "contract_success_count": sum(1 for r in mode_records if r["contract_success"]),
            "playback_success_count": sum(1 for r in mode_records if r["playback_success"]),
        }
    return summary


def _diff_against_baseline(baseline: dict[str, Any], candidate: dict[str, Any]) -> dict[str, Any]:
    diff: dict[str, Any] = {
        "plan_success_delta": candidate["plan_success_count"] - baseline["plan_success_count"],
        "contract_success_delta": candidate["contract_success_count"] - baseline["contract_success_count"],
        "playback_success_delta": candidate["playback_success_count"] - baseline["playback_success_count"],
        "by_mode": {},
    }
    mode_names = sorted(set(baseline["by_mode"]) | set(candidate["by_mode"]))
    for mode in mode_names:
        base_stats = baseline["by_mode"].get(mode, {})
        cand_stats = candidate["by_mode"].get(mode, {})
        diff["by_mode"][mode] = {
            "plan_success_delta": cand_stats.get("plan_success_count", 0) - base_stats.get("plan_success_count", 0),
            "contract_success_delta": cand_stats.get("contract_success_count", 0) - base_stats.get("contract_success_count", 0),
            "playback_success_delta": cand_stats.get("playback_success_count", 0) - base_stats.get("playback_success_count", 0),
        }
    return diff


def main() -> None:
    args = _parse_args()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_root = args.output_root / timestamp
    run_root.mkdir(parents=True, exist_ok=True)

    matrix = _build_matrix()
    original_active_bytes = ACTIVE_SPHERES_PATH.read_bytes()
    summary: dict[str, Any] = {
        "generated_at": timestamp,
        "workspace_root": str(WORKSPACE_ROOT),
        "robot_config": str(ROBOT_CONFIG_PATH),
        "active_spheres_path": str(ACTIVE_SPHERES_PATH),
        "baseline_spheres": str(args.baseline_spheres),
        "candidate_spheres": str(args.candidate_spheres),
        "rounds": {},
    }

    try:
        rounds = [("baseline", args.baseline_spheres)]
        if not args.skip_candidate:
            rounds.append(("candidate", args.candidate_spheres))

        for round_name, spheres_path in rounds:
            _switch_round_spheres(round_name, spheres_path)
            round_dir = run_root / round_name
            round_dir.mkdir(parents=True, exist_ok=True)
            records = []
            for spec in matrix:
                records.append(_run_case(round_dir, spec))
            (round_dir / "round_records.json").write_text(json.dumps(records, indent=2, ensure_ascii=False) + "\n")
            summary["rounds"][round_name] = _summarize_round(records)

        if "baseline" in summary["rounds"] and "candidate" in summary["rounds"]:
            summary["difference_vs_baseline"] = _diff_against_baseline(
                summary["rounds"]["baseline"],
                summary["rounds"]["candidate"],
            )
    finally:
        _restore_active_spheres(original_active_bytes)

    (run_root / "stress_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    print(f"Stress run complete: {run_root}")
    print(f"Summary: {run_root / 'stress_summary.json'}")


if __name__ == "__main__":
    main()
