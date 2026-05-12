#!/usr/bin/env python3
"""One-click official Franka planning + MuJoCo playback entrypoint."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import mujoco
import mujoco.viewer

from export_mujoco_playback_contract import export_contract
from replay_official_franka_mujoco import _load_contract, _resolve_qpos_addresses, generate_franka_stage1_mjcf, replay_contract

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]


def _realtime_replay(
    contract_path: Path,
    output_dir: Path,
    *,
    playback_speed: float,
    final_hold_s: float,
    shutdown_wait_s: float = 5.0,
) -> dict[str, Any]:
    contract = _load_contract(contract_path)
    joint_names = list(contract["joint_contract"]["mujoco_expected_joint_names"])
    waypoints = list(contract["trajectory_contract"]["waypoints"])
    dt = float(contract["timing_contract"]["sample_period_s"])

    model_xml_path = generate_franka_stage1_mjcf(output_dir / "franka_stage1_realtime.xml")
    model = mujoco.MjModel.from_xml_path(str(model_xml_path))
    data = mujoco.MjData(model)
    qpos_mapping = _resolve_qpos_addresses(model, joint_names)

    handle = mujoco.viewer.launch_passive(
        model,
        data,
        show_left_ui=False,
        show_right_ui=False,
    )
    sim_ref = handle._sim

    completed_all_waypoints = True
    start_time = time.perf_counter()
    rendered_waypoints = 0

    try:
        for waypoint in waypoints:
            if not handle.is_running():
                completed_all_waypoints = False
                break

            with handle.lock():
                for target, mapping in zip(waypoint, qpos_mapping):
                    data.qpos[mapping["qpos_adr"]] = target
                mujoco.mj_forward(model, data)

            handle.sync()
            rendered_waypoints += 1
            time.sleep(max(dt / playback_speed, 0.0))

        hold_until = time.perf_counter() + max(final_hold_s, 0.0)
        while handle.is_running() and time.perf_counter() < hold_until:
            handle.sync()
            time.sleep(min(dt / max(playback_speed, 1e-6), 0.05))
    finally:
        elapsed = time.perf_counter() - start_time
        handle.close()
        shutdown_deadline = time.perf_counter() + max(shutdown_wait_s, 0.0)
        while sim_ref() is not None and time.perf_counter() < shutdown_deadline:
            time.sleep(0.01)

    summary = {
        "success": completed_all_waypoints,
        "contract_path": str(contract_path),
        "model_xml_path": str(model_xml_path),
        "playback_speed": playback_speed,
        "requested_waypoints": len(waypoints),
        "rendered_waypoints": rendered_waypoints,
        "sample_period_s": dt,
        "final_hold_s": final_hold_s,
        "elapsed_wall_time_s": elapsed,
        "viewer_shutdown_wait_s": shutdown_wait_s,
        "viewer_shutdown_completed": sim_ref() is None,
        "stopped_early": not completed_all_waypoints,
    }
    (output_dir / "realtime_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description="One-click official Franka planning plus MuJoCo playback"
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help="Optional root directory for this run; defaults to evidence/one_click/<timestamp>",
    )
    parser.add_argument(
        "--render-every",
        type=int,
        default=1,
        help="Offscreen replay renders every Nth waypoint while always keeping the final frame",
    )
    parser.add_argument(
        "--playback-speed",
        type=float,
        default=1.0,
        help="Realtime viewer speed multiplier; 1.0 means one contract sample every contract dt",
    )
    parser.add_argument(
        "--final-hold-s",
        type=float,
        default=1.0,
        help="How long to keep the final realtime pose on screen before closing the viewer",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="Skip the realtime MuJoCo viewer and only generate offscreen evidence",
    )
    args = parser.parse_args()

    if args.render_every <= 0:
        raise SystemExit("--render-every must be positive")
    if args.playback_speed <= 0.0:
        raise SystemExit("--playback-speed must be positive")

    if args.output_root is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_root = WORKSPACE_ROOT / "evidence" / "one_click" / stamp
    else:
        output_root = args.output_root

    contract_dir = output_root / "contract"
    playback_dir = output_root / "playback"
    realtime_dir = output_root / "realtime"
    contract_dir.mkdir(parents=True, exist_ok=True)
    playback_dir.mkdir(parents=True, exist_ok=True)
    realtime_dir.mkdir(parents=True, exist_ok=True)

    contract = export_contract(contract_dir)
    contract_path = contract_dir / "playback_contract.json"
    playback_summary = replay_contract(contract_path, playback_dir, args.render_every)

    realtime_summary = None
    if not args.no_viewer:
        realtime_summary = _realtime_replay(
            contract_path,
            realtime_dir,
            playback_speed=args.playback_speed,
            final_hold_s=args.final_hold_s,
        )

    summary = {
        "success": True,
        "contract_path": str(contract_path),
        "offscreen_summary_path": str(playback_dir / "playback_summary.json"),
        "realtime_summary_path": str(realtime_dir / "realtime_summary.json") if realtime_summary else None,
        "contract_waypoint_count": contract["timing_contract"]["sample_count"],
        "contract_sample_period_s": contract["timing_contract"]["sample_period_s"],
        "offscreen_rendered_frames": playback_summary["render_summary"]["frame_count"],
        "viewer_enabled": not args.no_viewer,
        "playback_speed": args.playback_speed,
    }
    (output_root / "run_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )

    print("One-click Franka demo completed")
    print(f"Output root: {output_root}")
    print(f"Contract: {contract_path}")
    print(f"Offscreen GIF: {playback_dir / 'playback.gif'}")
    if realtime_summary is not None:
        print(f"Realtime summary: {realtime_dir / 'realtime_summary.json'}")


if __name__ == "__main__":
    main()
