#!/usr/bin/env python3
"""Unified planning -> contract -> playback pipeline entrypoint for ROKAE."""

from __future__ import annotations

import argparse
import json
import sys
import time
from copy import deepcopy
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Any

_SCRIPTS_DIR = Path(__file__).resolve().parent
_WORKSPACE_ROOT = _SCRIPTS_DIR.parent
_PLAYBACK_DIR = _WORKSPACE_ROOT / "playback"
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))
if str(_PLAYBACK_DIR) not in sys.path:
    sys.path.insert(0, str(_PLAYBACK_DIR))

from config_utils import PlanningConfig, load_config
from plan_rokae_motion import apply_overrides, run_to_output_dir
from export_rokae_playback_contract import build_contract_from_plan_output
from replay_rokae_mujoco import replay_contract
from run_rokae_demo import _realtime_replay


def _parse_args() -> argparse.Namespace:
    default_config = str(_WORKSPACE_ROOT / "resource" / "config" / "examples" / "pose_plan_example.yaml")
    parser = argparse.ArgumentParser(description="Unified ROKAE planning, contract export, and playback pipeline")
    parser.add_argument("--config", default=default_config, help="Pipeline YAML config path")
    parser.add_argument("--mode", help="覆盖规划模式")
    parser.add_argument("--start-jp", help="覆盖起始关节角（逗号分隔）")
    parser.add_argument("--goal-pose", help="覆盖目标位姿（逗号分隔，x,y,z,qx,qy,qz,qw）")
    parser.add_argument("--goal-jp", help="覆盖目标关节角（逗号分隔）")
    parser.add_argument("--output-root", help="覆盖统一输出根目录")
    parser.add_argument("--speed-scale", type=float, help="速度缩放 (0, 2.0]")
    parser.add_argument("--hold-vec-weight", help="方向保持权重（逗号分隔，x,y,z）")
    parser.add_argument("--approach-offset", type=float, help="接近偏移量（米）")
    parser.add_argument("--approach-axis", default="z", help="接近轴（x/y/z，默认 z）")
    parser.add_argument("--plan-output-dir", help="从已有规划输出继续后续阶段")
    parser.add_argument("--contract-json", help="从已有回放合同继续回放/viewer")
    parser.add_argument("--render-every", type=int, help="离屏回放每隔多少个 waypoint 渲染一帧")
    parser.add_argument("--playback-speed", type=float, help="实时 viewer 播放速度倍率")
    parser.add_argument("--final-hold-s", type=float, help="实时 viewer 最后一帧停留秒数")
    bool_action = argparse.BooleanOptionalAction
    parser.add_argument("--plan", action=bool_action, default=None, help="是否执行规划阶段")
    parser.add_argument("--export-contract", action=bool_action, default=None, help="是否导出回放合同")
    parser.add_argument("--replay-gif", action=bool_action, default=None, help="是否执行离屏回放并导出 GIF")
    parser.add_argument("--viewer", action=bool_action, default=None, help="是否尝试启动 realtime viewer")
    return parser.parse_args()


def _default_output_root() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return _WORKSPACE_ROOT / "evidence" / "rokae_pipeline" / stamp


def _apply_pipeline_overrides(cfg: PlanningConfig, args: argparse.Namespace) -> PlanningConfig:
    if args.output_root:
        cfg.output_dir = str(Path(args.output_root).resolve())
    if args.plan is not None:
        cfg.pipeline.run_plan = args.plan
    if args.export_contract is not None:
        cfg.pipeline.export_contract = args.export_contract
    if args.replay_gif is not None:
        cfg.pipeline.replay_gif = args.replay_gif
    if args.viewer is not None:
        cfg.pipeline.realtime_viewer = args.viewer
    if args.render_every is not None:
        cfg.pipeline.render_every = args.render_every
    if args.playback_speed is not None:
        cfg.pipeline.playback_speed = args.playback_speed
    if args.final_hold_s is not None:
        cfg.pipeline.final_hold_s = args.final_hold_s
    if args.plan_output_dir:
        cfg.pipeline.resume_from_plan_output_dir = str(Path(args.plan_output_dir).resolve())
    if args.contract_json:
        cfg.pipeline.resume_from_contract_json = str(Path(args.contract_json).resolve())
    return cfg


def _resolve_pipeline(cfg: PlanningConfig) -> dict[str, Any]:
    pipeline = deepcopy(cfg.pipeline)
    output_root = Path(cfg.output_dir).resolve() if cfg.output_dir else _default_output_root()
    plan_dir = output_root / "plan"
    contract_dir = output_root / "contract"
    playback_dir = output_root / "playback"
    realtime_dir = output_root / "realtime"

    resume_plan = Path(pipeline.resume_from_plan_output_dir).resolve() if pipeline.resume_from_plan_output_dir else None
    resume_contract = Path(pipeline.resume_from_contract_json).resolve() if pipeline.resume_from_contract_json else None
    if resume_plan is not None and resume_contract is not None:
        raise ValueError("pipeline.resume_from_plan_output_dir 与 pipeline.resume_from_contract_json 不能同时指定")

    stages = {
        "plan": {"enabled": pipeline.run_plan, "status": "pending", "reason": None},
        "contract": {"enabled": pipeline.export_contract, "status": "pending", "reason": None},
        "playback": {"enabled": pipeline.replay_gif, "status": "pending", "reason": None},
        "realtime": {"enabled": pipeline.realtime_viewer, "status": "pending", "reason": None},
    }

    plan_source_dir = plan_dir
    contract_source_json = contract_dir / "playback_contract.json"

    if resume_contract is not None:
        stages["plan"]["enabled"] = False
        stages["plan"]["status"] = "skipped"
        stages["plan"]["reason"] = "resume_from_contract_json supplied"
        stages["contract"]["enabled"] = False
        stages["contract"]["status"] = "skipped"
        stages["contract"]["reason"] = "resume_from_contract_json supplied"
        contract_source_json = resume_contract
    elif resume_plan is not None:
        stages["plan"]["enabled"] = False
        stages["plan"]["status"] = "skipped"
        stages["plan"]["reason"] = "resume_from_plan_output_dir supplied"
        plan_source_dir = resume_plan

    if stages["realtime"]["enabled"] and not (stages["plan"]["enabled"] or stages["contract"]["enabled"] or stages["playback"]["enabled"] or resume_contract is not None or resume_plan is not None):
        raise ValueError("realtime viewer 需要 contract 来源，但当前既未执行前置阶段也未提供 resume 输入")
    if stages["playback"]["enabled"] and not (stages["contract"]["enabled"] or stages["plan"]["enabled"] or resume_contract is not None or resume_plan is not None):
        raise ValueError("replay_gif 需要 contract 来源，但当前既未执行前置阶段也未提供 resume 输入")
    if stages["contract"]["enabled"] and not (stages["plan"]["enabled"] or resume_plan is not None):
        raise ValueError("export_contract 需要 plan 输出，但当前既未执行规划也未提供 resume_from_plan_output_dir")

    return {
        "pipeline": pipeline,
        "output_root": output_root,
        "plan_dir": plan_dir,
        "contract_dir": contract_dir,
        "playback_dir": playback_dir,
        "realtime_dir": realtime_dir,
        "plan_source_dir": plan_source_dir,
        "contract_source_json": contract_source_json,
        "stages": stages,
    }


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")


def _stage_record(enabled: bool, status: str, *, reason: str | None = None, artifacts: dict[str, Any] | None = None, elapsed_s: float | None = None) -> dict[str, Any]:
    return {
        "enabled": enabled,
        "status": status,
        "reason": reason,
        "elapsed_s": elapsed_s,
        "artifacts": artifacts or {},
    }


def main() -> None:
    args = _parse_args()
    config_path = Path(args.config).resolve()
    cfg = load_config(config_path)
    cfg = apply_overrides(cfg, args)
    cfg = _apply_pipeline_overrides(cfg, args)
    resolved = _resolve_pipeline(cfg)

    output_root: Path = resolved["output_root"]
    output_root.mkdir(parents=True, exist_ok=True)
    summary: dict[str, Any] = {
        "success": False,
        "input_config": str(config_path),
        "output_root": str(output_root),
        "resolved_pipeline": asdict(resolved["pipeline"]),
        "stages": {},
        "sources": {
            "resume_from_plan_output_dir": str(resolved["plan_source_dir"]) if resolved["plan_source_dir"] != resolved["plan_dir"] else None,
            "resume_from_contract_json": str(resolved["contract_source_json"]) if resolved["contract_source_json"] != (resolved["contract_dir"] / "playback_contract.json") else None,
        },
    }

    total_t0 = time.monotonic()
    contract_json_path = resolved["contract_source_json"]
    plan_source_dir = resolved["plan_source_dir"]

    try:
        if resolved["stages"]["plan"]["enabled"]:
            plan_cfg = deepcopy(cfg)
            plan_cfg.output_dir = str(resolved["plan_dir"])
            t0 = time.monotonic()
            plan_result = run_to_output_dir(plan_cfg, config_path=config_path)
            elapsed = time.monotonic() - t0
            plan_source_dir = Path(plan_result["plan_output_dir"])
            summary["stages"]["plan"] = _stage_record(
                True,
                "success" if plan_result["result"]["success"] else "failed",
                artifacts=plan_result["artifacts"],
                elapsed_s=elapsed,
            )
            if not plan_result["result"]["success"]:
                raise RuntimeError(f"planning failed (status={plan_result['result'].get('status')})")
        else:
            stage = resolved["stages"]["plan"]
            summary["stages"]["plan"] = _stage_record(False, stage["status"], reason=stage["reason"])

        if resolved["stages"]["contract"]["enabled"]:
            t0 = time.monotonic()
            resolved["contract_dir"].mkdir(parents=True, exist_ok=True)
            contract = build_contract_from_plan_output(plan_source_dir, resolved["contract_dir"])
            elapsed = time.monotonic() - t0
            contract_json_path = resolved["contract_dir"] / "playback_contract.json"
            summary["stages"]["contract"] = _stage_record(
                True,
                "success",
                artifacts={
                    "plan_output_dir": str(plan_source_dir),
                    "contract_json": str(contract_json_path),
                    "review_summary_json": str(resolved["contract_dir"] / "review_summary.json"),
                    "sample_count": contract["timing_contract"]["sample_count"],
                },
                elapsed_s=elapsed,
            )
        else:
            stage = resolved["stages"]["contract"]
            summary["stages"]["contract"] = _stage_record(
                False,
                stage["status"],
                reason=stage["reason"],
                artifacts={"contract_json": str(contract_json_path)} if contract_json_path.exists() else {},
            )

        if resolved["stages"]["playback"]["enabled"]:
            t0 = time.monotonic()
            resolved["playback_dir"].mkdir(parents=True, exist_ok=True)
            playback_summary = replay_contract(contract_json_path, resolved["playback_dir"], resolved["pipeline"].render_every)
            elapsed = time.monotonic() - t0
            summary["stages"]["playback"] = _stage_record(
                True,
                "success" if playback_summary["success"] else "failed",
                artifacts={
                    "contract_json": str(contract_json_path),
                    "playback_summary_json": str(resolved["playback_dir"] / "playback_summary.json"),
                    "playback_gif": str(resolved["playback_dir"] / "playback.gif"),
                    "model_xml": str(resolved["playback_dir"] / "rokae_stage1_playback.xml"),
                },
                elapsed_s=elapsed,
            )
            if not playback_summary["success"]:
                raise RuntimeError("offscreen playback failed")
        else:
            stage = resolved["stages"]["playback"]
            status = "skipped" if stage["status"] == "pending" else stage["status"]
            summary["stages"]["playback"] = _stage_record(False, status, reason=stage["reason"] or "disabled by pipeline configuration")

        if resolved["stages"]["realtime"]["enabled"]:
            t0 = time.monotonic()
            resolved["realtime_dir"].mkdir(parents=True, exist_ok=True)
            try:
                realtime_summary = _realtime_replay(
                    contract_json_path,
                    resolved["realtime_dir"],
                    playback_speed=resolved["pipeline"].playback_speed,
                    final_hold_s=resolved["pipeline"].final_hold_s,
                )
                elapsed = time.monotonic() - t0
                summary["stages"]["realtime"] = _stage_record(
                    True,
                    "success" if realtime_summary["success"] else "failed",
                    artifacts={
                        "realtime_summary_json": str(resolved["realtime_dir"] / "realtime_summary.json"),
                        "contract_json": str(contract_json_path),
                    },
                    elapsed_s=elapsed,
                )
                if not realtime_summary["success"]:
                    raise RuntimeError("realtime viewer stopped early")
            except Exception as exc:  # noqa: BLE001
                elapsed = time.monotonic() - t0
                fallback_summary = {
                    "success": False,
                    "status": "skipped",
                    "reason": f"viewer unavailable: {exc}",
                    "contract_path": str(contract_json_path),
                    "playback_speed": resolved["pipeline"].playback_speed,
                    "final_hold_s": resolved["pipeline"].final_hold_s,
                }
                _write_json(resolved["realtime_dir"] / "realtime_summary.json", fallback_summary)
                summary["stages"]["realtime"] = _stage_record(
                    True,
                    "skipped",
                    reason=str(exc),
                    artifacts={"realtime_summary_json": str(resolved["realtime_dir"] / "realtime_summary.json")},
                    elapsed_s=elapsed,
                )
        else:
            stage = resolved["stages"]["realtime"]
            status = "skipped" if stage["status"] == "pending" else stage["status"]
            summary["stages"]["realtime"] = _stage_record(False, status, reason=stage["reason"] or "disabled by pipeline configuration")

        summary["success"] = all(
            stage["status"] in {"success", "skipped"}
            for stage in summary["stages"].values()
        )
    except Exception as exc:  # noqa: BLE001
        summary["success"] = False
        summary["error"] = str(exc)
        raise
    finally:
        summary["total_elapsed_s"] = time.monotonic() - total_t0
        _write_json(output_root / "run_summary.json", summary)

    print("ROKAE unified pipeline completed")
    print(f"Output root: {output_root}")
    print(f"Run summary: {output_root / 'run_summary.json'}")
    if summary["stages"]["playback"]["status"] == "success":
        print(f"Playback GIF: {resolved['playback_dir'] / 'playback.gif'}")
    if summary["stages"]["realtime"]["status"] == "skipped":
        print(f"Realtime viewer skipped: {summary['stages']['realtime']['reason']}")


if __name__ == "__main__":
    main()
