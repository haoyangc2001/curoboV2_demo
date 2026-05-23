#!/usr/bin/env python3
"""离线规划输入配置加载与校验模块。

定义统一规划输入 schema，支持从 YAML 文件加载并校验。
配置格式详见 resource/config/examples/ 下的示例文件。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


VALID_MODES = frozenset({
    "point_to_point",
    "joint_target",
    "approach",
    "grasp",
    "level_carry",
})

_REQUIRED_BY_MODE: dict[str, list[str]] = {
    "point_to_point": ["start.joint_position", "goal.pose"],
    "joint_target": ["start.joint_position", "goal.joint_position"],
    "approach": ["start.joint_position", "goal.pose"],
    "grasp": ["start.joint_position", "goal.pose"],
    "level_carry": ["start.joint_position", "goal.pose"],
}


@dataclass
class StartConfig:
    joint_position: list[float] = field(default_factory=list)


@dataclass
class GoalConfig:
    pose: list[float] | None = None          # [x, y, z, qx, qy, qz, qw]
    joint_position: list[float] | None = None


@dataclass
class WorldConfig:
    obstacle_json: str | None = None         # 绝对障碍物 JSON 路径
    obstacle_rel_json: str | None = None     # 相对障碍物 JSON 路径


@dataclass
class PipelineConfig:
    run_plan: bool = True
    export_contract: bool = True
    replay_gif: bool = False
    realtime_viewer: bool = True
    render_every: int = 4
    playback_speed: float = 1.0
    final_hold_s: float = 1.0
    resume_from_plan_output_dir: str | None = None
    resume_from_contract_json: str | None = None


@dataclass
class PlanningConfig:
    """规划输入配置。"""
    mode: str = "point_to_point"
    robot_config: str | None = None          # 机器人 YAML 配置路径，None 则用默认
    start: StartConfig = field(default_factory=StartConfig)
    goal: GoalConfig = field(default_factory=GoalConfig)
    world: WorldConfig = field(default_factory=WorldConfig)
    pipeline: PipelineConfig = field(default_factory=PipelineConfig)
    output_dir: str | None = None

    # 可选规划参数
    speed_scale: float | None = None
    hold_vec_weight: list[float] | None = None
    approach_offset: float | None = None
    retract_offset: float | None = None
    linear_axis: int | None = None
    segment_length: float | None = None

    # 原始字典（供下游模块读取扩展字段）
    raw: dict[str, Any] = field(default_factory=dict, repr=False)


def load_config(config_path: Path) -> PlanningConfig:
    """从 YAML 文件加载并校验规划配置。

    Args:
        config_path: 配置文件路径。

    Returns:
        校验后的 PlanningConfig 实例。

    Raises:
        FileNotFoundError: 配置文件不存在。
        ValueError: 配置内容不合法。
    """
    config_path = Path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"配置文件不存在: {config_path}")

    raw: dict[str, Any] = yaml.safe_load(config_path.read_text())
    if not isinstance(raw, dict):
        raise ValueError(f"配置文件顶层必须是字典，实际为 {type(raw).__name__}")

    _validate(raw, config_path)
    return _build_config(raw, config_path)


def _validate(raw: dict[str, Any], config_path: Path) -> None:
    """校验配置字典。"""
    mode = raw.get("mode", "point_to_point")
    if mode not in VALID_MODES:
        raise ValueError(
            f"不支持的规划模式 '{mode}'，可选值: {sorted(VALID_MODES)}"
        )

    start = raw.get("start", {})
    if not start.get("joint_position"):
        raise ValueError("必须指定 start.joint_position")

    goal = raw.get("goal", {})
    required_fields = _REQUIRED_BY_MODE.get(mode, [])
    for field_path in required_fields:
        parts = field_path.split(".")
        obj = raw
        for part in parts:
            if not isinstance(obj, dict) or part not in obj:
                raise ValueError(
                    f"模式 '{mode}' 要求指定 {field_path}"
                )
            obj = obj[part]
        if obj is None:
            raise ValueError(
                f"模式 '{mode}' 要求 {field_path} 不能为空"
            )

    # 校验 joint_position 长度一致性
    start_jp = start.get("joint_position", [])
    goal_jp = goal.get("joint_position")
    if goal_jp is not None and len(start_jp) != len(goal_jp):
        raise ValueError(
            f"start.joint_position ({len(start_jp)}) 与 "
            f"goal.joint_position ({len(goal_jp)}) 长度不一致"
        )

    # 校验 goal.pose 长度
    goal_pose = goal.get("pose")
    if goal_pose is not None and len(goal_pose) != 7:
        raise ValueError(
            f"goal.pose 必须包含 7 个元素 [x,y,z,qx,qy,qz,qw]，实际 {len(goal_pose)}"
        )

    speed_scale = raw.get("speed_scale")
    if speed_scale is not None and not (0.0 < speed_scale <= 2.0):
        raise ValueError(f"speed_scale 必须在 (0, 2.0] 范围内，实际 {speed_scale}")

    hold_vec_weight = raw.get("hold_vec_weight")
    if hold_vec_weight is not None:
        if not isinstance(hold_vec_weight, list) or len(hold_vec_weight) != 3:
            raise ValueError("hold_vec_weight 必须是包含 3 个元素的列表 [x, y, z]")
        for w in hold_vec_weight:
            if not (0.0 <= w <= 1.0):
                raise ValueError(f"hold_vec_weight 每个元素必须在 [0, 1] 范围内，实际 {w}")

    pipeline = raw.get("pipeline", {})
    if pipeline is not None and not isinstance(pipeline, dict):
        raise ValueError(f"pipeline 必须是字典，实际为 {type(pipeline).__name__}")
    if isinstance(pipeline, dict):
        render_every = pipeline.get("render_every")
        if render_every is not None and int(render_every) <= 0:
            raise ValueError(f"pipeline.render_every 必须大于 0，实际 {render_every}")
        playback_speed = pipeline.get("playback_speed")
        if playback_speed is not None and float(playback_speed) <= 0.0:
            raise ValueError(f"pipeline.playback_speed 必须大于 0，实际 {playback_speed}")
        final_hold_s = pipeline.get("final_hold_s")
        if final_hold_s is not None and float(final_hold_s) < 0.0:
            raise ValueError(f"pipeline.final_hold_s 不能小于 0，实际 {final_hold_s}")


def _build_config(raw: dict[str, Any], config_path: Path) -> PlanningConfig:
    """从校验后的字典构建 PlanningConfig，解析相对路径为绝对路径。"""
    config_dir = config_path.parent

    start_raw = raw.get("start", {})
    goal_raw = raw.get("goal", {})
    world_raw = raw.get("world", {})
    pipeline_raw = raw.get("pipeline", {})

    robot_config = raw.get("robot_config")
    if robot_config is not None:
        robot_config_path = Path(robot_config)
        if not robot_config_path.is_absolute():
            robot_config_path = (config_dir / robot_config_path).resolve()
        robot_config = str(robot_config_path)

    obstacle_json = world_raw.get("obstacle_json")
    if obstacle_json is not None:
        p = Path(obstacle_json)
        if not p.is_absolute():
            p = (config_dir / p).resolve()
        obstacle_json = str(p)

    obstacle_rel_json = world_raw.get("obstacle_rel_json")
    if obstacle_rel_json is not None:
        p = Path(obstacle_rel_json)
        if not p.is_absolute():
            p = (config_dir / p).resolve()
        obstacle_rel_json = str(p)

    output_dir = raw.get("output_dir")
    if output_dir is not None:
        p = Path(output_dir)
        if not p.is_absolute():
            p = (config_dir / p).resolve()
        output_dir = str(p)

    resume_from_plan_output_dir = pipeline_raw.get("resume_from_plan_output_dir")
    if resume_from_plan_output_dir is not None:
        p = Path(resume_from_plan_output_dir)
        if not p.is_absolute():
            p = (config_dir / p).resolve()
        resume_from_plan_output_dir = str(p)

    resume_from_contract_json = pipeline_raw.get("resume_from_contract_json")
    if resume_from_contract_json is not None:
        p = Path(resume_from_contract_json)
        if not p.is_absolute():
            p = (config_dir / p).resolve()
        resume_from_contract_json = str(p)

    return PlanningConfig(
        mode=raw.get("mode", "point_to_point"),
        robot_config=robot_config,
        start=StartConfig(
            joint_position=start_raw.get("joint_position", []),
        ),
        goal=GoalConfig(
            pose=goal_raw.get("pose"),
            joint_position=goal_raw.get("joint_position"),
        ),
        world=WorldConfig(
            obstacle_json=obstacle_json,
            obstacle_rel_json=obstacle_rel_json,
        ),
        pipeline=PipelineConfig(
            run_plan=bool(pipeline_raw.get("run_plan", True)),
            export_contract=bool(pipeline_raw.get("export_contract", True)),
            replay_gif=bool(pipeline_raw.get("replay_gif", False)),
            realtime_viewer=bool(pipeline_raw.get("realtime_viewer", True)),
            render_every=int(pipeline_raw.get("render_every", 4)),
            playback_speed=float(pipeline_raw.get("playback_speed", 1.0)),
            final_hold_s=float(pipeline_raw.get("final_hold_s", 1.0)),
            resume_from_plan_output_dir=resume_from_plan_output_dir,
            resume_from_contract_json=resume_from_contract_json,
        ),
        output_dir=output_dir,
        speed_scale=raw.get("speed_scale"),
        hold_vec_weight=raw.get("hold_vec_weight"),
        approach_offset=raw.get("approach_offset"),
        retract_offset=raw.get("retract_offset"),
        linear_axis=raw.get("linear_axis"),
        segment_length=raw.get("segment_length"),
        raw=raw,
    )
