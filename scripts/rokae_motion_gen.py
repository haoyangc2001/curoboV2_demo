#!/usr/bin/env python3
"""离线 CuRobo 核心封装（基于 MotionPlanner）。

提供与 tashan_robot/dahuafuhe CuroboMotionGen 对应的离线规划接口，
底层使用 CuRobo V2 的 MotionPlanner（V2 中无 MotionGen 类）。

# adapted from tashan_robot/src/trajectory_planning/trajectory_planning/curobo_motion/curobo_motion_gen.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any

# 注入 vendored CuRobo 路径
_WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
_CUROBO_ROOT = _WORKSPACE_ROOT / "third_party" / "curobo"
if str(_CUROBO_ROOT) not in sys.path:
    sys.path.insert(0, str(_CUROBO_ROOT))

import torch
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.scene import Scene as SceneCfg
from curobo.types import GoalToolPose, JointState

from rokae_asset_utils import resolve_robot_config_for_workspace


class RokaeMotionGen:
    """基于 CuRobo V2 MotionPlanner 的离线规划封装。

    使用方式::

        gen = RokaeMotionGen()
        result = gen.plan_single(start_joint, target_pose)
        fk_result = gen.fk_single(joint_position)
        gen.update_world_from_dict(world_dict)
    """

    def __init__(
        self,
        robot_config_path: Path | None = None,
        collision_cache: dict[str, int] | None = None,
        use_cuda_graph: bool = False,
        num_warmup_iterations: int = 5,
    ) -> None:
        """初始化规划器。

        Args:
            robot_config_path: 机器人 YAML 配置路径，None 则用默认。
            collision_cache: 碰撞缓存配置，默认 {"obb": 64}。必须提供此参数才能让
                scene_collision_checker 正确初始化，否则后续无法 update_world。
            use_cuda_graph: 是否使用 CUDA graph 加速。默认 False，因为 CuRobo V2 的
                CUDA graph 不支持在 plan_pose 和 plan_cspace 之间动态切换。
            num_warmup_iterations: warmup 迭代次数。
        """
        if collision_cache is None:
            collision_cache = {"obb": 64}
        robot_cfg = resolve_robot_config_for_workspace(robot_config_path)
        self._cfg = MotionPlannerCfg.create(
            robot=robot_cfg,
            scene_model=None,
            collision_cache=collision_cache,
            use_cuda_graph=use_cuda_graph,
        )
        self._planner = MotionPlanner(self._cfg)
        self._planner.warmup(
            enable_graph=True,
            num_warmup_iterations=num_warmup_iterations,
        )

    # ------------------------------------------------------------------
    # 属性
    # ------------------------------------------------------------------

    @property
    def planner(self) -> MotionPlanner:
        return self._planner

    @property
    def joint_names(self) -> list[str]:
        return list(self._planner.joint_names)

    @property
    def tool_frames(self) -> list[str]:
        return list(self._planner.tool_frames)

    @property
    def default_joint_state(self) -> JointState:
        return self._planner.default_joint_state

    @property
    def interpolation_dt(self) -> float:
        return float(self._planner.trajopt_solver.config.interpolation_dt)

    # ------------------------------------------------------------------
    # FK
    # ------------------------------------------------------------------

    def fk_single(self, joint_position: list[float]) -> dict[str, Any]:
        """单次正运动学求解。

        Args:
            joint_position: 关节角列表（弧度）。

        Returns:
            {"position": [x,y,z], "quaternion": [w,qx,qy,qz]}
        """
        state = JointState.from_position(
            torch.tensor([joint_position], device="cuda", dtype=torch.float32),
            joint_names=self._planner.joint_names,
        )
        kin_state = self._planner.compute_kinematics(state)
        tool_pose = kin_state.tool_poses.get_link_pose(self._planner.tool_frames[0])
        pos = tool_pose.position.reshape(-1, 3).tolist()[0]
        quat = tool_pose.quaternion.reshape(-1, 4).tolist()[0]
        return {"position": pos, "quaternion": quat}

    # ------------------------------------------------------------------
    # World 更新
    # ------------------------------------------------------------------

    def update_world_from_dict(self, world_dict: dict[str, Any]) -> None:
        """用 world dict 更新障碍物场景。

        world_dict 格式: {"cuboid": {"name": {"dims": [...], "pose": [...]}, ...}}

        Args:
            world_dict: CuRobo cuboid world dict。
        """
        scene = SceneCfg.create(world_dict)
        self._planner.update_world(scene)

    def clear_world(self) -> None:
        """清除所有障碍物。"""
        self._planner.clear_scene_cache()

    # ------------------------------------------------------------------
    # 规划
    # ------------------------------------------------------------------

    def plan_single(
        self,
        start_joint: list[float],
        target_pose: list[float],
        max_attempts: int = 5,
        enable_graph_attempt: int = 1,
    ) -> dict[str, Any]:
        """点到点位姿规划。

        Args:
            start_joint: 起始关节角（弧度）。
            target_pose: 目标末端位姿 [x,y,z,qw,qx,qy,qz]（四元数 wxyz）。
            max_attempts: 最大尝试次数。
            enable_graph_attempt: 启用图搜索的尝试序号。

        Returns:
            包含 trajectory_points、interpolation_dt、solve_time、status 的字典。
        """
        current_state = JointState.from_position(
            torch.tensor([start_joint], device="cuda", dtype=torch.float32),
            joint_names=self._planner.joint_names,
        )
        goal = GoalToolPose(
            tool_frames=self._planner.tool_frames,
            position=torch.tensor(
                [[[[[target_pose[0], target_pose[1], target_pose[2]]]]]],
                device="cuda", dtype=torch.float32,
            ),
            quaternion=torch.tensor(
                [[[[[target_pose[3], target_pose[4], target_pose[5], target_pose[6]]]]]],
                device="cuda", dtype=torch.float32,
            ),
        )
        return self._run_plan(
            self._planner.plan_pose,
            goal, current_state, max_attempts, enable_graph_attempt,
        )

    def plan_single_js(
        self,
        start_joint: list[float],
        goal_joint: list[float],
        max_attempts: int = 5,
        enable_graph_attempt: int = 1,
    ) -> dict[str, Any]:
        """关节目标规划。

        Args:
            start_joint: 起始关节角（弧度）。
            goal_joint: 目标关节角（弧度）。
            max_attempts: 最大尝试次数。
            enable_graph_attempt: 启用图搜索的尝试序号。

        Returns:
            包含 trajectory_points、interpolation_dt、solve_time、status 的字典。
        """
        current_state = JointState.from_position(
            torch.tensor([start_joint], device="cuda", dtype=torch.float32),
            joint_names=self._planner.joint_names,
        )
        goal_state = JointState.from_position(
            torch.tensor([goal_joint], device="cuda", dtype=torch.float32),
            joint_names=self._planner.joint_names,
        )
        return self._run_plan(
            self._planner.plan_cspace,
            goal_state, current_state, max_attempts, enable_graph_attempt,
        )

    # ------------------------------------------------------------------
    # 内部方法
    # ------------------------------------------------------------------

    def _run_plan(
        self,
        plan_fn: Any,
        goal: Any,
        current_state: JointState,
        max_attempts: int,
        enable_graph_attempt: int,
    ) -> dict[str, Any]:
        """执行规划并提取结果。"""
        t0 = time.monotonic()
        result = plan_fn(
            goal, current_state,
            max_attempts=max_attempts,
            enable_graph_attempt=enable_graph_attempt,
        )
        wall_time = time.monotonic() - t0

        if result is None or not result.success.any():
            return {
                "success": False,
                "trajectory_points": [],
                "joint_names": [],
                "interpolation_dt": self.interpolation_dt,
                "solve_time": wall_time,
                "status": "planner_returned_none" if result is None else "optimization_failed",
            }

        interpolated = result.get_interpolated_plan()
        waypoints = interpolated.position.reshape(-1, interpolated.position.shape[-1]).tolist()
        joint_names = (
            list(interpolated.joint_names)
            if interpolated.joint_names
            else self._planner.joint_names
        )

        return {
            "success": True,
            "trajectory_points": waypoints,
            "joint_names": joint_names,
            "interpolation_dt": self.interpolation_dt,
            "solve_time": float(result.solve_time),
            "total_time": float(result.total_time),
            "wall_time": wall_time,
            "waypoint_count": len(waypoints),
            "status": "success",
        }
