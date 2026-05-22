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
from curobo.types import GoalToolPose, JointState, ToolPoseCriteria

from rokae_asset_utils import resolve_robot_config_for_workspace


class RokaeMotionGen:
    """基于 CuRobo V2 MotionPlanner 的离线规划封装。

    使用方式::

        gen = RokaeMotionGen(speed_scale=0.5)
        result = gen.plan_single(start_joint, target_pose)
        result = gen.plan_grasp_single(start_joint, grasp_pose, approach_offset=-0.1)
        fk_result = gen.fk_single(joint_position)
        gen.update_world_from_dict(world_dict)
    """

    def __init__(
        self,
        robot_config_path: Path | None = None,
        collision_cache: dict[str, int] | None = None,
        use_cuda_graph: bool = False,
        num_warmup_iterations: int = 5,
        speed_scale: float = 1.0,
    ) -> None:
        """初始化规划器。

        Args:
            robot_config_path: 机器人 YAML 配置路径，None 则用默认。
            collision_cache: 碰撞缓存配置，默认 {"obb": 64}。必须提供此参数才能让
                scene_collision_checker 正确初始化，否则后续无法 update_world。
            use_cuda_graph: 是否使用 CUDA graph 加速。默认 False，因为 CuRobo V2 的
                CUDA graph 不支持在 plan_pose 和 plan_cspace 之间动态切换。
            num_warmup_iterations: warmup 迭代次数。
            speed_scale: 速度缩放因子 (0, 2.0]。通过修改 robot config 中的
                cspace.velocity_scale 实现。1.0 = 原速，0.5 = 半速，2.0 = 倍速。
        """
        if collision_cache is None:
            collision_cache = {"obb": 64}
        if not 0.0 < speed_scale <= 2.0:
            raise ValueError(f"speed_scale must be in (0, 2.0], got {speed_scale}")
        robot_cfg = resolve_robot_config_for_workspace(robot_config_path)
        if speed_scale != 1.0:
            kin = robot_cfg.setdefault("robot_cfg", {}).setdefault("kinematics", {})
            cspace = kin.setdefault("cspace", {})
            cspace["velocity_scale"] = speed_scale
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
        self._speed_scale = speed_scale

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

    @property
    def speed_scale(self) -> float:
        return self._speed_scale

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
    # 方向约束
    # ------------------------------------------------------------------

    def _apply_hold_vec_weight(self, hold_vec_weight: list[float] | None) -> dict | None:
        """应用方向保持约束到 planner。

        Args:
            hold_vec_weight: [x, y, z] 方向保持权重，1.0 = 保持方向，0.0 = 不约束。
                None 表示不应用额外约束。

        Returns:
            旧的 tool_pose_criteria 字典，用于后续恢复。None 表示未应用。
        """
        if hold_vec_weight is None:
            return None
        if len(hold_vec_weight) != 3:
            raise ValueError(f"hold_vec_weight must have 3 elements, got {len(hold_vec_weight)}")

        # V1 语义: 1.0 = 保持方向（约束强），0.0 = 不约束
        # V2 ToolPoseCriteria: 高权重 = 跟踪强，低权重 = 跟踪弱
        # 映射: orientation_weight = max(0.001, 1.0 - hold_vec_weight[i])
        rpy_weights = [max(0.001, 1.0 - w) for w in hold_vec_weight]
        criteria = ToolPoseCriteria(
            terminal_pose_axes_weight_factor=[1.0, 1.0, 1.0] + rpy_weights,
            non_terminal_pose_axes_weight_factor=[0.0, 0.0, 0.0] + [w * 0.1 for w in rpy_weights],
        )
        tool_frame = self._planner.tool_frames[0]
        # 保存旧 criteria（从 planner 内部获取）
        old_criteria = {tool_frame: criteria}
        self._planner.update_tool_pose_criteria({tool_frame: criteria})
        return old_criteria

    def _restore_criteria(self, old_criteria: dict | None) -> None:
        """恢复旧的 tool_pose_criteria。"""
        if old_criteria is None:
            return
        # 恢复为默认（全权重跟踪）
        default_criteria = ToolPoseCriteria(
            terminal_pose_axes_weight_factor=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        )
        for frame in old_criteria:
            self._planner.update_tool_pose_criteria({frame: default_criteria})

    # ------------------------------------------------------------------
    # 规划
    # ------------------------------------------------------------------

    def plan_single(
        self,
        start_joint: list[float],
        target_pose: list[float],
        max_attempts: int = 5,
        enable_graph_attempt: int = 1,
        hold_vec_weight: list[float] | None = None,
    ) -> dict[str, Any]:
        """点到点位姿规划。

        Args:
            start_joint: 起始关节角（弧度）。
            target_pose: 目标末端位姿 [x,y,z,qw,qx,qy,qz]（四元数 wxyz）。
            max_attempts: 最大尝试次数。
            enable_graph_attempt: 启用图搜索的尝试序号。
            hold_vec_weight: 可选方向保持权重 [x, y, z]，1.0=保持，0.0=不约束。

        Returns:
            包含 trajectory_points、interpolation_dt、solve_time、status 的字典。
        """
        old_criteria = self._apply_hold_vec_weight(hold_vec_weight)
        try:
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
        finally:
            self._restore_criteria(old_criteria)

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

    def plan_grasp_single(
        self,
        start_joint: list[float],
        grasp_pose: list[float],
        approach_axis: str = "z",
        approach_offset: float = -0.15,
        approach_in_tool_frame: bool = True,
        lift_axis: str = "z",
        lift_offset: float = -0.15,
        lift_in_tool_frame: bool = True,
        plan_approach: bool = True,
        plan_lift: bool = True,
    ) -> dict[str, Any]:
        """抓取规划：approach → grasp → lift。

        Args:
            start_joint: 起始关节角（弧度）。
            grasp_pose: 抓取目标位姿 [x,y,z,qw,qx,qy,qz]（四元数 wxyz）。
            approach_axis: 接近轴，"x"/"y"/"z"。
            approach_offset: 接近偏移量（米），负值表示沿轴负方向。
            approach_in_tool_frame: 是否在工具坐标系中计算偏移。
            lift_axis: 提升轴。
            lift_offset: 提升偏移量（米）。
            lift_in_tool_frame: 是否在工具坐标系中计算偏移。
            plan_approach: 是否规划接近段。
            plan_lift: 是否规划提升段。

        Returns:
            包含 success、trajectory_points、solve_time、status 的字典。
        """
        current_state = JointState.from_position(
            torch.tensor([start_joint], device="cuda", dtype=torch.float32),
            joint_names=self._planner.joint_names,
        )
        goal = GoalToolPose(
            tool_frames=self._planner.tool_frames,
            position=torch.tensor(
                [[[[[grasp_pose[0], grasp_pose[1], grasp_pose[2]]]]]],
                device="cuda", dtype=torch.float32,
            ),
            quaternion=torch.tensor(
                [[[[[grasp_pose[3], grasp_pose[4], grasp_pose[5], grasp_pose[6]]]]]],
                device="cuda", dtype=torch.float32,
            ),
        )

        t0 = time.monotonic()
        result = self._planner.plan_grasp(
            goal, current_state,
            grasp_approach_axis=approach_axis,
            grasp_approach_offset=approach_offset,
            grasp_approach_in_tool_frame=approach_in_tool_frame,
            grasp_lift_axis=lift_axis,
            grasp_lift_offset=lift_offset,
            grasp_lift_in_tool_frame=lift_in_tool_frame,
            plan_approach_to_grasp=plan_approach,
            plan_grasp_to_lift=plan_lift,
        )
        wall_time = time.monotonic() - t0

        approach_ok = (result.approach_success is not None and result.approach_success.any())
        grasp_ok = (result.grasp_success is not None and result.grasp_success.any())
        lift_ok = (result.lift_success is not None and result.lift_success.any())

        # 合并已有轨迹段（即使部分失败也返回已成功的段）
        all_waypoints = []
        joint_names = list(self._planner.joint_names)
        for segment in [result.approach_interpolated_trajectory,
                        result.grasp_interpolated_trajectory,
                        result.lift_interpolated_trajectory]:
            if segment is not None:
                seg_wp = segment.position.reshape(-1, segment.position.shape[-1]).tolist()
                all_waypoints.extend(seg_wp)

        any_success = approach_ok or grasp_ok or lift_ok
        if not any_success:
            return {
                "success": False,
                "trajectory_points": [],
                "joint_names": [],
                "interpolation_dt": self.interpolation_dt,
                "solve_time": wall_time,
                "status": result.status if result else "planner_returned_none",
            }

        # 部分成功（如 approach 成功但 grasp/lift 失败）
        if not (result.success is not None and result.success.any()):
            return {
                "success": False,
                "trajectory_points": all_waypoints,
                "joint_names": joint_names,
                "interpolation_dt": self.interpolation_dt,
                "solve_time": wall_time,
                "status": result.status or "partial_failure",
                "approach_success": approach_ok,
                "grasp_success": grasp_ok,
                "lift_success": lift_ok,
                "waypoint_count": len(all_waypoints),
            }

        return {
            "success": True,
            "trajectory_points": all_waypoints,
            "joint_names": joint_names,
            "interpolation_dt": self.interpolation_dt,
            "solve_time": wall_time,
            "total_time": wall_time,
            "wall_time": wall_time,
            "waypoint_count": len(all_waypoints),
            "status": "success",
            "approach_success": approach_ok,
            "grasp_success": grasp_ok,
            "lift_success": lift_ok,
        }

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
