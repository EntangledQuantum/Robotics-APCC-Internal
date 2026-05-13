#!/usr/bin/env python3
"""
Trajectory-level collision checking for full scene (Feature 4).

Composes Pinocchio ``computeCollisions`` / ``computeDistances`` like
``SelfCollisionChecker``, and maps contacts to RobotStudio-style codes:
0 clear, 1–3 obstacle, 4 self-collision only.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
import pinocchio as pin

from core.collision_checker import CollisionResult
from core.scene_builder import SceneContext

logger = logging.getLogger(__name__)


@dataclass
class TrajectoryCollisionReport:
    """Result of scanning a joint trajectory.

    Attributes:
        mode: ``"early_termination"`` or ``"full_sweep"`` (full_sweep evaluates
            every waypoint; early_termination stops at the first violation).
        per_waypoint_pairs: List of colliding pair lists per waypoint (only filled
            in full_sweep mode; otherwise empty for waypoints past the first hit).
    """

    collision_free: bool
    first_violation_waypoint_idx: Optional[int]
    first_violation_pair: Optional[Tuple[str, str]]
    first_violation_code: Optional[int]
    global_min_clearance_m: float
    per_waypoint_min_clearance_m: List[float] = field(default_factory=list)
    per_waypoint_predicted_code: List[int] = field(default_factory=list)
    per_waypoint_pairs: List[List[Tuple[str, str]]] = field(default_factory=list)
    mode: str = "early_termination"


class TrajectoryCollisionChecker:
    """
    Full-scene collision checker for a built :class:`SceneContext`
    (URDF + obstacles, pairs already installed).
    """

    def __init__(
        self,
        scene: SceneContext,
        robot_geom_indices: Optional[Set[int]] = None,
    ):
        self.scene = scene
        self.model = scene.model
        self.data = scene.data
        self.geom_model = scene.geom_model
        self.geom_data = scene.geom_data
        self.env_geom_indices = set(scene.env_geom_indices)
        if robot_geom_indices is None:
            n = len(self.geom_model.geometryObjects)
            self.robot_geom_indices = set(range(n)) - self.env_geom_indices
        else:
            self.robot_geom_indices = set(robot_geom_indices)
        self.env_name_to_code = dict(scene.env_name_to_code)

    def _pad(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).flatten()
        if len(q) < self.model.nq:
            q_full = np.zeros(self.model.nq)
            q_full[: len(q)] = q
            return q_full
        return q[: self.model.nq]

    def check_waypoint(self, q: np.ndarray) -> CollisionResult:
        """Same detailed output as ``SelfCollisionChecker.check``."""
        q_full = self._pad(q)
        pin.computeCollisions(
            self.model,
            self.data,
            self.geom_model,
            self.geom_data,
            q_full,
            False,
        )
        pin.computeDistances(
            self.model,
            self.data,
            self.geom_model,
            self.geom_data,
            q_full,
        )
        colliding: List[Tuple[str, str]] = []
        all_dist: List[Tuple[str, str, float]] = []
        min_dist = float("inf")
        closest: Tuple[str, str] = ("", "")

        for i in range(len(self.geom_model.collisionPairs)):
            cp = self.geom_model.collisionPairs[i]
            n1 = self.geom_model.geometryObjects[cp.first].name
            n2 = self.geom_model.geometryObjects[cp.second].name
            d = self.geom_data.distanceResults[i].min_distance
            all_dist.append((n1, n2, d))
            if self.geom_data.collisionResults[i].isCollision():
                colliding.append((n1, n2))
            if d < min_dist:
                min_dist = d
                closest = (n1, n2)

        return CollisionResult(
            has_collision=len(colliding) > 0,
            colliding_pairs=colliding,
            min_distance_m=min_dist if min_dist != float("inf") else -1.0,
            closest_pair=closest,
            all_distances=sorted(all_dist, key=lambda t: t[2]),
        )

    def predict_code(self, result: CollisionResult) -> int:
        """
        Map collision pairs to RS-style label.

        If any colliding pair touches a known obstacle name → 1/2/3.
        Else if collision → 4 (self / robot–robot or unknown).
        """
        if not result.has_collision:
            return 0
        codes_found: List[int] = []
        for a, b in result.colliding_pairs:
            for name in (a, b):
                c = self.env_name_to_code.get(name)
                if c is not None:
                    codes_found.append(c)
        if codes_found:
            return min(codes_found)
        return 4

    def calibrate_robot_robot(
        self,
        n_samples: int = 20,
        seed: int = 42,
    ) -> List[Tuple[str, str]]:
        """
        Remove ROBOT×ROBOT pairs that collide in every sampled configuration.

        Environment pairs are never removed. Same logic as ``SelfCollisionChecker.calibrate``
        but restricted to pairs where both geometry indices are in ``robot_geom_indices``.
        """
        rng = np.random.RandomState(seed)
        lower = self.model.lowerPositionLimit[: self.model.nq]
        upper = self.model.upperPositionLimit[: self.model.nq]
        configs = [pin.neutral(self.model)]
        for _ in range(n_samples):
            configs.append(lower + rng.rand(self.model.nq) * (upper - lower))

        robot_only = self.robot_geom_indices
        pair_is_robot_robot = []
        for k in range(len(self.geom_model.collisionPairs)):
            cp = self.geom_model.collisionPairs[k]
            if cp.first in robot_only and cp.second in robot_only:
                pair_is_robot_robot.append(k)
            else:
                pair_is_robot_robot.append(-1)

        n_pairs = len(self.geom_model.collisionPairs)
        hit_counts = np.zeros(n_pairs, dtype=int)

        for q in configs:
            q_full = self._pad(q)
            pin.computeCollisions(
                self.model,
                self.data,
                self.geom_model,
                self.geom_data,
                q_full,
                False,
            )
            for k in range(n_pairs):
                if pair_is_robot_robot[k] < 0:
                    continue
                if self.geom_data.collisionResults[k].isCollision():
                    hit_counts[k] += 1

        n_total = len(configs)
        excluded_names: List[Tuple[str, str]] = []
        to_remove: List[pin.CollisionPair] = []
        for k in range(n_pairs):
            if pair_is_robot_robot[k] < 0:
                continue
            if hit_counts[k] == n_total:
                cp = self.geom_model.collisionPairs[k]
                to_remove.append(pin.CollisionPair(cp.first, cp.second))
                excluded_names.append(
                    (
                        self.geom_model.geometryObjects[cp.first].name,
                        self.geom_model.geometryObjects[cp.second].name,
                    )
                )

        for cp in to_remove:
            self.geom_model.removeCollisionPair(cp)

        self.geom_data = pin.GeometryData(self.geom_model)
        if excluded_names:
            logger.info(
                "Calibration excluded %d always-colliding ROBOT×ROBOT pairs",
                len(excluded_names),
            )
        return excluded_names

    def check_trajectory(
        self,
        q_traj: np.ndarray,
        mode: str = "early_termination",
        early_termination: Optional[bool] = None,
    ) -> TrajectoryCollisionReport:
        """
        Args:
            q_traj: shape (N, nq) or (N, 6) — padded internally.
            mode: ``"early_termination"`` (stop at first hit, fast) or
                ``"full_sweep"`` (evaluate every waypoint, diagnostic).
            early_termination: legacy bool; if set, overrides ``mode``.
        """
        q_traj = np.asarray(q_traj, dtype=float)
        if q_traj.ndim == 1:
            q_traj = q_traj.reshape(1, -1)

        if early_termination is not None:
            mode = "early_termination" if early_termination else "full_sweep"
        if mode not in ("early_termination", "full_sweep"):
            raise ValueError(
                f"Unknown mode: {mode!r}. Use 'early_termination' or 'full_sweep'."
            )
        stop_early = mode == "early_termination"

        per_clear: List[float] = []
        per_code: List[int] = []
        per_pairs: List[List[Tuple[str, str]]] = []
        first_idx: Optional[int] = None
        first_pair: Optional[Tuple[str, str]] = None
        first_code: Optional[int] = None
        global_min = float("inf")

        for i in range(len(q_traj)):
            res = self.check_waypoint(q_traj[i])
            per_clear.append(res.min_distance_m if res.min_distance_m >= 0 else float("inf"))
            code = self.predict_code(res)
            per_code.append(code)
            per_pairs.append(list(res.colliding_pairs))
            if res.min_distance_m >= 0 and res.min_distance_m < global_min:
                global_min = res.min_distance_m
            if res.has_collision and first_idx is None:
                first_idx = i
                first_code = code
                if res.colliding_pairs:
                    first_pair = res.colliding_pairs[0]
                if stop_early:
                    break

        if global_min == float("inf"):
            global_min = -1.0

        return TrajectoryCollisionReport(
            collision_free=first_idx is None,
            first_violation_waypoint_idx=first_idx,
            first_violation_pair=first_pair,
            first_violation_code=first_code,
            global_min_clearance_m=global_min,
            per_waypoint_min_clearance_m=per_clear,
            per_waypoint_predicted_code=per_code,
            per_waypoint_pairs=per_pairs,
            mode=mode,
        )


def build_checker_from_collision_config(
    cfg: Dict,
    project_root: Optional[Path] = None,
) -> TrajectoryCollisionChecker:
    """
    Convenience: load scene from config dict, rebuild pairs, optional calibration.

    ``cfg`` should be the ``collision`` subtree from YAML.
    """
    from core.collision_group_manager import rebuild_collision_pairs
    from core.scene_builder import build_scene_from_config

    if project_root is None:
        project_root = Path(__file__).resolve().parent.parent

    scene = build_scene_from_config(cfg, project_root=project_root)
    rebuild_collision_pairs(
        scene.geom_model,
        scene.env_geom_indices,
        min_joint_gap=int(cfg.get("min_joint_gap", 1)),
        whitelist_pairs=cfg.get("whitelist_pairs") or [],
    )
    scene.geom_data = pin.GeometryData(scene.geom_model)

    cal = cfg.get("calibration") or {}
    checker = TrajectoryCollisionChecker(scene)
    if cal.get("enabled", False):
        checker.calibrate_robot_robot(
            n_samples=int(cal.get("n_samples", 20)),
            seed=int(cal.get("seed", 42)),
        )
    return checker
