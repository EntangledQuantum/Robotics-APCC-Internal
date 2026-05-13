#!/usr/bin/env python3
"""Collision pair policy: ROBOT×ROBOT, ROBOT×ENV, no ENV×ENV, whitelist (Feature 4).

Whitelist semantics
-------------------
A whitelisted pair (e.g. ``["midsole_link", "knife_blade"]``) is the **only**
contact allowed between those two named geometries. The whitelist is matched on
exact geometry-object names from the unified ``GeometryModel``.

Conditional behaviour:
    If either endpoint of a whitelisted pair is not present in the loaded
    geometry (e.g. midsole is **optional**: not all setups mount one), the
    entry has no effect. Every robot↔knife pair remains enabled. We log a
    warning so silent typos do not pass.
"""

from __future__ import annotations

import logging
from typing import Iterable, List, Optional, Sequence, Set

import pinocchio as pin

logger = logging.getLogger(__name__)


def _clear_all_pairs(geom_model: pin.GeometryModel) -> None:
    # Pinocchio exposes collisionPairs as a vector; remove until empty.
    while len(geom_model.collisionPairs) > 0:
        geom_model.removeCollisionPair(geom_model.collisionPairs[0])


def _whitelisted(
    name_i: str,
    name_j: str,
    whitelist_pairs: Sequence[Sequence[str]],
) -> bool:
    for pair in whitelist_pairs:
        if len(pair) != 2:
            continue
        a, b = str(pair[0]), str(pair[1])
        if (name_i == a and name_j == b) or (name_i == b and name_j == a):
            return True
    return False


def rebuild_collision_pairs(
    geom_model: pin.GeometryModel,
    env_geom_indices: Set[int],
    min_joint_gap: int = 1,
    whitelist_pairs: Optional[Iterable[Sequence[str]]] = None,
) -> List[List[str]]:
    """
    Replace all collision pairs with the Feature 4 policy:

    - ROBOT×ROBOT: all pairs except adjacent (|parentJoint diff| ≤ min_joint_gap)
    - ROBOT×ENV: all pairs
    - ENV×ENV: none
    - Whitelist: exact name pairs removed (only when **both** names exist in
      ``geom_model``; otherwise the entry is silently ignored — see module docstring)

    Returns:
        The list of effective whitelist entries (those whose both endpoints
        actually exist in the model). Useful for logging.
    """
    names = [go.name for go in geom_model.geometryObjects]
    name_set = set(names)
    n = len(names)
    all_idx = set(range(n))
    robot_idx = sorted(all_idx - set(env_geom_indices))
    env_idx = sorted(env_geom_indices)

    # Partition whitelist by whether both endpoints exist
    raw_wl = [list(p) for p in (whitelist_pairs or []) if len(p) == 2]
    effective_wl: List[List[str]] = []
    for a, b in raw_wl:
        a_in = str(a) in name_set
        b_in = str(b) in name_set
        if a_in and b_in:
            effective_wl.append([str(a), str(b)])
        else:
            missing = [n for n, present in ((a, a_in), (b, b_in)) if not present]
            logger.warning(
                "Whitelist pair %s ignored (geometry %s not in model). "
                "Robot↔%s collisions remain ENABLED.",
                [str(a), str(b)],
                missing,
                str(b) if a_in else str(a),
            )
    wl = effective_wl

    _clear_all_pairs(geom_model)

    def add_pair(i: int, j: int) -> None:
        if i == j:
            return
        if i > j:
            i, j = j, i
        if _whitelisted(names[i], names[j], wl):
            return
        geom_model.addCollisionPair(pin.CollisionPair(i, j))

    # ROBOT × ROBOT
    for a in range(len(robot_idx)):
        i = robot_idx[a]
        for b in range(a + 1, len(robot_idx)):
            j = robot_idx[b]
            j1 = geom_model.geometryObjects[i].parentJoint
            j2 = geom_model.geometryObjects[j].parentJoint
            if abs(j1 - j2) <= min_joint_gap:
                continue
            add_pair(i, j)

    # ROBOT × ENVIRONMENT
    for i in robot_idx:
        for j in env_idx:
            add_pair(i, j)

    return effective_wl
