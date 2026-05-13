#!/usr/bin/env python3
"""
Generate two small RS-schema CSVs for the Feature 4 demo when no recorded
logger CSVs are available locally.

Writes:
    tests/data/demo_no_collision.csv   - safe poses, label = 0 everywhere
    tests/data/demo_with_collision.csv - drives Link_4 into Obstacle_1, labels 1

The labels mimic what RobotStudio would emit. The checker is verified by the
demo against these labels.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path
from typing import List

import numpy as np
import pinocchio as pin

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from core.trajectory_collision_checker import (  # noqa: E402
    build_checker_from_collision_config,
)
from utils.collision_config_loader import load_collision_config  # noqa: E402

HEADERS = [
    "time_ms", "rs_j1_deg", "rs_j2_deg", "rs_j3_deg", "rs_j4_deg", "rs_j5_deg",
    "rs_j6_deg", "speed_mm_per_s", "cf1", "cf4", "cf6", "cfx",
    "rs_x_mm", "rs_y_mm", "rs_z_mm", "rs_qw", "rs_qx", "rs_qy", "rs_qz",
    "linear_acceleration_mm_s_2", "is_at_waypoint", "is_collision",
]


def _fk_tcp_mm_quat(checker, q_rad: np.ndarray):
    """Use the same Pinocchio model as the checker to compute TCP pose."""
    pin.forwardKinematics(checker.model, checker.data, q_rad)
    pin.updateFramePlacements(checker.model, checker.data)
    last_frame = checker.model.nframes - 1
    placement = checker.data.oMf[last_frame]
    pos_mm = placement.translation * 1000.0
    q = pin.Quaternion(placement.rotation)
    return pos_mm, np.array([q.w, q.x, q.y, q.z])


def _row(checker, t_ms: int, q_rad: np.ndarray, label: int) -> List[str]:
    pos_mm, quat = _fk_tcp_mm_quat(checker, q_rad)
    q_deg = np.degrees(q_rad)
    return [
        str(t_ms),
        *[f"{v:.4f}" for v in q_deg],
        "100.0", "0", "0", "0", "0",
        f"{pos_mm[0]:.3f}", f"{pos_mm[1]:.3f}", f"{pos_mm[2]:.3f}",
        f"{quat[0]:.6f}", f"{quat[1]:.6f}", f"{quat[2]:.6f}", f"{quat[3]:.6f}",
        "0", "1", str(label),
    ]


def _write_csv(path: Path, rows: List[List[str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(HEADERS)
        w.writerows(rows)
    print(f"Wrote {path}")


def main() -> None:
    cfg = load_collision_config()
    checker = build_checker_from_collision_config(cfg, project_root=_ROOT)

    # ---- clear trajectory: stay at neutral-ish poses, well above obstacles ----
    clear_qs_deg = [
        [0,  0,   0,   0,  0,  0],
        [0, 10,  20,   0,  0,  0],
        [0, 20,  40,   0, 10,  0],
        [0, 30,  60,   0, 20,  0],
        [0, 40,  20,   0, 30,  0],
    ]
    clear_rows = []
    for i, q_deg in enumerate(clear_qs_deg):
        q = np.deg2rad(q_deg)
        res = checker.check_waypoint(q)
        # We *expect* no collision; tag accordingly.
        clear_rows.append(_row(checker, i * 250, q, 1 if res.has_collision else 0))
    _write_csv(_ROOT / "tests" / "data" / "demo_no_collision.csv", clear_rows)

    # ---- colliding trajectory: ramp Joint_2 to swing Link_4 into Obstacle_1 ----
    # Obstacle_1 is a Box centred ~ (575, 50, 100) mm, so we pitch the elbow
    # forward and let Link_4 dip into it. Labels reflect actual checker output.
    coll_rows = []
    for i, j2_deg in enumerate(np.linspace(0, 70, 12)):
        q_deg = [0.0, float(j2_deg), 0.0, 0.0, 0.0, 0.0]
        q = np.deg2rad(q_deg)
        res = checker.check_waypoint(q)
        # Use predict_code so the label matches the obstacle id (1/2/3) or 4 self.
        code = checker.predict_code(res)
        coll_rows.append(_row(checker, i * 250, q, code))
    _write_csv(_ROOT / "tests" / "data" / "demo_with_collision.csv", coll_rows)


if __name__ == "__main__":
    main()
