#!/usr/bin/env python3
"""
Feature 4 — End-to-end collision demo.

For every RobotStudio logger CSV provided, this script:

Phase A — Joint-space verification
    Read rs_j1..rs_j6 (degrees → radians), run the full-scene collision checker
    on every waypoint, report per-waypoint collisions and the trajectory verdict
    (FEASIBLE / INFEASIBLE).

Phase B — EAIK all-branches diagnostic (optional, --eaik)
    Read TCP (rs_x_mm,rs_y_mm,rs_z_mm,rs_qw,rs_qx,rs_qy,rs_qz), run EAIK at each
    waypoint to get every analytical branch, and run the collision checker on
    every branch. Reports whether ANY branch is collision-free per waypoint.

The two phases together answer:
    - Does our checker agree with RobotStudio's is_collision label?
    - When RS says "collision", does EAIK have a free branch we could pick?

Usage::

    # One or more CSVs
    python scripts/feature4_demo.py --csv path/to/traj1_obstacle1.csv

    # Whole folder of CSVs
    python scripts/feature4_demo.py --csv-dir path/to/csvs/

    # Pick mode
    python scripts/feature4_demo.py --csv x.csv --mode early_termination

    # Add EAIK branch diagnostic
    python scripts/feature4_demo.py --csv x.csv --eaik

See FEATURE_4_HANDOFF.md for CSV schema and obstacle definitions.
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from core.trajectory_collision_checker import (  # noqa: E402
    TrajectoryCollisionChecker,
    TrajectoryCollisionReport,
    build_checker_from_collision_config,
)
from utils.collision_config_loader import load_collision_config  # noqa: E402

EXPECTED_COLUMNS = [
    "time_ms",
    "rs_j1_deg",
    "rs_j2_deg",
    "rs_j3_deg",
    "rs_j4_deg",
    "rs_j5_deg",
    "rs_j6_deg",
    "speed_mm_per_s",
    "cf1",
    "cf4",
    "cf6",
    "cfx",
    "rs_x_mm",
    "rs_y_mm",
    "rs_z_mm",
    "rs_qw",
    "rs_qx",
    "rs_qy",
    "rs_qz",
    "linear_acceleration_mm_s_2",
    "is_at_waypoint",
    "is_collision",
]

LABEL_TO_OBSTACLE = {
    1: "Obstacle_1 (Box A)",
    2: "Obstacle_2 (Box B)",
    3: "Obstacle_3 (Cylinder C)",
    4: "Self-collision",
}


# ---------------------------------------------------------------------------
# CSV loading
# ---------------------------------------------------------------------------

@dataclass
class CsvData:
    path: Path
    n_rows: int
    q_traj_rad: np.ndarray  # (N, 6)
    tcp_pos_m: np.ndarray   # (N, 3)
    tcp_quat: np.ndarray    # (N, 4) qw, qx, qy, qz
    labels: np.ndarray      # (N,) ints
    is_at_waypoint: np.ndarray  # (N,) ints


def load_logger_csv(path: Path) -> CsvData:
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames:
            reader.fieldnames = [h.strip() for h in reader.fieldnames]
        missing = [c for c in EXPECTED_COLUMNS if c not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{path}: missing columns {missing}")
        rows = list(reader)

    n = len(rows)
    q = np.zeros((n, 6))
    pos = np.zeros((n, 3))
    quat = np.zeros((n, 4))
    labels = np.zeros(n, dtype=int)
    at_wp = np.zeros(n, dtype=int)

    for i, r in enumerate(rows):
        q[i] = np.deg2rad([
            float(r["rs_j1_deg"]),
            float(r["rs_j2_deg"]),
            float(r["rs_j3_deg"]),
            float(r["rs_j4_deg"]),
            float(r["rs_j5_deg"]),
            float(r["rs_j6_deg"]),
        ])
        pos[i] = [
            float(r["rs_x_mm"]) / 1000.0,
            float(r["rs_y_mm"]) / 1000.0,
            float(r["rs_z_mm"]) / 1000.0,
        ]
        quat[i] = [
            float(r["rs_qw"]),
            float(r["rs_qx"]),
            float(r["rs_qy"]),
            float(r["rs_qz"]),
        ]
        labels[i] = int(float(r["is_collision"]))
        at_wp[i] = int(float(r["is_at_waypoint"]))

    return CsvData(
        path=path,
        n_rows=n,
        q_traj_rad=q,
        tcp_pos_m=pos,
        tcp_quat=quat,
        labels=labels,
        is_at_waypoint=at_wp,
    )


# ---------------------------------------------------------------------------
# Phase A — joint-space verification
# ---------------------------------------------------------------------------

def print_phase_a(data: CsvData, report: TrajectoryCollisionReport) -> None:
    print(f"\n--- Phase A : joint-space (mode={report.mode}) ---")
    print(f"Waypoints in trajectory : {data.n_rows}")
    print(f"Mode                    : {report.mode}")
    print(f"Collision-free          : {report.collision_free}")
    print(f"Global min clearance    : {report.global_min_clearance_m:.4f} m")

    label_collision_rows = np.where(data.labels != 0)[0]
    pred_collision_rows = [
        i for i, c in enumerate(report.per_waypoint_predicted_code) if c != 0
    ]
    print(f"RS label collisions @   : {label_collision_rows.tolist()}")
    print(f"Predicted collisions @  : {pred_collision_rows}")

    if not report.collision_free:
        print(
            f"First collision         : waypoint {report.first_violation_waypoint_idx}, "
            f"pair={report.first_violation_pair}, code={report.first_violation_code} "
            f"({LABEL_TO_OBSTACLE.get(report.first_violation_code or 0, 'unknown')})"
        )

    if report.mode == "full_sweep":
        if pred_collision_rows:
            print("Per-colliding-waypoint detail:")
            for wp in pred_collision_rows:
                code = report.per_waypoint_predicted_code[wp]
                pairs = report.per_waypoint_pairs[wp]
                ls_lbl = LABEL_TO_OBSTACLE.get(code, f"code {code}")
                pretty = ", ".join(f"{a}↔{b}" for a, b in pairs)
                print(f"  WP {wp:>4}  (label={data.labels[wp]} pred={code} → {ls_lbl})  pairs: {pretty}")

    # Quick agreement metrics (binary)
    label_bin = (data.labels != 0)
    pred_bin = np.array(report.per_waypoint_predicted_code, dtype=int) != 0
    n_eval = min(len(label_bin), len(pred_bin))
    label_bin, pred_bin = label_bin[:n_eval], pred_bin[:n_eval]
    tp = int(np.sum(label_bin & pred_bin))
    fp = int(np.sum(~label_bin & pred_bin))
    fn = int(np.sum(label_bin & ~pred_bin))
    tn = int(np.sum(~label_bin & ~pred_bin))
    print(f"Binary agreement (eval={n_eval}): TP={tp} FP={fp} FN={fn} TN={tn}")


# ---------------------------------------------------------------------------
# Phase B — EAIK branch diagnostic
# ---------------------------------------------------------------------------

@dataclass
class BranchSummary:
    n_total: int = 0
    n_free: int = 0
    n_colliding: int = 0
    sample_free_q_deg: Optional[List[float]] = None
    sample_pair: Optional[Tuple[str, str]] = None


def _build_eaik_solver(urdf_abs: str, ee_frame_name: str = "ee_link"):
    """Build an EAIK solver against the same URDF used by the checker."""
    from core.eaik_ik_solver import EAIKConfig, EAIKIKSolver
    from utils.urdf_loader import load_robot_model_eaik

    rm = load_robot_model_eaik(urdf_abs, ee_frame_name=ee_frame_name)
    return EAIKIKSolver(rm, EAIKConfig(ee_frame_name=ee_frame_name))


def run_phase_b(
    data: CsvData,
    checker: TrajectoryCollisionChecker,
    eaik_solver,
    only_collision_rows: bool = True,
) -> List[Optional[BranchSummary]]:
    """For each waypoint, run EAIK to enumerate branches and test each for collision."""
    summaries: List[Optional[BranchSummary]] = [None] * data.n_rows

    target_rows = (
        list(np.where(data.labels != 0)[0])
        if only_collision_rows
        else list(range(data.n_rows))
    )

    for i in target_rows:
        _, _, info = eaik_solver.solve(
            data.tcp_pos_m[i], data.tcp_quat[i], q_init=None
        )
        all_q = info.get("all_solutions") or []
        if not all_q:
            summaries[i] = BranchSummary()
            continue
        n_free = 0
        n_total = 0
        sample_free: Optional[List[float]] = None
        sample_pair: Optional[Tuple[str, str]] = None
        for q in all_q:
            q_arr = np.asarray(q, dtype=float).flatten()
            if q_arr.shape[0] < 6:
                continue
            n_total += 1
            res = checker.check_waypoint(q_arr)
            if not res.has_collision:
                n_free += 1
                if sample_free is None:
                    sample_free = np.degrees(q_arr).round(3).tolist()
            elif sample_pair is None and res.colliding_pairs:
                sample_pair = res.colliding_pairs[0]
        summaries[i] = BranchSummary(
            n_total=n_total,
            n_free=n_free,
            n_colliding=n_total - n_free,
            sample_free_q_deg=sample_free,
            sample_pair=sample_pair,
        )
    return summaries


def print_phase_b(data: CsvData, summaries: List[Optional[BranchSummary]]) -> None:
    print("\n--- Phase B : EAIK all-branches diagnostic ---")
    rows = [(i, s) for i, s in enumerate(summaries) if s is not None]
    if not rows:
        print("(no rows evaluated)")
        return

    all_branches_collide_rows = []
    has_free_branch_rows = []
    no_branches_rows = []

    for i, s in rows:
        label = data.labels[i]
        if s.n_total == 0:
            tag = "NO EAIK BRANCHES"
            no_branches_rows.append(i)
        elif s.n_free == 0:
            tag = "ALL BRANCHES COLLIDE"
            all_branches_collide_rows.append(i)
        else:
            tag = f"{s.n_free}/{s.n_total} branches FREE"
            has_free_branch_rows.append(i)
        extra = ""
        if s.n_free > 0 and s.sample_free_q_deg is not None:
            extra = f"  free-q-deg≈{s.sample_free_q_deg}"
        elif s.sample_pair is not None:
            extra = f"  e.g. pair={s.sample_pair[0]}↔{s.sample_pair[1]}"
        print(f"  WP {i:>4}  label={label}  {tag}{extra}")

    print(f"Summary: ALL_COLLIDE={len(all_branches_collide_rows)}  "
          f"HAS_FREE={len(has_free_branch_rows)}  NO_BRANCHES={len(no_branches_rows)}")


# ---------------------------------------------------------------------------
# Per-CSV orchestration
# ---------------------------------------------------------------------------

def process_csv(
    path: Path,
    checker: TrajectoryCollisionChecker,
    mode: str,
    eaik_solver,
    eaik_full_traj: bool,
) -> bool:
    """Returns the feasibility verdict (True = feasible)."""
    print("\n" + "=" * 78)
    print(f"FILE: {path}")
    print("=" * 78)
    data = load_logger_csv(path)
    report = checker.check_trajectory(data.q_traj_rad, mode=mode)
    print_phase_a(data, report)

    if eaik_solver is not None:
        summaries = run_phase_b(
            data, checker, eaik_solver, only_collision_rows=not eaik_full_traj
        )
        print_phase_b(data, summaries)

    print()
    if report.collision_free:
        print(f"VERDICT: TRAJECTORY FEASIBLE — no collision detected by mesh-vs-mesh check.")
        return True
    code = report.first_violation_code
    obj = LABEL_TO_OBSTACLE.get(code or 0, f"code {code}")
    print(
        f"VERDICT: TRAJECTORY INFEASIBLE — first hit at waypoint "
        f"{report.first_violation_waypoint_idx} with {obj}."
    )
    return False


def _gather_csv_paths(args) -> List[Path]:
    paths: List[Path] = []
    for c in args.csvs or []:
        p = Path(c)
        if not p.is_absolute():
            p = (_ROOT / p).resolve()
        paths.append(p)
    for d in args.csv_dirs or []:
        p = Path(d)
        if not p.is_absolute():
            p = (_ROOT / p).resolve()
        paths.extend(sorted(p.glob("*.csv")))
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--config", default=None, help="Path to collision_config.yaml")
    parser.add_argument("--csv", action="append", dest="csvs", help="CSV path (repeatable)")
    parser.add_argument("--csv-dir", action="append", dest="csv_dirs", help="Directory of CSVs")
    parser.add_argument(
        "--mode",
        choices=("early_termination", "full_sweep"),
        default=None,
        help="Override mode from config",
    )
    parser.add_argument(
        "--eaik",
        action="store_true",
        help="Run EAIK all-branches diagnostic on collision-labeled rows",
    )
    parser.add_argument(
        "--eaik-full-traj",
        action="store_true",
        help="With --eaik: also evaluate non-collision-labeled waypoints",
    )
    parser.add_argument(
        "--ee-frame",
        default="ee_link",
        help="EE frame name for EAIK (default: ee_link)",
    )
    args = parser.parse_args()

    csv_paths = _gather_csv_paths(args)
    if not csv_paths:
        parser.error("Provide --csv or --csv-dir")

    cfg = load_collision_config(args.config)
    if args.mode is not None:
        cfg["mode"] = args.mode
    mode = str(cfg.get("mode", "full_sweep"))

    print(f"Mode: {mode}")
    print(f"URDF: {cfg.get('urdf_path')}")
    print(f"Whitelist (configured): {cfg.get('whitelist_pairs') or []}")

    checker = build_checker_from_collision_config(cfg, project_root=_ROOT)
    n_pairs = len(checker.geom_model.collisionPairs)
    print(f"Active collision pairs after policy: {n_pairs}")
    print(f"Obstacles registered: {sorted(checker.env_name_to_code.keys())}")

    eaik_solver = None
    if args.eaik:
        urdf_abs = checker.scene.urdf_abs
        try:
            eaik_solver = _build_eaik_solver(urdf_abs, ee_frame_name=args.ee_frame)
            print(f"EAIK solver loaded for {urdf_abs}")
        except Exception as e:  # pragma: no cover
            print(f"WARNING: EAIK init failed ({e}); skipping Phase B")
            eaik_solver = None

    overall = {"feasible": 0, "infeasible": 0}
    for p in csv_paths:
        try:
            ok = process_csv(p, checker, mode, eaik_solver, args.eaik_full_traj)
        except Exception as e:  # pragma: no cover
            print(f"\nERROR on {p}: {e}")
            continue
        overall["feasible" if ok else "infeasible"] += 1

    print("\n" + "=" * 78)
    print(f"OVERALL: {overall['feasible']} feasible, {overall['infeasible']} infeasible across {len(csv_paths)} CSV(s)")
    print("=" * 78)


if __name__ == "__main__":
    main()
