#!/usr/bin/env python3
"""
Evaluate RobotStudio logger CSVs against Pinocchio + hpp-fcl full-scene collision.

Usage (from repo root)::
    python scripts/evaluate_collision_csv.py --csv path/to/traj1_obstacle1.csv
    python scripts/evaluate_collision_csv.py --csv a.csv --csv b.csv --waypoints-only

See FEATURE_4_HANDOFF.md for CSV schema and label codes.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

# Repo root on path
_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from core.collision_report import (  # noqa: E402
    CollisionCsvEvalSummary,
    WaypointCollisionEval,
    confusion_binary,
)
from core.trajectory_collision_checker import (  # noqa: E402
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


def _read_rows(path: Path) -> List[Dict[str, Any]]:
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames:
            reader.fieldnames = [h.strip() for h in reader.fieldnames]
        missing = [c for c in EXPECTED_COLUMNS if c not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{path}: missing columns {missing}")
        return list(reader)


def evaluate_file(
    checker,
    path: Path,
    waypoints_only: bool,
) -> CollisionCsvEvalSummary:
    rows = _read_rows(path)
    labels: List[int] = []
    preds: List[int] = []
    details: List[WaypointCollisionEval] = []

    for idx, row in enumerate(rows):
        at_wp = int(float(row["is_at_waypoint"]))
        if waypoints_only and at_wp != 1:
            continue
        label = int(float(row["is_collision"]))
        q_deg = np.array(
            [
                float(row["rs_j1_deg"]),
                float(row["rs_j2_deg"]),
                float(row["rs_j3_deg"]),
                float(row["rs_j4_deg"]),
                float(row["rs_j5_deg"]),
                float(row["rs_j6_deg"]),
            ],
            dtype=float,
        )
        q = np.deg2rad(q_deg)
        res = checker.check_waypoint(q)
        pred = checker.predict_code(res)
        labels.append(label)
        preds.append(pred)
        # Code match: exact equality (0–4)
        match = pred == label
        details.append(
            WaypointCollisionEval(
                row_index=idx,
                time_ms=float(row["time_ms"]),
                label_code=label,
                predicted_code=pred,
                match=match,
                is_at_waypoint=at_wp,
            )
        )

    if not labels:
        return CollisionCsvEvalSummary(
            n_rows=len(rows),
            n_evaluated=0,
            n_label_collision=0,
            n_pred_collision=0,
            true_negatives=0,
            false_positives=0,
            false_negatives=0,
            true_positives=0,
            code_matches_on_collision_rows=0,
            collision_rows=0,
            per_waypoint=details,
        )

    labels_arr = np.array(labels, dtype=int)
    preds_arr = np.array(preds, dtype=int)
    tn, fp, fn, tp = confusion_binary(labels_arr, preds_arr)

    collision_rows = int(np.sum(labels_arr != 0))
    code_match_coll = int(
        np.sum((labels_arr != 0) & (preds_arr == labels_arr))
    )

    return CollisionCsvEvalSummary(
        n_rows=len(rows),
        n_evaluated=len(labels),
        n_label_collision=int(np.sum(labels_arr != 0)),
        n_pred_collision=int(np.sum(preds_arr != 0)),
        true_negatives=tn,
        false_positives=fp,
        false_negatives=fn,
        true_positives=tp,
        code_matches_on_collision_rows=code_match_coll,
        collision_rows=collision_rows,
        per_waypoint=details,
    )


def print_summary(path: Path, s: CollisionCsvEvalSummary) -> None:
    print(f"\n=== {path.name} ===")
    print(f"Rows in file: {s.n_rows}  |  Evaluated: {s.n_evaluated}")
    print(
        f"Label collisions: {s.n_label_collision}  |  Predicted collisions: {s.n_pred_collision}"
    )
    print(f"Binary confusion TN={s.true_negatives} FP={s.false_positives} FN={s.false_negatives} TP={s.true_positives}")
    print(f"Recall (collision): {s.recall_collision:.3f}  Precision: {s.precision_collision:.3f}")
    print(
        f"Exact code match on collision rows: {s.code_matches_on_collision_rows}/{s.collision_rows}"
    )
    if s.false_negatives > 0:
        print("FALSE NEGATIVES (label collision, predicted clear):")
        for d in s.per_waypoint:
            if d.label_code != 0 and d.predicted_code == 0:
                print(
                    f"  row {d.row_index} t={d.time_ms}ms label={d.label_code} pred={d.predicted_code}"
                )


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate RS collision CSV vs Pinocchio scene")
    parser.add_argument(
        "--config",
        default=None,
        help="Path to collision_config.yaml (default: config/collision_config.yaml)",
    )
    parser.add_argument(
        "--csv",
        action="append",
        dest="csvs",
        required=True,
        help="Logger CSV path (repeat for multiple files)",
    )
    parser.add_argument(
        "--waypoints-only",
        action="store_true",
        help="Only rows with is_at_waypoint==1",
    )
    args = parser.parse_args()

    cfg = load_collision_config(args.config)
    checker = build_checker_from_collision_config(cfg, project_root=_ROOT)

    for csv_path in args.csvs:
        p = Path(csv_path)
        if not p.is_absolute():
            p = (_ROOT / p).resolve()
        s = evaluate_file(checker, p, waypoints_only=args.waypoints_only)
        print_summary(p, s)


if __name__ == "__main__":
    main()
