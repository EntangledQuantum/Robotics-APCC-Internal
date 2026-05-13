#!/usr/bin/env python3
"""Dataclasses for full-scene / trajectory collision reporting (Feature 4)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


@dataclass
class WaypointCollisionEval:
    """Per-row CSV evaluation result."""

    row_index: int
    time_ms: float
    label_code: int
    predicted_code: int
    match: bool
    is_at_waypoint: Optional[int] = None


@dataclass
class CollisionCsvEvalSummary:
    """Aggregate metrics after scanning a RobotStudio logger CSV."""

    n_rows: int
    n_evaluated: int
    n_label_collision: int
    n_pred_collision: int
    true_negatives: int
    false_positives: int
    false_negatives: int
    true_positives: int
    code_matches_on_collision_rows: int
    collision_rows: int
    per_waypoint: List[WaypointCollisionEval] = field(default_factory=list)

    @property
    def recall_collision(self) -> float:
        """Fraction of label collision rows where we also predicted collision."""
        if self.n_label_collision == 0:
            return 1.0
        # TP = predicted collision when label collision
        return self.true_positives / self.n_label_collision

    @property
    def precision_collision(self) -> float:
        if self.n_pred_collision == 0:
            return 1.0
        return self.true_positives / self.n_pred_collision


def confusion_binary(
    labels: np.ndarray, preds: np.ndarray
) -> Tuple[int, int, int, int]:
    """labels/preds: 0 = clear, nonzero = collision. Returns TN, FP, FN, TP."""
    lc = labels != 0
    pc = preds != 0
    tn = int(np.sum(~lc & ~pc))
    fp = int(np.sum(~lc & pc))
    fn = int(np.sum(lc & ~pc))
    tp = int(np.sum(lc & pc))
    return tn, fp, fn, tp
