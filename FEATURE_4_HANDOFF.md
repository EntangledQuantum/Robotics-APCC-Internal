# Feature 4 — Collision Checking: Full Handoff to Cursor

This document captures everything established in the design and validation session before implementation begins.
It is the single source of truth for building Feature 4. Read it fully before touching any file.

---

## 1. What this repo does (context)

- **Purpose:** Kinematic feasibility analysis for ABB IRB-1300-class robots — toolpaths, IK, singularity, manipulability, continuity, scoring.
- **IK solver:** EAIK (primary). Pinocchio used for FK/Jacobian metrics.
- **Collision today:** `core/collision_checker.py` — Pinocchio + hpp-fcl, **self-collision only**, not wired into `feasibility_analysis.py`.
- **Feature 4 goal:** Extend collision checking to the full scene — robot body vs static obstacles — and integrate it into the main pipeline after IK.

---

## 2. Codebase map (relevant files)

| File | Role |
|------|------|
| `core/collision_checker.py` | Current `SelfCollisionChecker` — URDF load, adjacency filter, calibration, `has_self_collision(q)`, `check(q)` |
| `core/feasibility_checks.py` | `FeasibilityAnalyzer` — per-waypoint IK, Jacobian metrics, trajectory stats |
| `core/__init__.py` | `create_solvers()`, exports |
| `feasibility_analysis.py` | Main pipeline — **collision stage missing here** |
| `tests/test_reachability.py` | Optional self-collision gate after IK (`--check_self_collision` flag) |
| `utils/config_loader.py` | YAML config loader |
| `utils/urdf_loader.py` | URDF resolution (fuzzy match) |

---

## 3. URDF structure (IRB-1300 1400)

Extracted from the URDF CSV. Joint chain in order:

| Joint | Parent → Child | Origin (m) | Axis | Limits (rad) |
|-------|---------------|------------|------|--------------|
| Joint_1 | Base → Link_1 | (0, 0, 0.254) | Z | −3.1416 to 3.1416 |
| Joint_2 | Link_1 → Link_2 | (0.15, 0, 0.29) | Y | −1.6581 to 2.7053 |
| Joint_3 | Link_2 → Link_3 | (0, 0, 0.575) | Y | −3.6652 to 1.2043 |
| Joint_4 | Link_3 → Link_4 | (0.1295, 0, 0.04) | X | −4.0143 to 4.0143 |
| Joint_5 | Link_4 → Link_5 | (0.5455, 0, 0) | Y | −2.2689 to 2.2689 |
| Joint_6 | Link_5 → Link_7 | (0.092, 0, 0) + pitch=π/2 | Z | −6.9813 to 6.9813 |

**First link name:** `Base` (not `world`, not `base_link`).
All joint angles are in **radians** when passed to Pinocchio. CSV data is in **degrees** — always `np.deg2rad()` before use.

---

## 4. Frame alignment — VERIFIED, ZERO OFFSET

**This is the most important finding of the session.**

A forward kinematics check was run using the first waypoint of `collision_traj_1.csv`:

- Joint angles: `[-0.0, 11.313, 41.7946, 0.0, 36.8925, 0.0]` degrees
- Pinocchio FK result: `X=700.0, Y=0.0, Z=500.0 mm`
- RobotStudio reported TCP (`rs_x_mm`, `rs_y_mm`, `rs_z_mm`): `700.0, 0.0, 500.0 mm`
- **Euclidean error: 0.00 mm**

**Conclusion:** RobotStudio world frame = URDF base frame. They share the same origin and axes.
**No transform is needed.** Obstacle poses from RobotStudio drop directly into Pinocchio after mm → m conversion.

---

## 5. Obstacle definitions — CONFIRMED

Source: `obstacle_poses.csv` and `ObstacleCollision_Logger.modx`.

All positions are in the **robot base frame**. Convert mm → m (divide by 1000) when loading into Pinocchio.

### Obstacle_1 — Box A
- **Type:** Box
- **Pose reference:** `corner_min_xyz` — position marks the minimum-X, minimum-Y, minimum-Z corner
- **Position:** (500, 0, 0) mm → **(0.500, 0.0, 0.0) m**
- **Dimensions:** L=150mm (X), W=100mm (Y), H=200mm (Z)
- **Full boundary:** X: 500–650 mm, Y: 0–100 mm, Z: 0–200 mm

### Obstacle_2 — Box B
- **Type:** Box
- **Pose reference:** `corner_min_xyz`
- **Position:** (0, 500, 0) mm → **(0.0, 0.500, 0.0) m**
- **Dimensions:** L=200mm (X), W=150mm (Y), H=150mm (Z)
- **Full boundary:** X: 0–200 mm, Y: 500–650 mm, Z: 0–150 mm

### Obstacle_3 — Cylinder C
- **Type:** Cylinder
- **Pose reference:** `center_base_xy` — position marks the centre of the base circle at Z=0
- **Position:** (250, −400, 0) mm → **(0.250, −0.400, 0.0) m**
- **Dimensions:** R=150mm, H=150mm
- **Full boundary:** sqrt((x−250)² + (y+400)²) ≤ 150 mm, Z: 0–150 mm

---

## 6. Validation dataset — CONFIRMED BODY COLLISION DATA

### Two separate sets of CSVs

**Set A — `collision_traj_1.csv` to `collision_traj_5.csv`** (from `CollisionTestModule_Logger_final.mod`)
- TCP-point collision detection in RAPID (`CheckCollision()` function)
- `is_collision` codes: 0=none, 1=Obstacle_1, 2=Obstacle_2, 3=Obstacle_3
- These are TCP-in-bounding-volume checks — useful but less precise

**Set B — `traj1_obstacle1.csv` to `traj5_obstacle3.csv`** (from `ObstacleCollision_Logger.modx`)
- **True mesh/body collision data** — verified manually via RobotStudio `CollisionSet_1` 3D view
- `is_collision` set manually before known collision moves
- **All collision rows have TCP completely outside obstacle boundaries** — confirmed by checking `rs_x_mm/y/z` against obstacle bounds
- These are the **primary ground truth dataset** for validating Feature 4

### Set B — what collides with what

| File | Colliding links | Obstacle | is_collision code |
|------|----------------|----------|-------------------|
| `traj1_obstacle1.csv` | Link_4 | Obstacle_1 | 1 |
| `traj2_obstacle2.csv` | Link_4 | Obstacle_2 | 2 |
| `traj3_obstacle3.csv` | Link_4, Link_5 | Obstacle_3 | 3 |
| `traj4_obstacle2.csv` | Link_5 | Obstacle_2 | 2 |
| `traj5_obstacle3.csv` | Link_4, Link_5 | Obstacle_3 | 3 |

### CSV schema (22 columns, both sets)

```
time_ms, rs_j1_deg, rs_j2_deg, rs_j3_deg, rs_j4_deg, rs_j5_deg, rs_j6_deg,
speed_mm_per_s, cf1, cf4, cf6, cfx,
rs_x_mm, rs_y_mm, rs_z_mm, rs_qw, rs_qx, rs_qy, rs_qz,
linear_acceleration_mm_s_2, is_at_waypoint, is_collision
```

**Checker input columns:** `rs_j1_deg` … `rs_j6_deg` → convert to radians → `q` vector for Pinocchio.
**Label column:** `is_collision` (last column).
**Optional filter:** `is_at_waypoint == 1` to evaluate only true waypoints (not interpolated motion rows).

### is_collision codes (both sets)

| Code | Meaning |
|------|---------|
| 0 | No collision |
| 1 | Any robot link colliding with Obstacle_1 |
| 2 | Any robot link colliding with Obstacle_2 |
| 3 | Any robot link colliding with Obstacle_3 |
| 4 | Self-collision (robot link vs robot link) — self-collision CSVs only |

---

## 7. Critical distinction: TCP-point vs mesh-vs-mesh

**TCP-point checking (what Set A CSVs use):**
- Checks only if the tool centre point (end-effector tip) falls inside an obstacle bounding volume
- Fast but misses all arm/body collisions
- Implemented in RAPID as a simple coordinate boundary check

**Mesh-vs-mesh checking (what Pinocchio + hpp-fcl does, and what Set B requires):**
- Checks every robot link STL mesh against every obstacle STL mesh
- Catches elbow, forearm, upper arm collisions — anything touching anything
- Set B TCP positions are **entirely outside** obstacle bounds at collision rows — TCP checking would return 0 (no collision) for all of them
- Pinocchio mesh-vs-mesh is the correct and necessary approach for this dataset

**Implication for evaluation:**
- Set A (TCP): Pinocchio may fire slightly *before* the label flips — that is correct and expected
- Set B (body): Pinocchio must catch the collision; TCP checking would completely miss it
- **Priority: zero false negatives on Set B collision rows**
- False positives (Pinocchio catches something RS missed) are acceptable — stricter is better

---

## 8. Feature 4 — target pipeline

Current order (no collision):
```
Toolpath CSV → IK (EAIK) → Metrics/scoring → Plots
```

Feature 4 target order:
```
Toolpath CSV → IK (EAIK) → Collision check (full scene) → Speed/continuity → Metrics/scoring → Plots
```

Collision stage sits **immediately after IK produces `q`**, before any downstream metrics.
EAIK produces `q` in radians. Pinocchio consumes `q` in radians on the same URDF — joint order and `nq` must match.

---

## 9. Target module architecture

| Module | File (suggested) | Responsibility |
|--------|-----------------|----------------|
| `SceneBuilder` | `core/scene_builder.py` | Load URDF + register static obstacle STLs at their base-frame poses → single `GeometryModel` |
| `CollisionGroupManager` | `core/collision_group_manager.py` | Define ROBOT / ENVIRONMENT groups; pair rules; whitelist; adjacency filter on self-pairs only |
| `TrajectoryCollisionChecker` | `core/trajectory_collision_checker.py` | Accept `q_traj` (N×6); early-stop or full sweep; return distances + violations per waypoint |
| `CollisionReport` | `core/collision_report.py` | Dataclasses: first collision index, offending pair, min clearance, per-waypoint summary |
| Config | `collision_config.yaml` | URDF path, obstacle STL paths, obstacle poses, whitelist pairs, safety margins |

### Pair policy (what to check)

| Pair type | Check? | Notes |
|-----------|--------|-------|
| ROBOT link ↔ ENVIRONMENT obstacle | ✅ Yes | Core of Feature 4 |
| ROBOT link ↔ ROBOT link (self) | ✅ Yes | Reuse existing adjacency filter + calibration |
| ENVIRONMENT ↔ ENVIRONMENT | ❌ No | Static objects never move |
| midsole ↔ knife blade | ❌ Whitelist | Intended process contact |

---

## 10. Implementation order (do this exactly)

1. **`collision_config.yaml`** — URDF path, 3 obstacle STL paths + poses (in metres), whitelist entry. No code yet.
2. **`SceneBuilder`** — load URDF + register obstacles into one `GeometryModel`. Test by printing all geometry objects and confirming count.
3. **`CollisionGroupManager`** — implement pair policy above. Test by printing active pairs.
4. **CSV evaluation script** (`scripts/evaluate_collision_csv.py`) — reads any of the 10 CSVs, converts deg→rad, runs checker per row, compares to `is_collision`, prints confusion matrix.
5. **Validate against Set B first** (body collision CSVs) — these are the cleaner ground truth.
6. **Then validate against Set A** (TCP collision CSVs) — expect Pinocchio to fire slightly earlier.
7. **Integrate into `feasibility_analysis.py`** only after steps 1–6 pass.

---

## 11. Risks and known limitations

| Risk | Status | Mitigation |
|------|--------|------------|
| Discrete waypoint sampling | Accepted for v1 | Densify path if needed |
| STL mesh quality (joint overlap) | Handled | Calibration removes persistent self-collision false positives |
| Whitelist requires separate blade mesh | Must verify | Knife must be a separate STL from housing |
| Set A labels lag real collision | Understood | Pinocchio firing before label flip is correct behaviour |
| Set B has only 10 rows per file | Accepted | Sufficient for structural validation; not a statistical benchmark |

---

## 12. What NOT to do

- Do not use TCP-point-in-bounding-box logic in Pinocchio — that is what RAPID did and it misses body collisions entirely.
- Do not apply any frame transform between RobotStudio coordinates and URDF coordinates — alignment is confirmed zero-offset.
- Do not integrate into `feasibility_analysis.py` before the CSV evaluation script passes on Set B.
- Do not check ENVIRONMENT ↔ ENVIRONMENT pairs — static objects never collide with each other.
- Do not forget mm → m conversion when loading obstacle poses into Pinocchio.

---

## 13. Related files in repo

| File | Content |
|------|---------|
| `FEATURE_4_COLLISION_CHECKING_PLAN.md` | Original full Feature 4 dev plan |
| `FEATURE_4_AND_CODEBASE_NOTES.md` | Consolidated codebase + Feature 4 notes (pre-session) |
| `FEATURE_WISE_DOC.md` | Broader simulator/pipeline overview |
| `core/collision_checker.py` | Current self-collision implementation (reference for SceneBuilder pattern) |
| `tests/test_reachability.py` | Optional self-collision gate — reference for CSV evaluation script pattern |
| `config/collision_config.yaml` | URDF path, primitive obstacles (handoff §5), calibration / whitelist |
| `utils/collision_config_loader.py` | Loads `collision_config.yaml` |
| `core/scene_builder.py` | `build_scene_from_config` — URDF + obstacle primitives (or STL) |
| `core/collision_group_manager.py` | `rebuild_collision_pairs` — pair policy |
| `core/trajectory_collision_checker.py` | `TrajectoryCollisionChecker`, `build_checker_from_collision_config` |
| `core/collision_report.py` | CSV evaluation summary dataclasses |
| `scripts/evaluate_collision_csv.py` | CLI: RS logger CSV vs checker (confusion matrix) |

---

*Generated after full design review and frame alignment / dataset validation session.
All findings (FK check, TCP-vs-body collision confirmation, obstacle poses) are empirically verified — not assumed.*
