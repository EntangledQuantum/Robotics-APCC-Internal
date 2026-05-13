# Kinematic Feasibility Analysis — ABB IRB-1300
## Simulator Design Document

---

## 1. Brief 

A brief feature wise implementation and testing document after having a chat with Sahil on 03/18. 
---

## 2. Pipeline Overview

The simulator follows a phased "solve → verify → parameterize → check" architecture. The input toolpath (CSV) is loaded, optionally transformed from knife frame to base frame, and then each trajectory is processed through the full pipeline.

```
Toolpath CSV
    │  [x, y, z] + [qw, qx, qy, qz] + per-waypoint TCP speed
    │
    ▼  load_toolpath_trajectories_ext()        ← utils/csv_loader_toolpath.py
    │
    │  ToolpathLoadResult { trajectories, speeds, speed_extracted }
    │
    ▼  transform_trajectories_to_base_frame()  ← utils/transform_handler.py
    │  (knife → base frame, skipped if already in base frame)
    │
    ▼  Waypoint Density Check (optional)       ← utils/time_parameterization.py
    │  compute_arc_lengths() → segment gaps
    │  check_waypoint_density() → sparse segments
    │  interpolate_sparse_segments() if sparse:
    │    • position: linear interpolation
    │    • orientation: SLERP (geodesic on S³)
    │
    │  trajectories_t_b_p (n_wp, 7) per trajectory
    │
    ╔═══════════════════ PER-TRAJECTORY LOOP ═══════════════════╗
    ║                                                           ║
    ▼  PHASE 1: IK + C0 (Geometric Path)                       ║
┌───────────────────────────────────────────────────────────┐   ║
│  FeasibilityAnalyzer.analyze_trajectory()                 │   ║
│  ← core/feasibility_checks.py                            │   ║
│                                                           │   ║
│  Per waypoint i:                                          │   ║
│    analyze_waypoint(pos_i, quat_i, q_prev)                │   ║
│      ├─ check_reachability() → IK solve via EAIK          │   ║
│      │    ← core/eaik_ik_solver.py :: EAIKIKSolver.solve()│  ║
│      ├─ _select_best_multi_solution() (if EAIK enabled)   │   ║
│      │    ← score_ik_solution():                          │   ║
│      │       C0 distance + 1/σ_min + manipulability       │   ║
│      ├─ compute_singularity_proximity(J) → σ_min          │   ║
│      │    ← core/checks/singularity.py                    │   ║
│      ├─ compute_manipulability(J, Lc) → Yoshikawa         │   ║
│      │    ← core/checks/manipulability.py                 │   ║
│      ├─ compute_translational_manipulability(J)            │   ║
│      ├─ compute_rotational_manipulability(J)               │   ║
│      ├─ compute_normalized_manipulability(J, Lc)           │   ║
│      └─ compute_directional_manipulability(J, t̂)          │   ║
│                                                           │   ║
│  After all waypoints:                                     │   ║
│    check_c0_continuity(joint_angles_rad)                  │   ║
│      ← core/checks/c0_continuity.py                      │   ║
│      ├─ compute_per_joint_deltas() → |Δqⱼ| per segment   │   ║
│      ├─ detect_config_flips() → flip_indices              │   ║
│      └─ C0Result { passed, max_joint_delta, flip_indices }│   ║
│                                                           │   ║
│  Output: per_waypoint_results, joint_angles_rad (n, 6),   │   ║
│          c0_result, feasibility_flags {reachability_ok,    │   ║
│          c0_ok}                                           │   ║
└──────────────────────────┬────────────────────────────────┘   ║
                           │                                    ║
                           ▼                                    ║
    PHASE 2: TOPP-RA Time Parameterization                      ║
┌───────────────────────────────────────────────────────────┐   ║
│  parameterize_trajectory(joint_angles_rad,                │   ║
│                          vel_limits, accel_limits)         │   ║
│  ← core/topp_check.py                                    │   ║
│                                                           │   ║
│  SplineInterpolator: q(s), s ∈ [0,1]                     │   ║
│  Constraints: q̇_max, q̈_max (hardware limits only)      │   ║
│  Algorithm: TOPPRA time-optimal path parameterization     │   ║
│                                                           │   ║
│  Output: ToppraResult { duration_s, t_samples,            │   ║
│           q(t), q̇(t), q̈(t), s_grid, sd_grid }          │   ║
└──────────────────────────┬────────────────────────────────┘   ║
                           │                                    ║
                           ▼                                    ║
    PHASE 3: Downstream Checks                                  ║
┌───────────────────────────────────────────────────────────┐   ║
│  3a. Task-Space Velocity Verification                     │   ║
│      compute_task_space_velocity(t, q(t), q̇(t), J)      │   ║
│      ← core/checks/task_space_velocity.py                 │   ║
│      V(t) = J(q(t)) · q̇(t) → Speed(t) = ‖v(t)‖        │   ║
│      check_speed_limits(result, speed_limit_m_s)          │   ║
│      → flag violations vs CSV commanded speed             │   ║
│                                                           │   ║
│  3b. C1 Continuity Check                                  │   ║
│      check_c1_continuity(t, q̇(t), q̈(t), limits)        │   ║
│      ← core/checks/c1_continuity.py                      │   ║
│      max|q̇ⱼ(t)| vs q̇ⱼ_max · safety_factor             │   ║
│      max|q̈ⱼ(t)| vs q̈ⱼ_max · safety_factor             │   ║
│      → C1Result { passed, velocity_violations,            │   ║
│                    acceleration_violations }               │   ║
│                                                           │   ║
│  3c. Scoring (optional)                                   │   ║
│      compute_safety_tier() → safety_tier                  │   ║
│      compute_normalized_joint_energy() → smoothness_cost  │   ║
└──────────────────────────┬────────────────────────────────┘   ║
                           │                                    ║
                           ▼                                    ║
    PHASE 4: Graph Generation                                   ║
┌───────────────────────────────────────────────────────────┐   ║
│  Per-trajectory plots (gated by config toggles):          │   ║
│    plot_reachability_per_waypoint()                        │   ║
│    plot_singularity_per_waypoint()                         │   ║
│    plot_manipulability_per_waypoint()                      │   ║
│    plot_c0_continuity_per_waypoint()                       │   ║
│    plot_topp_velocity_profile()                            │   ║
│    plot_task_space_velocity()                              │   ║
│    plot_joint_space_trajectory()                           │   ║
│    plot_3d_spline_trajectory()                             │   ║
│    plot_eaik_solutions_with_scores()                       │   ║
│  ← utils/feasibility_plot.py                              │   ║
└──────────────────────────┬────────────────────────────────┘   ║
                           │                                    ║
                           ▼                                    ║
    PHASE 5: Report                                             ║
┌───────────────────────────────────────────────────────────┐   ║
│  _generate_analysis_report(results, output_path)          │   ║
│  ← feasibility_analysis.py                                │   ║
╚═══════════════════════════════════════════════════════════╝   ║
                                                                ║
    ╚═══════════════════════════════════════════════════════════╝
```

**Feasibility Verdict:**

Without speed checks (Feature 2): `level1_valid = reachability_ok AND c0_ok`

With speed checks (Feature 3): `level1_valid = reachability_ok AND c0_ok AND c1_ok`

Where `c1_ok` evaluates TOPP-RA joint velocities and accelerations against hardware limits, and `check_speed_limits()` separately flags TCP speed violations against the commanded CSV speed.

**Orchestration:** `feasibility_analysis.py` → `process_toolpath()` is the top-level entry point. It calls `FeasibilityAnalyzer.analyze_trajectory()` for Phase 1, `parameterize_trajectory()` for Phase 2, then `compute_task_space_velocity()`, `check_speed_limits()`, and `check_c1_continuity()` for Phase 3. Phase 4 and 5 are graph generation and report writing respectively.

---

## 3. Feature Overview

The three features are ordered by what kinematic constraint each introduces. Each builds on the previous.

| Feature | Adds | Core Question |
|---|---|---|
| 1 — Kinematic Waypoint Feasibility | IK solvability, joint limits, singularity | Is each waypoint kinematically reachable? |
| 2 — Kinematic Trajectory Feasibility | Path interpolation, branch consistency, path-level singularity | Can this path be tracked continuously at any speed? |
| 3 — Kinematic Trajectory w/ Speed | Joint velocity limits at commanded TCP speed | Can this path be tracked at the commanded speed? |

---

## 4. Feature 1 — Kinematic Waypoint Feasibility

Each waypoint is evaluated independently. There is no notion of ordering, interpolation, or speed.

### 4.1 Pipeline

**Step 1 — IK solve via EAIK**

Each 6D pose $$T_i \in SE(3)$$ is passed to the EAIK analytical solver, which returns the full closed-form solution set (up to 16 solutions for a general 6R). An empty solution set means the pose is geometrically unreachable.

**Step 2 — Joint limit filter**

For each solution $$\mathbf{q}^{(k)}$$, check:

$$q_{j,\min} \leq q_j^{(k)} \leq q_{j,\max} \quad \forall j \in \{1,\ldots,6\}$$

Solutions that violate any joint limit are discarded. If no solution survives, the waypoint is infeasible.

**Step 3 — Wrist singularity check**

For each surviving solution, evaluate the wrist singularity condition:

$$|\sin(q_5)| < \sin(0.76^\circ) \approx 0.01327$$

This threshold captures configurations where J4 and J6 become co-axial (J5 ≈ 0 or π), causing the wrist to lose a rotational degree of freedom. This is the primary and default singularity check; broader Jacobian-rank checks are deferred to later analysis phases.

**Outcome per waypoint:** `UNREACHABLE` | `OUT_OF_LIMITS` | `SINGULAR` | `FEASIBLE`

For feasible waypoints, report the best solution (the one with highest $$|\sin(q_5)|$$ among joint-limit-passing solutions).

---

### 4.2 RobotStudio Validation

#### Test 1.A — Reachability Ground Truth

Refer past experiments

**Codebase methods tested:**

| Method | Module | What is validated |
|---|---|---|
| `EAIKIKSolver.solve()` | `core/eaik_ik_solver.py` | IK solvability — whether the EAIK analytical solver returns at least one valid solution for each waypoint pose |
| `EAIKIKSolver._normalize_to_joint_limits()` | `core/eaik_ik_solver.py` | Angle wrapping correctness — ±2π shifts land solutions inside URDF limits |
| `EAIKIKSolver._filter_valid()` | `core/eaik_ik_solver.py` | Joint-limit filtering + FK round-trip verification (1 mm position, 0.02° orientation) |
| `EAIKIKSolver._verify_fk()` | `core/eaik_ik_solver.py` | FK pose reconstruction accuracy vs the original Cartesian target |
| `check_reachability()` | `core/feasibility_checks.py` | Top-level reachability wrapper that calls `solve_with_retries()` |
| `FeasibilityAnalyzer.analyze_waypoint()` | `core/feasibility_checks.py` | Full single-waypoint pipeline: IK → metrics → feasibility result |


#### Test 1.B — Wrist Singularity Ground Truth

Refer past experiments

**Codebase methods tested:**

| Method | Module | What is validated |
|---|---|---|
| `compute_singularity_proximity()` | `core/checks/singularity.py` | σ_min(J) computation — should correlate with RobotStudio's singularity events |
| `SingularityAnalyzer._classify_wrist()` | `core/checks/singularity.py` | J5-specific wrist check: \|sin(q5)\| < sin(0.76°), validated against RobotStudio's `SingArea\Wrist` |
| `SingularityAnalyzer.analyze()` | `core/checks/singularity.py` | Full singularity analysis (unified or classified mode) |
| `FeasibilityAnalyzer.analyze_waypoint()` | `core/feasibility_checks.py` | `near_singularity` flag based on σ_min threshold |
| `compute_condition_number()` | `core/checks/singularity.py` | κ = σ_max/σ_min — condition number should spike at singularity |


#### Test 1.C — Joint Limit Boundary Characterization

**Objective:** Characterize the true software joint limits enforced by RobotStudio/RobotWare for all six joints of the IRB-1300, and quantify any discrepancy relative to the URDF limits. Praneeth's preliminary observation suggests RobotStudio permits motion up to approximately 0.005° beyond the URDF-specified limits, which may cause the simulator to classify certain waypoints as `OUT_OF_LIMITS` while RobotStudio accepts them. This experiment establishes the per-joint tolerance and determines whether the URDF limits need to be updated.

**Metrics evaluated:**
- **RobotWare enforced limits (per joint, per boundary):** $$q_{j,\mathrm{RobotWare}}^{\mathrm{limit}}$$
- **URDF limits (per joint, per boundary):** $$q_{j,\mathrm{URDF}}^{\mathrm{limit}}$$
- **Discrepancy (per joint, per boundary):** $$\Delta q_j = \left|q_{j,\mathrm{RobotWare}}^{\mathrm{limit}} - q_{j,\mathrm{URDF}}^{\mathrm{limit}}\right|$$ (evaluate at both lower and upper limits)

**Why this test is needed:** EAIK's joint limit filter uses the URDF bounds as hard thresholds. If RobotWare's internal limits are even marginally wider, EAIK will discard valid IK solutions that the physical robot can actually reach. Correcting the URDF limits (or introducing a per-joint tolerance in the filter) eliminates this systematic false negative.

**Test setup in RobotStudio:**

- Run six independent experiments — one per joint.
- In each experiment, hold all other joints at a safe, non-singular nominal configuration (e.g., the robot's calibration pose) and command only the joint under test.
- Use `MoveAbsJ` in RAPID to command the joint incrementally toward its URDF-specified limit in steps of 0.001°. Continue incrementing beyond the URDF limit until RobotStudio raises `ERR_OUTSIDE_REACH` or a joint-limit error, or until the motion is silently rejected.
- Repeat the sweep from both sides (approach the upper limit from below; approach the lower limit from above) since ABB's internal buffer is not guaranteed to be symmetric.
- Test both in simulation (RobotStudio virtual controller) and on the physical controller (if available), as the virtual controller may enforce limits slightly differently.

**Parameters to capture:**

- The last commanded joint angle accepted without error, per joint, per boundary (upper and lower) — this is the empirical RobotWare limit $$q_{j,\mathrm{RobotWare}}^{\mathrm{limit}}$$
- The first commanded joint angle that triggers rejection or an error event, per joint, per boundary
- The resulting $$\Delta q_j$$ per joint per boundary, computed against the URDF value

- The specific error code returned at rejection (e.g., `ERR_OUTSIDE_REACH`, `ERR_ROBLIMIT`, or other) — different codes may indicate different enforcement mechanisms
- Whether the behavior differs between the virtual controller and the physical controller, if both are tested

**Codebase methods tested:**

| Method | Module | What is validated |
|---|---|---|
| `EAIKIKSolver._within_joint_limits()` | `core/eaik_ik_solver.py` | Hard joint-limit threshold (URDF bounds ± tolerance) — test determines if this tolerance needs updating |
| `EAIKIKSolver._normalize_to_joint_limits()` | `core/eaik_ik_solver.py` | Whether angle wrapping near limits matches RobotWare's acceptance window |
| `FeasibilityAnalyzer._is_within_joint_limits()` | `core/feasibility_checks.py` | Joint-limit filter in multi-solution scoring — same tolerance question |
| `RobotModel.lower_position_limit` / `upper_position_limit` | `utils/urdf_loader.py` | The URDF-sourced limit values themselves — discrepancy determines if URDF needs correction |

---


## 5. Feature 2 — Kinematic Trajectory Feasibility

A set of ordered waypoints now defines a geometric path. The question is whether a continuous joint-space curve exists that maps onto that path, independent of speed.

### 5.1 Pipeline

**Step 1 — Arc-length parameterization**

The task-space path is parameterized by cumulative arc-length $$s \in [0, L]$$. Between consecutive waypoints $$(T_i, T_{i+1})$$, intermediate poses are generated by geodesic interpolation in $$SE(3)$$: linear interpolation in $$\mathbb{R}^3$$ for position, SLERP for orientation. Parameterizing by arc-length decouples path geometry from speed, which is required to separate Feature 2 from Feature 3 cleanly.

**Step 2 — IK branch tracking and best branch selection**

At each interpolated pose, EAIK returns the full discrete solution set. Because the robot must follow a continuous joint-space path, the branch must be tracked consistently across $$s$$.

For every candidate branch, the solver estimates a weighted composite score over three criteria:

- **C0 continuity:** joint-space distance to the previously selected configuration, $$\|\mathbf{q}^{(k)} - \mathbf{q}^*(s - \delta s)\|_2$$
- **C1:** For Feature 2, this weight is set to 0 and removed from weighting as well.
- **Wrist singularity margin:** $$|\sin(q_5^{(k)})|$$
- **Manipulability:** Yoshikawa index $$w = \sqrt{\det(J(\mathbf{q}^{(k)}) J(\mathbf{q}^{(k)})^\top)}$$ or an equivalent decomposed metric. __This metric is untested, but can be tested to a large extent with data collection for singularity testing. I want to hold on to estimating this for testing purpose. Until then, the weightage for this metric in IK branch selection will be set to 0 (or something like 0.5)__.

The branch maximizing the weighted sum is selected as $$\mathbf{q}^*(s)$$. If no solution satisfies joint limits, the path is infeasible at that arc-length location. The weights are configurable here: https://github.com/darkknight024/Robotics-APCC/blob/continuity/config/batch_feasibility_config.yaml#L50 

**Step 3 — Path-level singularity check**

Apply the wrist singularity condition $$|\sin(q_5^*(s))| < \sin(0.76^\circ)$$ along the selected branch for all $$s$$. A singularity encountered on the interior of the path — not just at isolated waypoints — makes the path infeasible at any finite speed.


**Outcome per path:** `FEASIBLE` | `BRANCH_DISCONTINUITY` (location in arc-length) | `SINGULAR_PATH` (location and joint state)

---

### 5.2 RobotStudio Validation

#### Test 2.A — Continuous Path Joint Angle Ground Truth

**Objective:** Validate that the joint-space trajectory produced by EAIK branch tracking matches the joint trajectory RobotStudio generates for the same Cartesian path. This directly validates C0 continuity and branch selection.

*Metrics evaluated:* Branch tracking correctness, C0 continuity of the selected joint-space path.

**Why this test is needed:** EAIK may select a different IK branch than RobotStudio's internal resolver at points where multiple valid solutions exist. A discrepancy means the simulator is predicting a different physical motion than what the robot would actually execute, which invalidates Feature 3's velocity analysis.

**Test setup in RobotStudio:**
- Define a multi-waypoint Cartesian path using `MoveL` in RAPID with `\ID` synchronization markers.
- In another experiment, setup a trajectory where successive waypoints are atleast 10cm apart, which will test our task space interpolation. 
- Set `ConfL\Off` initially to allow RobotStudio to freely choose configuration, then repeat with `ConfL\On` and explicit `ConfData` to observe branch-constrained motion.
- Log joint angles at high frequency using the RAPID `ReadMotor` function or the PC SDK `MotionSystem.GetJointValues()` during path execution.

**Parameters to capture:**
- Joint angles $$q[1..6]$$ sampled at ≥100 Hz along the path
- Corresponding TCP pose from `CRobT()` at each sample
- Active configuration (confdata) at each waypoint
- Any configuration change events (branch switch) via the Event Log

**Codebase methods tested:**

| Method | Module | What is validated |
|---|---|---|
| `FeasibilityAnalyzer.analyze_trajectory()` | `core/feasibility_checks.py` | Full Phase 1 pipeline — per-waypoint IK with branch tracking via `q_prev` chaining |
| `EAIKIKSolver.solve()` with `q_init` | `core/eaik_ik_solver.py` | IK solve using previous config as seed — `_pick_best()` with `"closest"` strategy |
| `EAIKIKSolver._select_closest()` | `core/eaik_ik_solver.py` | Angle-wrapped L2 distance for branch selection — compare selected branch vs RobotStudio's branch |
| `score_ik_solution()` | `core/feasibility_checks.py` | EAIK multi-solution scoring (C0 + singularity + manipulability) — does the selected branch match RS? |
| `_select_best_multi_solution()` | `core/feasibility_checks.py` | Re-ranking of EAIK candidates with weighted composite score |
| `interpolate_sparse_segments()` | `utils/time_parameterization.py` | Task-space densification (linear + SLERP) — tested by the 10cm-apart experiment |
| `_slerp()` | `utils/time_parameterization.py` | Quaternion SLERP correctness for orientation interpolation |
| `compute_arc_lengths()` | `utils/time_parameterization.py` | Segment distance computation used for density gating |
| `check_c0_continuity()` | `core/checks/c0_continuity.py` | C0 pass/fail — compare against RobotStudio's continuous joint trajectory |

---

#### Test 2.B — Branch Switch Detection

**Objective:** Identify paths where RobotStudio performs an involuntary configuration change (branch switch) mid-path, and verify that our solver flags `BRANCH_DISCONTINUITY` at the same arc-length location.

*Metrics evaluated:* IK branch consistency detection.

**Why this test is needed:** A configuration change mid-path represents a physical discontinuity — the robot must decelerate to zero, reorient, and resume. Our simulator must predict this to correctly flag such paths as requiring re-planning.

**Test setup in RobotStudio:**
- Construct paths known to cross configuration boundaries (e.g., paths that require J1 to cross ±180° or J4 to flip sign). Use systematic sweeps or paths sourced from earlier test cases that exposed branch issues.
- Execute with `ConfL\Off` and monitor for automatic configuration changes via the FlexPendant event log or the PC SDK `MotionSystem` state events.

**Parameters to capture:**
- Arc-length (approximated by TCP position along path) at which configuration change occurs
- Pre- and post-switch joint angles
- Configuration flags before and after switch from `CRobT().robconf`

**Codebase methods tested:**

| Method | Module | What is validated |
|---|---|---|
| `detect_config_flips()` | `core/checks/c0_continuity.py` | Flip detection — should flag the same segment indices where RobotStudio performs a configuration change |
| `check_c0_continuity()` | `core/checks/c0_continuity.py` | `C0Result.flip_indices` and `C0Result.passed` — should agree with RS confdata changes |
| `compute_per_joint_deltas()` | `core/checks/c0_continuity.py` | Per-joint angular delta spikes at branch switch locations |
| `compute_joint_space_distance()` | `utils/math.py` | Euclidean joint-space distance — large spikes indicate branch discontinuity |
| `EAIKIKSolver._select_closest()` | `core/eaik_ik_solver.py` | Branch selection under ambiguity — does our solver avoid involuntary branch switches that RS also avoids? |

---

#### Test 2.C — Path Singularity Detection

**Objective:** Verify that RobotStudio encounters a wrist singularity at the same arc-length location flagged by the simulator's path-level singularity check.

*Metrics evaluated:* Path-level wrist singularity detection.

**Why this test is needed:** A singular point interior to a Cartesian path is not detectable from waypoints alone (Feature 1 would miss it if neither endpoint is singular). This test confirms that the interpolation-level singularity check in Feature 2 catches what RobotStudio also treats as a singularity event.

**Test setup in RobotStudio:**
- Define a `MoveL` path that passes through or near a wrist-singular configuration by construction (e.g., a straight-line path whose midpoint requires J5 ≈ 0).
- Execute twice: once with `SingArea\Wrist`, once without. Without `SingArea\Wrist`, RobotStudio should interrupt or modify the path at the singularity.

**Parameters to capture:**
- TCP position and joint angles at the point of singularity interrupt or motion modification
- $$q_5$$ value at that point
- Whether `SingArea\Wrist` altered the orientation (confirmation of wrist singularity handling)
- Event log timestamp and joint state for the singularity event

**Codebase methods tested:**

| Method | Module | What is validated |
|---|---|---|
| `compute_singularity_proximity()` | `core/checks/singularity.py` | σ_min(J) along the path — should drop to near-zero at the same arc-length location as RS singularity event |
| `SingularityAnalyzer._classify_wrist()` | `core/checks/singularity.py` | \|sin(q5)\| < sin(0.76°) — should flag the interior singular point that RS detects |
| `FeasibilityAnalyzer.analyze_trajectory()` | `core/feasibility_checks.py` | `near_singularity` flag on interpolated waypoints — validates that densification + per-waypoint singularity catches interior singularities |
| `interpolate_sparse_segments()` | `utils/time_parameterization.py` | Densification must generate intermediate waypoints through the singular zone so the per-waypoint check can detect it |
| `compute_condition_number()` | `core/checks/singularity.py` | κ(q) spike at the singular location should correlate with RS singularity interrupt |

---

## 6. Feature 3 — Kinematic Trajectory Feasibility with Speed

Feature 2's continuous joint-space path is now time-parameterized at the commanded TCP speed, and the resulting joint velocities are checked against hardware limits.

### 6.1 Pipeline

**Step 1 — Time parameterization at constant TCP speed**

Given commanded TCP speed $$v_{\text{cmd}}$$ (mm/s) and the arc-length parameterization from Feature 2:

$$\dot{s} = v_{\text{cmd}} \quad \Rightarrow \quad t(s) = \frac{s}{v_{\text{cmd}}}$$

This is consistent with ABB's `MoveL` semantics, where `v` specifies constant TCP speed along a Cartesian path.

**Step 2 — Joint velocity via Jacobian inversion**

The instantaneous joint velocity at arc-length $$s$$ is:

$$\dot{\mathbf{q}}(s) = J^{-1}(\mathbf{q}^*(s)) \cdot \dot{\mathbf{x}}(s)$$

where $$\dot{\mathbf{x}}(s) \in \mathbb{R}^6$$ is the task-space velocity vector (linear component: $$v_{\text{cmd}} \hat{\mathbf{t}}(s)$$, angular component: $$\boldsymbol{\omega}(s)$$ from orientation rate along the path), and $$J^{-1}$$ is the exact inverse of the $$6 \times 6$$ body Jacobian (valid since singular configurations have already been excluded). Near-singular configurations will cause $$\|J^{-1}\|$$ to grow, amplifying joint velocities even at moderate TCP speeds — this is the primary failure mode.

**Step 3 — Joint velocity limit check**

$$\max_j |\dot{q}_j(s)| \leq \dot{q}_{j,\max} \quad \forall s \in [0, L]$$

Any violation is reported with its arc-length location, the violating joint, and the margin of violation. The maximum feasible TCP speed at each point can also be computed:

$$v_{\max}(s) = \min_j \frac{\dot{q}_{j,\max}}{|[J^{-1}(s)\,\hat{\mathbf{x}}(s)]_j|}$$

This profile is useful for diagnosis and directly feeds into TOPP-RA if optimal speed scheduling is required.

**Outcome per trajectory:** `FEASIBLE` | `VELOCITY_LIMIT_EXCEEDED` (arc-length location, joint index, violation magnitude)

---

### 6.2 RobotStudio Validation

#### Test 3.A — Joint Velocity Profile Ground Truth

**Objective:** Compare the joint velocity profile predicted by the simulator's Jacobian inversion against the actual joint velocities recorded by RobotStudio during a `MoveL` execution at a specified TCP speed.

*Metrics evaluated:* Joint velocity estimation accuracy, Jacobian correctness, time parameterization fidelity.

**Why this test is needed:** The Jacobian used in simulation is derived from the URDF kinematic model. Any mismatch with ABB's internal model will produce incorrect velocity predictions. This test quantifies the error and identifies whether discrepancies are systematic (frame offset error) or path-dependent (local Jacobian error).

**Test setup in RobotStudio:**
- Define a Cartesian path executed with `MoveL` at multiple TCP speeds (e.g., 50, 200, 500, 1000 mm/s).
- Use the PC SDK `MotionSystem.GetJointValues()` at high sample rate (≥ 250 Hz) or log via the `EGM` interface for real-time joint data.
- Alternatively, use the `RAPID` instruction `ReadMotor` in a background task with a tight loop timer.

**Parameters to capture:**
- Joint angles $$q[1..6]$$ and joint velocities $$\dot{q}[1..6]$$ at each sample
- TCP speed setpoint and actual TCP speed from `CRobT()` differentiated over time
- Timestamp for each sample (wall clock + RAPID clock) for synchronization
- Path arc-length reconstructed from TCP position samples

**Codebase methods tested:**

| Method | Module | What is validated |
|---|---|---|
| `parameterize_trajectory()` | `core/topp_check.py` | TOPP-RA time parameterization — `ToppraResult.qdot_t` compared to RobotStudio's logged q̇ |
| `compute_task_space_velocity()` | `core/checks/task_space_velocity.py` | V(t) = J(q(t))·q̇(t) — predicted TCP speed profile compared to RS actual TCP speed |
| `PinocchioFKSolver.get_jacobian()` | `core/pin_fk_solver.py` | Analytical Jacobian correctness — systematic errors indicate frame/DH mismatch |
| `EAIKFKSolver.get_jacobian()` | `core/eaik_fk_solver.py` | Numerical (finite-difference) Jacobian — compare against Pinocchio's analytical Jacobian and RS velocities |
| `check_c1_continuity()` | `core/checks/c1_continuity.py` | `C1Result.max_joint_velocities_rad_s` compared to RS peak joint velocities |

---

#### Test 3.B — Velocity Limit Violation Ground Truth

**Objective:** Verify that paths and speeds that the simulator flags as `VELOCITY_LIMIT_EXCEEDED` are either rejected by RobotStudio, executed at a reduced speed, or trigger a joint speed warning.

*Metrics evaluated:* Joint velocity limit detection, near-singularity velocity amplification.

**Why this test is needed:** The critical failure mode is a path that passes near a wrist singularity at a non-zero speed, causing joint velocity to spike. RobotStudio handles this by automatically reducing TCP speed. The simulator must predict where this happens and agree with RobotStudio on the severity. A false negative from the simulator (predicting feasible when it is not) is a safety-relevant error.

**Test setup in RobotStudio:**
- Construct paths that approach the wrist singularity zone at varying TCP speeds; increase speed incrementally until RobotStudio either reduces speed autonomously or triggers a warning.
- Use `MoveL` with `SpeedRefine` disabled where possible to prevent automatic speed reduction from masking the ground truth.
- Log the joint velocity at the near-singular point and the TCP speed at which RobotStudio first intervenes.

**Parameters to capture:**
- TCP speed at which RobotStudio reduces speed or raises an event, per path
- $$q_5$$ value and full joint state at the near-singular location
- Joint velocities $$\dot{q}[1..6]$$ at the point of maximum joint speed
- RobotWare event log entries for speed reduction or singularity proximity

**Codebase methods tested:**

| Method | Module | What is validated |
|---|---|---|
| `check_c1_continuity()` | `core/checks/c1_continuity.py` | `C1Result.velocity_violations` — should flag the same joints/locations where RS reduces speed |
| `check_speed_limits()` | `core/checks/task_space_velocity.py` | TCP speed violation detection — should agree with RS speed reduction events |
| `compute_task_space_velocity()` | `core/checks/task_space_velocity.py` | TCP speed spike near singularity — compare magnitude and location to RS observed speed reduction |
| `compute_singularity_proximity()` | `core/checks/singularity.py` | σ_min at the near-singular point — should correlate with the TCP speed at which RS first intervenes |
| `SingularityAnalyzer._classify_wrist()` | `core/checks/singularity.py` | \|sin(q5)\| at the near-singular location — validate against RS event q5 value |
| `parameterize_trajectory()` | `core/topp_check.py` | TOPP-RA feasibility — `sd_grid` should show the speed profile narrowing at the near-singular point |

---

#### Test 3.C — Speed Profile vs. TOPP-RA Bound

**Objective:** Validate the $$v_{\max}(s)$$ speed profile computed by the simulator against the maximum speed RobotStudio actually sustains along the path. This confirms that the simulator's Jacobian-based speed bound is tight and not overly conservative.

*Metrics evaluated:* Maximum feasible speed profile, Jacobian-based speed limit tightness.

**Why this test is needed:** If the simulator's $$v_{\max}(s)$$ is significantly more conservative than RobotStudio's actual capability, the feasibility check will produce false negatives and constrain trajectory planning unnecessarily. This test calibrates the threshold.

**Test setup in RobotStudio:**
- Execute the same path at a sweep of TCP speeds from low to high.
- For each speed, record whether the path completes at the commanded speed or whether RobotStudio reduces it.
- The speed at which RobotStudio first reduces the TCP rate defines the empirical $$v_{\max}(s)$$ bound at the most constrained point.

**Parameters to capture:**
- Commanded vs. achieved TCP speed at each path point
- Joint velocities at the most-constrained arc-length location
- ABB-reported maximum speed for the path (from `SpeedData` feedback if available)
- $$q_5$$ and full joint state at the speed-limiting location

**Codebase methods tested:**

| Method | Module | What is validated |
|---|---|---|
| `parameterize_trajectory()` | `core/topp_check.py` | TOPP-RA time-optimal speed profile — `sd_grid` represents the maximum feasible path velocity at each gridpoint; compare against RS empirical v_max |
| `compute_task_space_velocity()` | `core/checks/task_space_velocity.py` | Predicted `linear_speed` profile — should bound the achievable TCP speed from above; compare tightness against RS speed sweep |
| `check_speed_limits()` | `core/checks/task_space_velocity.py` | Threshold detection — the TCP speed at which violations first appear should match the RS speed at which intervention begins |
| `check_c1_continuity()` | `core/checks/c1_continuity.py` | Joint velocity/acceleration margins — the most-constrained joint at the speed-limiting location |
| `PinocchioFKSolver.get_jacobian()` | `core/pin_fk_solver.py` | Jacobian accuracy at the speed-limiting point — any systematic error makes the speed bound too loose or too tight |
| `compute_singularity_proximity()` | `core/checks/singularity.py` | σ_min at the speed-limiting location — calibrates how close to singularity the speed constraint becomes active |

---

## 7. Summary

| Feature | Key Check | Simulator Output | RobotStudio Ground Truth |
|---|---|---|---|
| 1 — Waypoint | IK solvability, joint limits | $$\sin q_5 < \sin(0.76^\circ)$$ | `FEASIBLE` / `UNREACHABLE` / `SINGULAR` | Reachability, joint state, singularity event |
| 2 — Path | Branch consistency, path singularity, C0/C1 continuity | `FEASIBLE` / `BRANCH_DISCONTINUITY` / `SINGULAR_PATH` | Joint trajectory, confdata changes, singularity interrupts |
| 3 — Speed | Joint velocity vs. limits at $$v_{\text{cmd}}$$; $$v_{\max}(s)$$ profile | `FEASIBLE` / `VELOCITY_LIMIT_EXCEEDED` | Joint velocity logs, speed reduction events, $$v_{\max}$$ empirical bound |