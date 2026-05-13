# Feature 4 — Collision Checking Demo

Mesh-vs-mesh trajectory collision checking for the IRB-1300 cell.  
Reads a RobotStudio logger CSV, runs Pinocchio + hpp-fcl over the joint
trajectory, and reports per-waypoint collisions, the first violating waypoint,
the offending obstacle, and (optionally) whether any EAIK IK branch could have
avoided it.

For design and dataset context, see `FEATURE_4_HANDOFF.md`.

---

## What the demo proves

| Input CSV | Expected output |
|-----------|-----------------|
| No-collision trajectory (`is_collision = 0` throughout) | `Collision-free: True` and **VERDICT: TRAJECTORY FEASIBLE** |
| Colliding trajectory (`is_collision = 1/2/3`) | First violating waypoint + colliding pair (`Link_X ↔ Obstacle_Y`) + EAIK branch summary + **VERDICT: TRAJECTORY INFEASIBLE** |

The agreement against RS labels is printed as `TP / FP / FN / TN` (binary).

---

## 1. Install (one-time)

Requires [conda / Miniforge](https://conda-forge.org/miniforge/) so Pinocchio
and hpp-fcl come as prebuilt binaries (no C++ compiler needed).

```powershell
conda env create -f environment.yml
conda activate apcc
```

Verify:

```powershell
python -c "import pinocchio, hppfcl; print('pinocchio', pinocchio.__version__)"
```

---

## 2. Configure the scene

Defaults in `config/collision_config.yaml` already match the validated
handoff scene (3 obstacles, frame alignment confirmed zero-offset). Adjust
`urdf_path`, `mode`, or `whitelist_pairs` there.

Key fields:

- `mode`: `early_termination` (stop at first hit) or `full_sweep` (every waypoint).
- `whitelist_pairs`: e.g. `[["midsole_link", "knife_blade"]]`. Midsole is **optional** —
  if `midsole_link` is not in the model, the entry is ignored and every robot↔knife
  contact stays flagged (logged as a warning).

---

## 3. Run

Single CSV:

```powershell
python scripts\feature4_demo.py --csv "C:\path\to\traj1_obstacle1.csv"
```

Folder of CSVs + EAIK all-branches diagnostic:

```powershell
python scripts\feature4_demo.py --csv-dir "C:\path\to\Phase_2B_Feature4" --mode full_sweep --eaik
```

Useful flags:

- `--mode early_termination | full_sweep` — overrides the YAML setting.
- `--eaik` — also runs EAIK on TCP per colliding waypoint and tests every analytical branch.
- `--eaik-full-traj` — Phase B on **all** rows, not just label-collision rows.
- `--config <path>` — point at a different YAML.

---

## 4. What the output means

Example INFEASIBLE run (excerpt):

```
--- Phase A : joint-space (mode=full_sweep) ---
Waypoints in trajectory : 10
Collision-free          : False
First collision         : waypoint 5, pair=('Link_4', 'Obstacle_1'), code=1 (Obstacle_1 (Box A))
Binary agreement (eval=10): TP=4 FP=0 FN=0 TN=6

--- Phase B : EAIK all-branches diagnostic ---
  WP    5  label=1  ALL BRANCHES COLLIDE  e.g. pair=Link_4↔Obstacle_1
  WP    6  label=1  ALL BRANCHES COLLIDE  e.g. pair=Link_4↔Obstacle_1
  WP    7  label=1  ALL BRANCHES COLLIDE  e.g. pair=Link_4↔Obstacle_1

VERDICT: TRAJECTORY INFEASIBLE — first hit at waypoint 5 with Obstacle_1 (Box A).
```

Example FEASIBLE run:

```
--- Phase A : joint-space (mode=full_sweep) ---
Collision-free          : True
Global min clearance    : 0.123 m
Binary agreement (eval=10): TP=0 FP=0 FN=0 TN=10
VERDICT: TRAJECTORY FEASIBLE — no collision detected by mesh-vs-mesh check.
```

---

## 5. Files in this delivery

| Path | Purpose |
|------|---------|
| `config/collision_config.yaml` | Scene + policy config |
| `core/scene_builder.py` | Builds URDF + obstacle geometry |
| `core/collision_group_manager.py` | Pair policy + whitelist (midsole optional) |
| `core/trajectory_collision_checker.py` | Trajectory check (early / full-sweep), branch helpers |
| `core/collision_checker.py` | Existing self-collision kernel (reused) |
| `core/collision_report.py` | Result dataclasses |
| `scripts/feature4_demo.py` | The demo entry point |
| `scripts/evaluate_collision_csv.py` | Bulk RS-CSV evaluation |
| `environment.yml` | One-shot conda install |
| `FEATURE_4_HANDOFF.md` | Truth source (frames, dataset, obstacles) |

---

## 6. Troubleshooting

- **`Python was not found ...`** — open a fresh terminal after installing
  Miniforge, or run `conda activate apcc` first.
- **`ImportError: No module named 'pinocchio'`** — you are in the wrong env;
  `conda activate apcc` and retry.
- **Whitelist did nothing** — geometry names must match exactly
  (`midsole_link`, `knife_blade`). The script logs a warning when a
  whitelisted name is absent from the loaded model.
- **No collisions reported on a known-collision CSV** — confirm URDF path in
  `collision_config.yaml`; the URDF in the handoff is the *non*-fixture
  variant (`IRB_1300_1400_URDF.urdf`).
