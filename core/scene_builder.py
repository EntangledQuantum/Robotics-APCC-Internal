#!/usr/bin/env python3
"""
SceneBuilder — URDF robot collision geometry + static obstacles (Feature 4).

Obstacles default to **hpp-fcl primitives** (box / cylinder) with poses and
dimensions from ``FEATURE_4_HANDOFF.md``. Optional STL obstacles can be added
via config (``mesh_path``) when ``use_primitives: false`` per item.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np
import pinocchio as pin

try:
    import hppfcl as fcl
except ImportError:  # pragma: no cover
    fcl = None  # type: ignore


@dataclass
class SceneContext:
    """Pinocchio model + geometry model after URDF load and obstacle registration."""

    model: pin.Model
    data: pin.Data
    geom_model: pin.GeometryModel
    geom_data: pin.GeometryData
    env_geom_indices: Set[int]
    env_name_to_code: Dict[str, int]
    urdf_abs: str


def _resolve_urdf_path(urdf_path: str) -> str:
    """Same resolution strategy as ``SelfCollisionChecker._resolve_path``."""
    p = Path(urdf_path)
    if p.is_absolute() and p.exists():
        return str(p)
    cwd_path = (Path.cwd() / p).resolve()
    if cwd_path.exists():
        return str(cwd_path)
    try:
        from utils.urdf_loader import resolve_urdf_path

        return str(resolve_urdf_path(urdf_path))
    except Exception:
        pass
    raise FileNotFoundError(
        f"URDF not found: {urdf_path}\n"
        f"Tried absolute, relative to CWD ({Path.cwd()}), and fuzzy URDF search."
    )


def _se3_from_xyz_rpy(xyz: np.ndarray, rpy: Optional[np.ndarray] = None) -> pin.SE3:
    if rpy is None:
        rpy = np.zeros(3)
    R = pin.rpy.rpyToMatrix(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    return pin.SE3(R, xyz.astype(float))


def _make_geometry_object(
    name: str,
    parent_joint: int,
    geometry,
    placement: pin.SE3,
) -> pin.GeometryObject:
    """Pinocchio bindings differ: (geom, placement) vs (placement, geom)."""
    try:
        return pin.GeometryObject(name, parent_joint, geometry, placement)
    except TypeError:
        return pin.GeometryObject(name, parent_joint, placement, geometry)


def _append_box(
    geom_model: pin.GeometryModel,
    name: str,
    half_extents_m: np.ndarray,
    center_m: np.ndarray,
    rpy: Optional[np.ndarray] = None,
) -> int:
    if fcl is None:
        raise ImportError("hppfcl is required for SceneBuilder")
    box = fcl.Box(float(half_extents_m[0]), float(half_extents_m[1]), float(half_extents_m[2]))
    placement = _se3_from_xyz_rpy(center_m, rpy)
    go = _make_geometry_object(name, 0, box, placement)
    geom_model.addGeometryObject(go)
    return len(geom_model.geometryObjects) - 1


def _append_cylinder(
    geom_model: pin.GeometryModel,
    name: str,
    radius_m: float,
    half_length_m: float,
    center_m: np.ndarray,
    rpy: Optional[np.ndarray] = None,
) -> int:
    if fcl is None:
        raise ImportError("hppfcl is required for SceneBuilder")
    cyl = fcl.Cylinder(float(radius_m), float(half_length_m))
    placement = _se3_from_xyz_rpy(center_m, rpy)
    go = _make_geometry_object(name, 0, cyl, placement)
    geom_model.addGeometryObject(go)
    return len(geom_model.geometryObjects) - 1


def _append_mesh(
    geom_model: pin.GeometryModel,
    name: str,
    mesh_path: str,
    center_m: np.ndarray,
    rpy: Optional[np.ndarray] = None,
    scale: Optional[np.ndarray] = None,
) -> int:
    if fcl is None:
        raise ImportError("hppfcl is required for SceneBuilder")
    path = Path(mesh_path)
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()
    if not path.exists():
        raise FileNotFoundError(f"Obstacle mesh not found: {mesh_path}")
    loader = fcl.MeshLoader()
    mesh = loader.load(str(path))
    if scale is not None:
        mesh.scale = np.asarray(scale, dtype=np.float64)
    placement = _se3_from_xyz_rpy(center_m, rpy)
    go = _make_geometry_object(name, 0, mesh, placement)
    geom_model.addGeometryObject(go)
    return len(geom_model.geometryObjects) - 1


def _build_obstacle_geom(
    geom_model: pin.GeometryModel,
    item: Dict[str, Any],
    use_primitives: bool,
) -> int:
    name = item["name"]
    kind = str(item.get("kind", "box")).lower()

    if not use_primitives and item.get("mesh_path"):
        xyz = np.asarray(item.get("center_xyz_m", [0, 0, 0]), dtype=float)
        rpy = item.get("rpy_rad")
        rpy_arr = np.asarray(rpy, dtype=float) if rpy is not None else None
        scale = item.get("scale")
        sc = np.asarray(scale, dtype=float) if scale is not None else None
        return _append_mesh(geom_model, name, str(item["mesh_path"]), xyz, rpy_arr, sc)

    if kind == "box":
        corner = np.asarray(item["corner_min_xyz_mm"], dtype=float) / 1000.0
        size = np.asarray(item["size_xyz_mm"], dtype=float) / 1000.0
        half = size * 0.5
        center = corner + half
        return _append_box(geom_model, name, half, center, None)

    if kind == "cylinder":
        xy = np.asarray(item["center_xy_mm"], dtype=float) / 1000.0
        r_mm = float(item["radius_mm"])
        h_mm = float(item["height_mm"])
        r_m = r_mm / 1000.0
        half_len = (h_mm / 1000.0) * 0.5
        center = np.array([xy[0], xy[1], half_len], dtype=float)
        return _append_cylinder(geom_model, name, r_m, half_len, center, None)

    raise ValueError(f"Unknown obstacle kind: {kind} for {name}")


def obstacle_name_to_code(name: str) -> Optional[int]:
    if name == "Obstacle_1":
        return 1
    if name == "Obstacle_2":
        return 2
    if name == "Obstacle_3":
        return 3
    return None


def build_scene_from_config(
    cfg: Dict[str, Any],
    project_root: Optional[Path] = None,
) -> SceneContext:
    """
    Build Pinocchio model + collision GeometryModel from collision config dict.

    Args:
        cfg: The ``collision`` subtree from ``collision_config.yaml``.
        project_root: Used to resolve relative URDF paths; default: repo root.
    """
    if project_root is None:
        project_root = Path(__file__).resolve().parent.parent

    urdf_rel = cfg["urdf_path"]
    urdf_path = urdf_rel
    p = Path(urdf_rel)
    if not p.is_absolute():
        cand = (project_root / urdf_rel).resolve()
        if cand.exists():
            urdf_path = str(cand)
        else:
            urdf_path = _resolve_urdf_path(urdf_rel)

    urdf_abs = str(Path(urdf_path).resolve())
    urdf_dir = os.path.dirname(urdf_abs)
    mesh_root = os.path.dirname(urdf_dir)

    model = pin.buildModelFromUrdf(urdf_abs)
    geom_model = pin.buildGeomFromUrdf(
        model,
        urdf_abs,
        pin.GeometryType.COLLISION,
        package_dirs=[urdf_dir, mesh_root],
    )
    n_robot = len(geom_model.geometryObjects)
    if n_robot == 0:
        raise ValueError(f"No collision geometry in URDF: {urdf_abs}")

    obs_cfg = cfg.get("obstacles", {})
    use_primitives = bool(obs_cfg.get("use_primitives", True))
    env_indices: Set[int] = set()
    env_name_to_code: Dict[str, int] = {}

    for item in obs_cfg.get("items", []):
        idx = _build_obstacle_geom(geom_model, item, use_primitives)
        env_indices.add(idx)
        code = obstacle_name_to_code(str(item["name"]))
        if code is not None:
            env_name_to_code[str(item["name"])] = code

    data = model.createData()
    geom_data = pin.GeometryData(geom_model)

    return SceneContext(
        model=model,
        data=data,
        geom_model=geom_model,
        geom_data=geom_data,
        env_geom_indices=env_indices,
        env_name_to_code=env_name_to_code,
        urdf_abs=urdf_abs,
    )
