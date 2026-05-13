#!/usr/bin/env python3
"""Load Feature 4 ``collision_config.yaml``."""

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml


def _project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def load_collision_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load collision configuration.

    Args:
        config_path: Path to YAML. If None, uses ``config/collision_config.yaml``
                      under the project root.
    """
    if config_path is None:
        config_path = str(_project_root() / "config" / "collision_config.yaml")
    path = Path(config_path)
    if not path.exists():
        raise FileNotFoundError(f"collision config not found: {config_path}")
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if "collision" not in data:
        raise ValueError("collision_config.yaml must have a top-level 'collision:' key")
    return data["collision"]


def resolve_project_path(maybe_relative: str) -> Path:
    """Resolve path relative to project root if not absolute."""
    p = Path(maybe_relative)
    if p.is_absolute():
        return p
    return (_project_root() / p).resolve()
