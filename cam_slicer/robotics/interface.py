from dataclasses import dataclass, field
from pathlib import Path
import json
import logging
import math
from typing import Callable, List, Tuple, Sequence, Optional

try:  # optional, for YAML profiles
    import yaml  # type: ignore
except Exception as exc:  # pragma: no cover - yaml optional
    logging.getLogger(__name__).warning("PyYAML not installed: %s", exc)
    yaml = None

try:  # optional geofence utilities
    from cam_slicer.utils.geofence import GeoFence
except Exception:  # pragma: no cover - optional
    GeoFence = None  # type: ignore

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


@dataclass
class ArmKinematicProfile:
    """Robotic arm description with basic kinematics."""

    name: str
    link_lengths: List[float] = field(default_factory=list)
    joint_types: List[str] = field(default_factory=list)
    joint_limits: List[Tuple[float, float]] = field(default_factory=list)
    dh_params: Optional[List[Tuple[float, float, float, float]]] = None
    urdf_path: Optional[str] = None
    tcp: Tuple[float, float, float, float, float, float] = (
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
