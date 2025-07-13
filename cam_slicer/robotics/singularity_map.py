"""Singularity and dead-zone simulation utilities."""
from __future__ import annotations

import logging
from typing import List, Tuple

from cam_slicer.logging_config import setup_logging
from .interface import ArmKinematicProfile

setup_logging()
logger = logging.getLogger(__name__)


def map_singularity_zones(
    profile: ArmKinematicProfile,
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    *,
    step: float = 50.0,
    margin: float = 5.0,
) -> List[Tuple[float, float, str]]:
    """Return grid points that are unreachable or near singularity."""
    points: List[Tuple[float, float, str]] = []
    x = x_range[0]
    while x <= x_range[1]:
        y = y_range[0]
        while y <= y_range[1]:
            try:
                joints = profile.workspace_to_joints((x, y, 0.0))
            except Exception:
                points.append((x, y, "dead"))
                y += step
                continue
            if len(joints) > 1 and abs(abs(joints[1]) - 180) < margin:
                points.append((x, y, "singularity"))
            y += step
        x += step
    logger.info("Mapped %d dangerous points", len(points))
    return points

