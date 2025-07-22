"""Simple robotic arm control helpers."""
from __future__ import annotations

import logging
from typing import Sequence

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


def move_to_pose(pose: Sequence[float]) -> str:
    """Move robotic arm to the given pose.

    Parameters
    ----------
    pose : Sequence[float]
        Target pose defined as XYZABC.
    """
    logger.info("Moving arm to pose %s", pose)
    # Placeholder for real hardware integration
    return "ok"
