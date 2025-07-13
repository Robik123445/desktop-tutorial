"""Collision alert system for robotic arms."""
from __future__ import annotations

import logging
from typing import Sequence

from cam_slicer.logging_config import setup_logging
from cam_slicer.utils.geofence import GeoFence
from .interface import ArmKinematicProfile
from .safety import emergency_stop

setup_logging()
logger = logging.getLogger(__name__)

class CollisionAlerts:
    """Trigger warnings when the arm enters forbidden zones."""

    def __init__(self, profile: ArmKinematicProfile, geofence: GeoFence) -> None:
        self.profile = profile
        self.geofence = geofence

    def check_pose(self, joints: Sequence[float]) -> bool:
        """Return ``True`` if collision is detected and trigger alert."""
        if self.profile.check_collision(joints, self.geofence):
            logger.error("Collision detected at joints %s", joints)
            emergency_stop()
            return True
        return False
