import logging
from typing import Iterable, Sequence

from cam_slicer.logging_config import setup_logging
from .interface import ArmKinematicProfile
from .safety import emergency_stop

setup_logging()
logger = logging.getLogger(__name__)


class TrajectoryCorrector:
    """Dynamically adjust trajectories to keep the robot safe."""

    def __init__(self, profile: ArmKinematicProfile, obstacles=None) -> None:
        self.profile = profile
        self.obstacles = obstacles

    def correct(self, joints: Sequence[float]) -> Sequence[float]:
        """Clamp joint angles and lift Z on collision."""
        corrected = list(joints)
        for i, limits in enumerate(self.profile.joint_limits):
            lo, hi = limits
            if corrected[i] < lo:
                logger.warning("Joint %d below limit: %.2f < %.2f", i, corrected[i], lo)
                corrected[i] = lo
            elif corrected[i] > hi:
                logger.warning("Joint %d above limit: %.2f > %.2f", i, corrected[i], hi)
                corrected[i] = hi
        if self.profile.check_collision(corrected, self.obstacles):
            logger.error("Collision predicted, lifting Z")
            if len(corrected) >= 3:
                corrected[2] += 5.0
            emergency_stop()
        return corrected

    def correct_path(self, path: Iterable[Sequence[float]]) -> list[Sequence[float]]:
        """Return a corrected path."""
        return [self.correct(p) for p in path]
