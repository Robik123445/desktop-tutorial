import logging
from typing import Sequence, Callable, Optional

from cam_slicer.logging_config import setup_logging
from .interface import ArmKinematicProfile
from .trajectory_corrector import TrajectoryCorrector

setup_logging()
logger = logging.getLogger(__name__)


class LiveFeedbackEngine:
    """Monitor live joint data and apply corrections if needed."""

    def __init__(
        self,
        profile: ArmKinematicProfile,
        *,
        obstacles=None,
        on_corrected: Optional[Callable[[Sequence[float]], None]] = None,
    ) -> None:
        self.corrector = TrajectoryCorrector(profile, obstacles)
        self.on_corrected = on_corrected

    def process_pose(self, joints: Sequence[float]) -> Sequence[float]:
        """Check pose and return corrected version.``joints`` can be joint angles or XYZABC."""
        corrected = self.corrector.correct(joints)
        if corrected != list(joints):
            logger.info("Pose corrected from %s to %s", joints, corrected)
            if self.on_corrected:
                self.on_corrected(corrected)
        return corrected
