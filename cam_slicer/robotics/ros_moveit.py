import logging
import math
from dataclasses import dataclass, field
from typing import Iterable, List, Callable

logger = logging.getLogger(__name__)

try:  # optional ROS imports
    import rclpy
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except Exception as exc:  # pragma: no cover - ROS2 not installed
    logger.warning("ROS2 packages missing: %s", exc)
    rclpy = None

    @dataclass
    class JointTrajectoryPoint:  # type: ignore
        positions: List[float] = field(default_factory=list)
        time_from_start: float = 0.0

    @dataclass
    class JointTrajectory:  # type: ignore
        joint_names: List[str] = field(default_factory=list)
        points: List[JointTrajectoryPoint] = field(default_factory=list)

from .interface import ArmKinematicProfile

logger = logging.getLogger(__name__)


def toolpath_to_trajectory(
    toolpath: Iterable[Iterable[float]],
    profile: ArmKinematicProfile,
    joint_names: List[str],
    dt: float = 0.1,
) -> JointTrajectory:
    """Convert XYZABC path to a ROS ``JointTrajectory``."""
    traj = JointTrajectory()
