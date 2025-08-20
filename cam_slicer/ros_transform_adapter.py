"""Utilities to map CAM coordinates to ROS TF frames and back."""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass
from typing import Iterable, List, Dict, Any, Optional, Tuple

from .logging_config import setup_logging
from .robotics.kinematics import TransformConfig

# Configure logging early so functions can emit debug info into ``log.txt``.
setup_logging()
logger = logging.getLogger(__name__)

try:  # optional ROS imports – these may not exist in minimal setups
    import rclpy  # type: ignore
    from geometry_msgs.msg import TransformStamped  # type: ignore
    from tf2_ros import TransformBroadcaster, Buffer, TransformListener  # type: ignore
except Exception as exc:  # pragma: no cover - ROS2 not installed
    # Log and replace with harmless placeholders so the module remains usable.
    logger.warning("ROS2 packages missing: %s", exc)
    rclpy = None  # type: ignore
    TransformStamped = object  # type: ignore
    TransformBroadcaster = object  # type: ignore
    Buffer = object  # type: ignore
    TransformListener = object  # type: ignore


@dataclass
class Pose:
    """Minimal 3D pose container."""

    x: float
    y: float
    z: float


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Dict[str, float]:
    """Return quaternion from Euler angles (radians)."""
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)
    return {
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
        "w": cr * cp * cy + sr * sp * sy,
    }


def transform_config_to_pose(cfg: TransformConfig) -> Dict[str, Dict[str, float]]:
    """Convert :class:`TransformConfig` to a ROS-like pose dictionary.

    Only the rotation around the Z axis is considered which suits the
    planar nature of most CAM toolpaths.
    """

    logger.debug("Converting TransformConfig %s to pose", cfg)
    yaw = math.radians(cfg.rotation_deg)
    quat = quaternion_from_euler(0.0, 0.0, yaw)
    pose = {
        "position": {
            "x": float(cfg.offset[0]),
            "y": float(cfg.offset[1]),
            "z": float(cfg.offset[2]),
        },
        "orientation": quat,
    }
    return pose


def toolpath_to_pose_list(toolpath: Iterable[Tuple[float, float, float]]) -> List[Pose]:
    """Convert raw toolpath coordinates to a list of :class:`Pose`.

    Parameters
    ----------
    toolpath : Iterable[Tuple[float, float, float]]
        Sequence of ``(x, y, z)`` points.
    """

    poses = [Pose(float(x), float(y), float(z)) for x, y, z in toolpath]
    logger.debug("Converted %d toolpath points to poses", len(poses))
    return poses
