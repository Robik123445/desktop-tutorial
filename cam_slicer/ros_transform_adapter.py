"""Utilities to map CAM coordinates to ROS TF frames and back."""

from __future__ import annotations

import math
import logging
from typing import Iterable, List, Dict, Any, Optional, Tuple

from .logging_config import setup_logging
from .robotics.kinematics import TransformConfig, transform_point

setup_logging()
logger = logging.getLogger(__name__)

try:  # optional ROS imports
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster, Buffer, TransformListener
except Exception as exc:  # pragma: no cover - ROS2 not installed
    logger.warning("ROS2 packages missing: %s", exc)
    rclpy = None  # type: ignore
    TransformStamped = object  # type: ignore
    TransformBroadcaster = object  # type: ignore
    Buffer = object  # type: ignore
    TransformListener = object  # type: ignore


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
    """Convert :class:`TransformConfig` to a ROS pose dictionary."""
