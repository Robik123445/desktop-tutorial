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
    )
    workspace: Optional[Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]] = None

    def inverse_kinematics(self, pose: Sequence[float]) -> List[float]:
        """Compute inverse kinematics for a 2R planar arm."""
        if len(self.link_lengths) != 2 or self.joint_types != ["revolute", "revolute"]:
            raise NotImplementedError("IK only implemented for 2R arms")
        x, y = pose[0], pose[1]
        l1, l2 = self.link_lengths
        r2 = x * x + y * y
        cos_q2 = (r2 - l1 * l1 - l2 * l2) / (2 * l1 * l2)
        cos_q2 = max(min(cos_q2, 1.0), -1.0)
        q2 = math.acos(cos_q2)
        k1 = l1 + l2 * cos_q2
        k2 = l2 * math.sin(q2)
        q1 = math.atan2(y, x) - math.atan2(k2, k1)
        return [math.degrees(q1), math.degrees(q2)]

    def within_limits(self, joints: Sequence[float]) -> bool:
        """Return ``True`` if all joint angles are inside limits."""
        for angle, limit in zip(joints, self.joint_limits):
            if angle < limit[0] or angle > limit[1]:
                logger.debug("Joint angle %.2f outside %s", angle, limit)
                return False
        return True

    def within_workspace(self, pose: Sequence[float]) -> bool:
        """Check if XY position lies within defined workspace."""
        if self.workspace is None:
            return True
        x, y = pose[0], pose[1]
        z = pose[2] if len(pose) > 2 else 0.0
        (x_min, x_max), (y_min, y_max), (z_min, z_max) = self.workspace
        inside = x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max
        if not inside:
            logger.debug("Pose %.2f,%.2f,%.2f outside workspace", x, y, z)
        return inside

    def workspace_to_joints(self, pose: Sequence[float]) -> List[float]:
        """Convert pose to joint angles while checking limits and workspace."""
        if not self.within_workspace(pose):
            raise ValueError("pose outside workspace")
        joints = self.inverse_kinematics(pose)
        if not self.within_limits(joints):
            raise ValueError("joint limits exceeded")
        return joints

    def check_collision(self, joints: Sequence[float], fence: Optional["GeoFence"]) -> bool:
        """Return ``True`` if any joint lies inside a forbidden zone."""
        if fence is None:
            return False
        x = y = 0.0
        angle = 0.0
        for length, jtype, j in zip(self.link_lengths, self.joint_types, joints):
            if jtype == "revolute":
                angle += math.radians(j)
                x += length * math.cos(angle)
                y += length * math.sin(angle)
            else:
                x += (length + j) * math.cos(angle)
                y += (length + j) * math.sin(angle)
            if fence.is_inside(x, y):
                logger.debug("Collision at joint %.1f,%.1f", x, y)
                return True
        return False


def format_extended_g1(
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    a: float = 0.0,
    b: float = 0.0,
    c: float = 0.0,
    feed: Optional[float] = None,
) -> str:
    """Return extended ``G1`` line including optional feed."""
    line = (
        f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f} "
        f"A{a:.3f} B{b:.3f} C{c:.3f}"
    )
    if feed is not None:
        line += f" F{float(feed)}"
    return line


def format_move_arm(joints: Sequence[float], feed: Optional[float] = None) -> str:
    """Format ``MOVE_ARM`` joint command."""
    parts = ["MOVE_ARM"]
    for idx, val in enumerate(joints, 1):
        parts.append(f"J{idx}={val:.3f}")
    if feed is not None:
        parts.append(f"F{float(feed)}")
    return " ".join(parts)


def export_robotic_toolpath(
    toolpath: Sequence[Sequence[float]],
    profile: ArmKinematicProfile,
    *,
    mode: str = "xyzabc",
) -> List[str]:
    """Export a toolpath in either workspace or joint format."""
    lines: List[str] = []
    for pose in toolpath:
        pose = tuple(pose)
        if mode == "xyzabc":
            p6 = pose + (0.0,) * (6 - len(pose))
            lines.append(format_extended_g1(*p6))
        elif mode == "joint":
            joints = profile.workspace_to_joints(pose)
            lines.append(format_move_arm(joints))
        else:
            raise ValueError("mode must be 'xyzabc' or 'joint'")
    logger.info("Exported %d lines in mode %s", len(lines), mode)
    return lines
