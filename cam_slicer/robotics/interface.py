from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import json
import logging
import math
from typing import Callable, List, Tuple, Sequence

try:  # optional, for YAML profiles
    import yaml  # type: ignore
except Exception:  # pragma: no cover - yaml optional
    yaml = None

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
    tcp: Tuple[float, float, float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    workspace: Tuple[
        Tuple[float, float],
        Tuple[float, float],
        Tuple[float, float],
    ] | None = None

    def to_dict(self) -> dict:
        """Serialize profile to a dictionary."""
        return {
            "name": self.name,
            "link_lengths": self.link_lengths,
            "joint_types": self.joint_types,
            "joint_limits": self.joint_limits,
            "tcp": list(self.tcp),
            "workspace": self.workspace,
        }

    def save(self, path: str | Path) -> None:
        """Save profile to JSON or YAML."""
        data = self.to_dict()
        if str(path).lower().endswith((".yml", ".yaml")) and yaml:
            Path(path).write_text(yaml.safe_dump(data))
        else:
            Path(path).write_text(json.dumps(data, indent=2))
        logger.info("Saved profile %s to %s", self.name, path)

    @staticmethod
    def load(path: str | Path) -> "ArmKinematicProfile":
        """Load profile from JSON or YAML."""
        text = Path(path).read_text()
        if str(path).lower().endswith((".yml", ".yaml")) and yaml:
            data = yaml.safe_load(text)
        else:
            data = json.loads(text)
        profile = ArmKinematicProfile(
            name=data["name"],
            link_lengths=data.get("link_lengths", []),
            joint_types=data.get("joint_types", []),
            joint_limits=[tuple(l) for l in data.get("joint_limits", [])],
            tcp=tuple(data.get("tcp", (0, 0, 0, 0, 0, 0))),
            workspace=(
                tuple(tuple(w) for w in data.get("workspace"))
                if data.get("workspace")
                else None
            ),
        )
        logger.info("Loaded kinematic profile %s", profile.name)
        return profile

    # --- Kinematics -----------------------------------------------------

    def forward_kinematics(self, joints: Sequence[float]) -> Tuple[float, float, float]:
        """Compute planar XY position of the end effector."""
        x = 0.0
        y = 0.0
        angle = 0.0
        for length, jtype, a in zip(self.link_lengths, self.joint_types, joints):
            if jtype == "revolute":
                angle += math.radians(a)
                x += length * math.cos(angle)
                y += length * math.sin(angle)
            else:  # prismatic
                x += (length + a) * math.cos(angle)
                y += (length + a) * math.sin(angle)
        return (x, y, angle)

    def inverse_kinematics(self, pose: Sequence[float]) -> List[float]:
        """Solve planar two-link inverse kinematics."""
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

    # --- Checks ---------------------------------------------------------

    def within_limits(self, joints: Sequence[float]) -> bool:
        """Return ``True`` if all joint angles are inside limits."""
        for angle, limit in zip(joints, self.joint_limits):
            if angle < limit[0] or angle > limit[1]:
                logger.debug("Joint angle %.2f outside %s", angle, limit)
                return False
        return True

    def within_workspace(self, pose: Sequence[float]) -> bool:
        """Check if XY position lies within defined workspace."""
        if not self.workspace:
            return True
        (xmin, xmax), (ymin, ymax), (zmin, zmax) = self.workspace
        x, y, z = pose[0], pose[1], pose[2] if len(pose) > 2 else 0.0
        inside = xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax
        if not inside:
            logger.debug("Point %s outside workspace", pose)
        return inside

    def workspace_to_joints(self, pose: Sequence[float]) -> List[float]:
        """Convert a workspace pose to joint angles with limit check."""
        joints = self.inverse_kinematics(pose)
        if not self.within_limits(joints):
            raise ValueError("Target pose violates joint limits")
        if not self.within_workspace(pose):
            raise ValueError("Target pose outside workspace")
        return joints

    def check_collision(
        self, joints: Sequence[float], obstacles: "GeoFence" | None = None
    ) -> bool:
        """Return ``True`` if any joint intersects a forbidden zone."""
        try:
            from cam_slicer.utils.geofence import GeoFence
        except Exception:  # pragma: no cover - optional import
            GeoFence = None  # type: ignore

        if obstacles is None or GeoFence is None:
            return False

        x = 0.0
        y = 0.0
        angle = 0.0
        if obstacles.is_inside(x, y):
            logger.debug("Base inside forbidden zone")
            return True
        for length, jtype, a in zip(self.link_lengths, self.joint_types, joints):
            if jtype == "revolute":
                angle += math.radians(a)
                x += length * math.cos(angle)
                y += length * math.sin(angle)
            else:
                x += (length + a) * math.cos(angle)
                y += (length + a) * math.sin(angle)
            if obstacles.is_inside(x, y):
                logger.debug("Joint at X%.2f Y%.2f collides with obstacle", x, y)
                return True
        return False

    @staticmethod
    def from_json(path: str | Path) -> "ArmKinematicProfile":
        """Backward compatible JSON loader."""
        return ArmKinematicProfile.load(path)


def format_extended_g1(
    x: float | None = None,
    y: float | None = None,
    z: float | None = None,
    a: float | None = None,
    b: float | None = None,
    c: float | None = None,
    feed: float | None = None,
) -> str:
    """Format a G1 command with up to six axes."""
    parts = ["G1"]
    for axis, val in zip("XYZABC", (x, y, z, a, b, c)):
        if val is not None:
            parts.append(f"{axis}{val:.3f}")
    if feed is not None:
        parts.append(f"F{feed:.1f}")
    line = " ".join(parts)
    logger.debug("Generated G1 line: %s", line)
    return line


def format_move_arm(joints: List[float]) -> str:
    """Format a custom MOVE_ARM command with joint angles in degrees."""
    args = " ".join(f"J{i+1}={angle:.3f}" for i, angle in enumerate(joints))
    line = f"MOVE_ARM {args}"
    logger.debug("Generated MOVE_ARM line: %s", line)
    return line


def export_robotic_toolpath(
    toolpath: List[Tuple[float, float, float, float, float, float]],
    profile: ArmKinematicProfile,
    mode: str = "xyzabc",
) -> List[str]:
    """Export toolpath in robotic arm format.

    Parameters
    ----------
    toolpath : list of (x, y, z, a, b, c)
        Cartesian path in millimetres and degrees.
    profile : ArmKinematicProfile
        Arm kinematics description.
    mode : str
        ``'xyzabc'`` for extended G-code or ``'joint'`` for MOVE_ARM commands.
    """
    lines: List[str] = []
    if mode == "xyzabc":
        for p in toolpath:
            lines.append(format_extended_g1(*p))
    elif mode == "joint":
        for p in toolpath:
            lines.append(format_move_arm(list(p)))
    else:
        raise ValueError("mode must be 'xyzabc' or 'joint'")
    logger.info("Exported %d lines using mode %s", len(lines), mode)
    return lines
