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
    dh_params: List[Tuple[float, float, float, float]] | None = None
    urdf_path: str | None = None
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
            "dh_params": self.dh_params,
            "urdf_path": self.urdf_path,
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
            dh_params=[tuple(p) for p in data.get("dh_params", [])] or None,
            urdf_path=data.get("urdf_path"),
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
        """Compute end-effector position for planar or DH-based arms."""
        if self.dh_params:
            try:
                import numpy as np
            except Exception as exc:  # pragma: no cover - numpy optional
                raise ImportError("numpy required for DH kinematics") from exc

            def _dh(a, alpha, d, theta):
                ca, sa = math.cos(alpha), math.sin(alpha)
                ct, st = math.cos(theta), math.sin(theta)
                return np.array([
                    [ct, -st * ca, st * sa, a * ct],
                    [st, ct * ca, -ct * sa, a * st],
                    [0.0, sa, ca, d],
                    [0.0, 0.0, 0.0, 1.0],
                ])

            T = np.eye(4)
            for (alpha, a, d, theta0), q in zip(self.dh_params, joints):
                T = T @ _dh(a, alpha, d, math.radians(q) + theta0)
            pos = T[:3, 3]
            return float(pos[0]), float(pos[1]), float(pos[2])

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
        """Compute joint angles for planar or 6-DOF arms."""
        if self.dh_params:
            try:
                import numpy as np
            except Exception:  # pragma: no cover - numpy optional
                x, y, z, rx, ry, rz = pose
                return [x, y, z, math.degrees(rx), math.degrees(ry), math.degrees(rz)]

            def _dh(a, alpha, d, theta):
                ca, sa = math.cos(alpha), math.sin(alpha)
                ct, st = math.cos(theta), math.sin(theta)
                return np.array([
                    [ct, -st * ca, st * sa, a * ct],
                    [st, ct * ca, -ct * sa, a * st],
                    [0.0, sa, ca, d],
                    [0.0, 0.0, 0.0, 1.0],
                ])

            def _forward(q):
                T = np.eye(4)
                for (alpha, a, d, t0), qq in zip(self.dh_params, q):
                    T = T @ _dh(a, alpha, d, math.radians(qq) + t0)
                return T

            def _pose_matrix(p):
                x, y, z, rx, ry, rz = p
                cx, sx = math.cos(rx), math.sin(rx)
                cy, sy = math.cos(ry), math.sin(ry)
                cz, sz = math.cos(rz), math.sin(rz)
                Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
                Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
                Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
                R = Rz @ Ry @ Rx
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = [x, y, z]
                return T

            def _mat_to_error(T_cur, T_des):
                pos_err = T_des[:3, 3] - T_cur[:3, 3]
                R_err = T_cur[:3, :3].T @ T_des[:3, :3]
                angle = math.acos(max(min((np.trace(R_err) - 1) / 2, 1.0), -1.0))
                if abs(angle) < 1e-6:
                    ori_err = np.zeros(3)
                else:
                    axis = np.array([
                        R_err[2, 1] - R_err[1, 2],
                        R_err[0, 2] - R_err[2, 0],
                        R_err[1, 0] - R_err[0, 1],
                    ]) / (2 * math.sin(angle))
                    ori_err = axis * angle
                return np.concatenate([pos_err, ori_err])

            def _jacobian(q):
                J = np.zeros((6, len(q)))
                eps = 1e-5
                f0 = _forward(q)
                for i in range(len(q)):
                    dq = q.copy()
                    dq[i] += math.degrees(eps)
                    f1 = _forward(dq)
                    err = _mat_to_error(f0, f1)
                    J[:, i] = err / eps
                return J

            q = np.array([0.0] * len(self.dh_params))
            T_target = _pose_matrix(pose)
            for _ in range(100):
                T_cur = _forward(q)
                err = _mat_to_error(T_cur, T_target)
                if np.linalg.norm(err) < 1e-3:
                    return q.tolist()
                J = _jacobian(q)
                dq = np.linalg.pinv(J) @ err
                q += dq * 180 / math.pi
            raise ValueError("IK did not converge")

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
        # ... zvyšok funkcií ako predtým
