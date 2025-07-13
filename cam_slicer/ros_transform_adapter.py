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
except Exception:  # pragma: no cover - ROS2 not installed
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
    x, y, z = transform_point(0, 0, 0, cfg)
    quat = quaternion_from_euler(0.0, 0.0, math.radians(cfg.rotation_deg))
    return {"position": {"x": x, "y": y, "z": z}, "orientation": quat}


def toolpath_to_pose_list(
    toolpath: Iterable[Iterable[float]],
    cfg: Optional[TransformConfig] = None,
) -> List[Dict[str, Dict[str, float]]]:
    """Convert a toolpath to a list of pose dictionaries."""
    poses: List[Dict[str, Dict[str, float]]] = []
    tf = cfg or TransformConfig()
    for point in toolpath:
        x, y, z = point[:3]
        px, py, pz = transform_point(x, y, z, tf)
        quat = quaternion_from_euler(0.0, 0.0, math.radians(tf.rotation_deg))
        poses.append({"position": {"x": px, "y": py, "z": pz}, "orientation": quat})
    return poses


def pose_to_transform_config(pose: Dict[str, Dict[str, float]]) -> TransformConfig:
    """Convert a ROS pose dictionary back to :class:`TransformConfig`. Only yaw
    rotation is handled."""

    pos = pose.get("position", {})
    ori = pose.get("orientation", {})
    yaw = math.atan2(
        2 * (ori.get("w", 1.0) * ori.get("z", 0.0)),
        1 - 2 * (ori.get("z", 0.0) ** 2),
    )
    return TransformConfig(
        rotation_deg=math.degrees(yaw),
        offset=(pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)),
    )


def _matrix_from_config(cfg: TransformConfig) -> List[List[float]]:
    """Return 4x4 transform matrix for ``cfg``."""

    angle = math.radians(cfg.rotation_deg)
    c, s = math.cos(angle), math.sin(angle)
    return [
        [c * cfg.scale, -s * cfg.scale, 0.0, cfg.offset[0]],
        [s * cfg.scale, c * cfg.scale, 0.0, cfg.offset[1]],
        [0.0, 0.0, cfg.scale, cfg.offset[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _config_from_matrix(mat: List[List[float]]) -> TransformConfig:
    """Create :class:`TransformConfig` from 4x4 matrix."""

    scale = math.sqrt(mat[0][0] ** 2 + mat[1][0] ** 2)
    angle = math.atan2(mat[1][0], mat[0][0])
    return TransformConfig(
        rotation_deg=math.degrees(angle),
        scale=scale,
        offset=(mat[0][3], mat[1][3], mat[2][3]),
    )


def _mat_mul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    """Multiply two 4x4 matrices."""

    res = [[0.0 for _ in range(4)] for _ in range(4)]
    for i in range(4):
        for j in range(4):
            res[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
    return res


def compose_transforms(a: TransformConfig, b: TransformConfig) -> TransformConfig:
    """Compose two transforms ``a`` then ``b``."""

    mat = _mat_mul(_matrix_from_config(b), _matrix_from_config(a))
    return _config_from_matrix(mat)


def invert_transform(cfg: TransformConfig) -> TransformConfig:
    """Return the inverse of ``cfg``."""

    mat = _matrix_from_config(cfg)
    # inverse of rotation+scale matrix
    inv_scale = 1.0 / cfg.scale if cfg.scale != 0 else 0.0
    angle = -cfg.rotation_deg
    inv = _matrix_from_config(
        TransformConfig(rotation_deg=angle, scale=inv_scale, offset=(0, 0, 0))
    )
    inv_trans = [-cfg.offset[0], -cfg.offset[1], -cfg.offset[2], 1.0]
    inv[0][3] = inv[0][0] * inv_trans[0] + inv[0][1] * inv_trans[1] + inv[0][3]
    inv[1][3] = inv[1][0] * inv_trans[0] + inv[1][1] * inv_trans[1] + inv[1][3]
    inv[2][3] = inv_scale * inv_trans[2]
    return _config_from_matrix(inv)


def map_point_between_frames(
    point: Tuple[float, float, float],
    from_cfg: TransformConfig,
    to_cfg: TransformConfig,
) -> Tuple[float, float, float]:
    """Convert ``point`` defined in ``from_cfg`` frame to ``to_cfg`` frame."""

    mat_from = _matrix_from_config(from_cfg)
    vec = [point[0], point[1], point[2], 1.0]
    world = [
        sum(mat_from[i][j] * vec[j] for j in range(4)) for i in range(4)
    ]
    inv_to = invert_transform(to_cfg)
    mapped = _matrix_from_config(inv_to)
    x = sum(mapped[0][j] * world[j] for j in range(4))
    y = sum(mapped[1][j] * world[j] for j in range(4))
    z = sum(mapped[2][j] * world[j] for j in range(4))
    return float(x), float(y), float(z)


def gcode_to_pose_list(
    gcode_lines: Iterable[str],
    cfg: Optional[TransformConfig] = None,
) -> List[Dict[str, Dict[str, float]]]:
    """Parse G-code lines and convert moves to pose dictionaries."""

    poses: List[Dict[str, Dict[str, float]]] = []
    current = [0.0, 0.0, 0.0]
    tf = cfg or TransformConfig()
    for line in gcode_lines:
        if line.startswith("G"):
            for axis in "XYZ":
                if axis in line:
                    idx = line.find(axis)
                    val = ""
                    j = idx + 1
                    while j < len(line) and (line[j].isdigit() or line[j] in "-+.eE"):
                        val += line[j]
                        j += 1
                    try:
                        current["XYZ".index(axis)] = float(val)
                    except ValueError:
                        pass
            x, y, z = transform_point(current[0], current[1], current[2], tf)
            quat = quaternion_from_euler(0.0, 0.0, math.radians(tf.rotation_deg))
            poses.append({"position": {"x": x, "y": y, "z": z}, "orientation": quat})
    return poses


class ROSTransformAdapter:
    """Bridge between CAM coordinates and ROS TF frames."""

    def __init__(self, node_name: str = "transform_adapter") -> None:
        if rclpy is None:  # pragma: no cover - optional dependency
            raise ImportError("rclpy is required for ROSTransformAdapter")
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.broadcaster = TransformBroadcaster(self.node)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.logger = logging.getLogger(node_name)

    # ------------------------------------------------------------------ ROS TX
    # ------------------------------------------------------------------

    def broadcast_transform(
        self,
        cfg: TransformConfig,
        parent_frame: str = "world",
        child_frame: str = "tool",
    ) -> None:
        """Publish a static transform derived from ``cfg``."""
        if TransformStamped is object:  # pragma: no cover - ROS messages missing
            raise ImportError("geometry_msgs is required for broadcasting")
        msg = TransformStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame
        pose = transform_config_to_pose(cfg)
        msg.transform.translation.x = pose["position"]["x"]
        msg.transform.translation.y = pose["position"]["y"]
        msg.transform.translation.z = pose["position"]["z"]
        msg.transform.rotation.x = pose["orientation"]["x"]
        msg.transform.rotation.y = pose["orientation"]["y"]
        msg.transform.rotation.z = pose["orientation"]["z"]
        msg.transform.rotation.w = pose["orientation"]["w"]
        self.broadcaster.sendTransform(msg)
        self.logger.debug("Broadcasted transform %s -> %s", parent_frame, child_frame)

    def broadcast_toolpath(
        self,
        toolpath: Iterable[Tuple[float, float, float]],
        cfg: Optional[TransformConfig] = None,
        parent_frame: str = "world",
        child_prefix: str = "pt",
    ) -> None:
        """Broadcast each point in ``toolpath`` as a child frame."""

        if TransformStamped is object:  # pragma: no cover
            raise ImportError("geometry_msgs is required for broadcasting")
        tf_cfg = cfg or TransformConfig()
        for idx, pt in enumerate(toolpath):
            msg = TransformStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = parent_frame
            msg.child_frame_id = f"{child_prefix}_{idx}"
            x, y, z = transform_point(pt[0], pt[1], pt[2], tf_cfg)
            msg.transform.translation.x = x
            msg.transform.translation.y = y
            msg.transform.translation.z = z
            quat = quaternion_from_euler(0.0, 0.0, math.radians(tf_cfg.rotation_deg))
            msg.transform.rotation.x = quat["x"]
            msg.transform.rotation.y = quat["y"]
            msg.transform.rotation.z = quat["z"]
            msg.transform.rotation.w = quat["w"]
            self.broadcaster.sendTransform(msg)
        self.logger.info("Broadcasted %d toolpath points", idx + 1)

    def lookup_transform(self, target: str, source: str) -> Optional[Dict[str, Any]]:
        """Return the latest transform between frames or ``None`` if unavailable."""
        try:
            trans = self.buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception as exc:  # pragma: no cover - missing tf
            self.logger.error("Lookup failed: %s", exc)
            return None
        return {
            "translation": {
                "x": trans.transform.translation.x,
                "y": trans.transform.translation.y,
                "z": trans.transform.translation.z,
            },
            "rotation": {
                "x": trans.transform.rotation.x,
                "y": trans.transform.rotation.y,
                "z": trans.transform.rotation.z,
                "w": trans.transform.rotation.w,
            },
        }

    def lookup_transform_config(self, target: str, source: str) -> Optional[TransformConfig]:
        """Return :class:`TransformConfig` from TF lookup."""

        data = self.lookup_transform(target, source)
        if not data:
            return None
        pose = {"position": data["translation"], "orientation": data["rotation"]}
        return pose_to_transform_config(pose)

    def spin_once(self, timeout: float = 0.1) -> None:
        """Process pending TF messages once."""
        rclpy.spin_once(self.node, timeout_sec=timeout)

    def map_point(
        self,
        point: Tuple[float, float, float],
        from_frame: str,
        to_frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        """Transform ``point`` between ROS frames using TF."""

        data = self.lookup_transform(to_frame, from_frame)
        if not data:
            return None
        cfg = pose_to_transform_config({"position": data["translation"], "orientation": data["rotation"]})
        return transform_point(*point, cfg)

    def shutdown(self) -> None:
        """Destroy ROS node and shutdown rclpy."""
        self.node.destroy_node()
        rclpy.shutdown()
        self.logger.info("ROSTransformAdapter stopped")

