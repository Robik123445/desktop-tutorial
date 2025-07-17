import logging
import math
from dataclasses import dataclass, field
from typing import Iterable, List, Callable

try:
    import rclpy
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except Exception:
    rclpy = None

    @dataclass
    class JointTrajectoryPoint:
        positions: List[float] = field(default_factory=list)
        time_from_start: float = 0.0

    @dataclass
    class JointTrajectory:
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
    traj = JointTrajectory()
    traj.joint_names = joint_names
    for idx, pose in enumerate(toolpath):
        joints = profile.workspace_to_joints(pose)
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(j) for j in joints]
        if hasattr(pt, "time_from_start") and not isinstance(pt.time_from_start, float):
            sec = int(idx * dt)
            nanosec = int((idx * dt - sec) * 1e9)
            pt.time_from_start.sec = sec
            pt.time_from_start.nanosec = nanosec
        else:
            pt.time_from_start = idx * dt
        traj.points.append(pt)
    logger.info("Generated trajectory with %d points", len(traj.points))
    return traj

def send_joint_trajectory(
    trajectory: JointTrajectory,
    topic: str = "/joint_trajectory",
    *,
    publisher: Callable[[JointTrajectory], None] | None = None,
) -> None:
    if publisher:
        publisher(trajectory)
        return
    if rclpy is None:
        raise ImportError("rclpy is required for ROS publishing")
    rclpy.init()
    node = rclpy.create_node("cam_slicer_traj_pub")
    pub = node.create_publisher(JointTrajectory, topic, 10)
    pub.publish(trajectory)
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    logger.info("Trajectory published to %s", topic)
