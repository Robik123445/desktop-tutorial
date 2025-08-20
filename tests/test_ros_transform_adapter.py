import math

from cam_slicer.ros_transform_adapter import Pose, toolpath_to_pose_list, transform_config_to_pose
from cam_slicer.robotics.kinematics import TransformConfig


def test_toolpath_to_pose_list():
    """Toolpath is converted to ``Pose`` objects."""
    path = [(0, 0, 0), (1, 2, 3)]
    poses = toolpath_to_pose_list(path)
    assert poses == [Pose(0.0, 0.0, 0.0), Pose(1.0, 2.0, 3.0)]


def test_transform_config_to_pose():
    """TransformConfig is mapped to a pose dict."""
    cfg = TransformConfig(rotation_deg=90, offset=(1, 2, 3))
    pose = transform_config_to_pose(cfg)
    assert pose["position"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert math.isclose(
        pose["orientation"]["w"], math.cos(math.radians(90) / 2), rel_tol=1e-6
    )

