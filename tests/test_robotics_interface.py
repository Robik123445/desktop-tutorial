import json
import pytest
from cam_slicer.robotics.interface import (
    ArmKinematicProfile,
    format_extended_g1,
    format_move_arm,
    export_robotic_toolpath,
)


def test_format_extended_g1():
    """Test extended G1 line generation."""
    line = format_extended_g1(x=1, y=2, z=3, a=10, b=20, c=30, feed=1000)
    assert line.startswith("G1 X1.000 Y2.000 Z3.000 A10.000 B20.000 C30.000 F1000.0")


def test_load_profile(tmp_path):
    """Ensure kinematic profile loads from YAML."""
    path = tmp_path / "profile.json"
    data = {
        "name": "test",
        "link_lengths": [1, 1],
        "joint_types": ["revolute", "revolute"],
        "joint_limits": [[-90, 90], [-90, 90]],
        "workspace": [[-2, 2], [-2, 2], [0, 1]],
    }
    path.write_text(json.dumps(data))
    profile = ArmKinematicProfile.load(path)
    assert profile.link_lengths == [1, 1]


def test_export_robotic_toolpath_xyzabc():
    """Export toolpath using XYZABC mode."""
    profile = ArmKinematicProfile("p", (0,0,0,0,0,0), [(0,1)], ((0,1),(0,1),(0,1)))
    toolpath = [(1,2,3,4,5,6)]
    lines = export_robotic_toolpath(toolpath, profile, mode="xyzabc")
    assert lines[0].startswith("G1")


def test_export_robotic_toolpath_joint():
    """Export toolpath using joint mode."""
    profile = ArmKinematicProfile("p", (0,0,0,0,0,0), [(0,1)], ((0,1),(0,1),(0,1)))
    toolpath = [(1,2,3,4,5,6)]
    lines = export_robotic_toolpath(toolpath, profile, mode="joint")
    assert lines[0].startswith("MOVE_ARM")


def test_forward_inverse_kinematics():
    """Validate simple planar kinematics."""
    profile = ArmKinematicProfile(
        name="planar",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
    )
    pose = profile.forward_kinematics([0, 0])
    assert pose[0] == pytest.approx(2.0)
    joints = profile.inverse_kinematics((2, 0))
    assert joints[0] == pytest.approx(0.0, abs=1e-6)
    assert profile.within_limits(joints)


def test_collision_check():
    """Detect collision with forbidden zone."""
    profile = ArmKinematicProfile(
        name="planar",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
    )
    from cam_slicer.utils.geofence import GeoFence

    fence = GeoFence(forbidden_zones=[(0.0, 1.0, 0.5, 2.5)])
    collision = profile.check_collision([0, 0], fence)
    assert not collision
    collision = profile.check_collision([90, 0], fence)
    assert collision
