from cam_slicer.robotics.collision_alerts import CollisionAlerts
from cam_slicer.robotics.interface import ArmKinematicProfile
from cam_slicer.utils.geofence import GeoFence


def _profile():
    return ArmKinematicProfile(
        name="p",
        link_lengths=[10, 10],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
    )


def test_collision_detection():
    """Collision alert returns True when pose intersects zone."""
    profile = _profile()
    fence = GeoFence(forbidden_zones=[(0, 0, 5, 5)])
    alerts = CollisionAlerts(profile, fence)
    # pose that will place first link inside zone
    collided = alerts.check_pose([0.0, 0.0])
    assert collided
