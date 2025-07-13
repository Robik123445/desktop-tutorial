from cam_slicer.robotics.singularity_map import map_singularity_zones
from cam_slicer.robotics.interface import ArmKinematicProfile


def test_map_singularity_zones():
    """Detect dead zone for 2R arm beyond reach."""
    profile = ArmKinematicProfile(
        name="p",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
        workspace=[(-2, 2), (-2, 2), (0, 1)],
    )
    points = map_singularity_zones(profile, (0, 3), (0, 0), step=1)
    assert any(p[2] == "dead" for p in points)
