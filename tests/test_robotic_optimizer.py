from cam_slicer.robotics.trajectory_optimizer import optimize_robotic_trajectory
from cam_slicer.robotics.interface import ArmKinematicProfile


def test_optimize_robotic_trajectory_warnings():
    """Return warnings when path hits joint limits."""
    profile = ArmKinematicProfile(
        name="p",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-90, 90), (-90, 90)],
    )
    path = [(0.5, 0.5, 0), (2, 0, 0)]  # second pose outside workspace (large reach)
    optimized, warnings = optimize_robotic_trajectory(path, profile)
    assert optimized
    assert warnings
