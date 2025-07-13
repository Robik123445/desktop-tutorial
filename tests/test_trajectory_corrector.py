from cam_slicer.robotics.trajectory_corrector import TrajectoryCorrector
from cam_slicer.robotics.interface import ArmKinematicProfile


def _profile():
    return ArmKinematicProfile(
        name="p",
        link_lengths=[1, 1, 1],
        joint_types=["revolute", "revolute", "revolute"],
        joint_limits=[(-90, 90), (-90, 90), (-90, 90)],
    )


def test_correct_clamps_limits():
    """Angles beyond limits are clamped."""
    corr = TrajectoryCorrector(_profile())
    out = corr.correct([100, 0, 0])
    assert out[0] == 90
