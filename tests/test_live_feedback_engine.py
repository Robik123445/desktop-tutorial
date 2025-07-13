from cam_slicer.robotics.live_feedback_engine import LiveFeedbackEngine
from cam_slicer.robotics.interface import ArmKinematicProfile


def _profile():
    return ArmKinematicProfile(
        name="p",
        link_lengths=[1, 1, 1],
        joint_types=["revolute", "revolute", "revolute"],
        joint_limits=[(-90, 90), (-90, 90), (-90, 90)],
    )


def test_process_pose_returns_corrected():
    """Feedback engine should return corrected pose."""
    engine = LiveFeedbackEngine(_profile())
    pose = engine.process_pose([100, 0, 0])
    assert pose[0] == 90
