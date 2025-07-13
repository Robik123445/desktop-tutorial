from cam_slicer.robotics.pipeline import convert_xyz_toolpath, batch_convert, stream_converted_toolpath
from cam_slicer.robotics.interface import ArmKinematicProfile


def test_convert_xyz_toolpath():
    """Convert simple XYZ path to joint commands."""
    profile = ArmKinematicProfile(
        name="planar",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
    )
    toolpath = [(2, 0, 0)]
    lines = convert_xyz_toolpath(toolpath, profile, mode="xyzabc")
    assert lines[0].startswith("G1")


def test_stream_converted_toolpath():
    """Ensure streaming sends generated lines."""
    profile = ArmKinematicProfile(
        name="planar",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
    )
    toolpath = [(2, 0, 0)]
    sent = []
    stream_converted_toolpath(toolpath, profile, sent.append, mode="joint")
    assert sent and sent[0].startswith("MOVE_ARM")

def test_batch_convert():
    """Batch conversion returns list per toolpath."""
    profile = ArmKinematicProfile(
        name="planar",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
    )
    toolpaths = [[(2, 0, 0)], [(1, 1, 0)]]
    out = batch_convert(toolpaths, profile, mode="xyzabc")
    assert len(out) == 2 and all(out)
