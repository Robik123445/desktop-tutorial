from cam_slicer.robotics.interface import ArmKinematicProfile
from cam_slicer.robotics.pipeline import convert_xyz_toolpath
from cam_slicer.robotics.ros_moveit import toolpath_to_trajectory, send_joint_trajectory

def _profile():
    dh = [
        (0.0, 0.0, 0.1, 0.0),
        (-1.57079632679, 0.0, 0.0, 0.0),
        (0.0, 0.4, 0.0, 0.0),
        (-1.57079632679, 0.0, 0.3, 0.0),
        (1.57079632679, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.1, 0.0),
    ]
    return ArmKinematicProfile(name="arm6", joint_limits=[(-180, 180)] * 6, dh_params=dh)

def test_convert_xyz_toolpath_6dof():
    prof = _profile()
    tp = [(0.2, 0.0, 0.2, 0.0, 0.0, 0.0)]
    lines = convert_xyz_toolpath(tp, prof, mode="joint")
    assert lines and lines[0].startswith("MOVE_ARM")

def test_toolpath_to_trajectory_and_send(monkeypatch):
    prof = _profile()
    tp = [(0.2, 0.0, 0.2, 0.0, 0.0, 0.0)]
    traj = toolpath_to_trajectory(tp, prof, [f"j{i}" for i in range(6)])
    assert len(traj.points) == 1
    assert len(traj.points[0].positions) == 6

    published = []
    send_joint_trajectory(traj, publisher=published.append)
    assert published
