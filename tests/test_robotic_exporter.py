from cam_slicer.robotics.exporter import export_toolpath, stream_robotic_toolpath
from cam_slicer.robotics.interface import ArmKinematicProfile
from cam_slicer.core.header_footer import ControllerConfig
import types
import logging


def _profile():
    return ArmKinematicProfile(
        name="planar",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
    )


def test_export_robot_mode_xyzabc():
    """Export in XYZABC format."""
    prof = _profile()
    tp = [(2, 0, 0)]
    lines = export_toolpath(tp, prof, robot_mode=True, mode="xyzabc")
    assert lines[0].startswith("G1 X")


def test_export_robot_mode_joint():
    """Export in joint MOVE_ARM format."""
    prof = _profile()
    tp = [(2, 0, 0)]
    lines = export_toolpath(tp, prof, robot_mode=True, mode="joint")
    assert lines[0].startswith("MOVE_ARM")


def test_export_cnc_mode():
    """Fallback to standard G-code when robot_mode=False."""
    prof = _profile()
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    tp = [(0, 0, 0), (1, 0, 0)]
    lines = export_toolpath(tp, prof, cfg, robot_mode=False)
    assert lines[0] == "grbl-header"


def test_stream_robotic_toolpath(monkeypatch, caplog):
    """Stream converted commands to serial."""
    class DummySerial:
        def __init__(self):
            self.written = []
        def write(self, data):
            self.written.append(data.decode().strip())
        def readline(self):
            return b"ok\n"
        def __enter__(self):
            return self
        def __exit__(self, exc_type, exc, tb):
            pass

    dummy = DummySerial()
    monkeypatch.setattr(
        "cam_slicer.sender.serial_streamer.serial",
        types.SimpleNamespace(Serial=lambda *a, **kw: dummy),
    )
    prof = _profile()
    caplog.set_level(logging.INFO)
    stream_robotic_toolpath([(1, 0, 0)], prof, "COM1")
    assert dummy.written
    assert any("Robot run completed" in rec.message for rec in caplog.records)
