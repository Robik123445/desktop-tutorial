import time
import pytest
from cam_slicer.digital_twin import DigitalTwin

class DummyConn:
    def __init__(self, lines):
        self.lines = lines
    def readline(self):
        if self.lines:
            return self.lines.pop(0)
        time.sleep(0.01)
        return ""


def test_digital_twin_monitoring(tmp_path):
    """Test monitoring a logging with DigitalTwin."""
    conn = DummyConn(["X0 Y0 Z0 F100 T1 A0", "X1 Y0 Z0 F100 T1 A0"])
    twin = DigitalTwin(conn)
    twin.start_monitoring()
    time.sleep(0.05)
    twin.stop_monitoring()
    state = twin.get_live_state()
    assert state["x"] == 1.0
    twin.log_status(tmp_path / "report.txt")
    assert (tmp_path / "report.txt").exists()
    dev = twin.compare_with_planned([(0,0,0),(1,0,0)])
    assert dev == []


def test_simulate_toolpath_preview():
    """Toolpath simulation returns a 3D figure when requested."""
    pytest.importorskip("matplotlib")
    twin = DigitalTwin(None)
    tp = [(0, 0, 0), (1, 0, 1), (2, 0, 0)]
    fig = twin.simulate_toolpath(tp, interval=0, show_3d=True)
    assert fig is not None
    ax = fig.axes[0]
    assert getattr(ax, "name", "") == "3d"
    assert len(ax.lines[0].get_xdata()) == len(tp)
