import time
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
    """Test monitoring and logging with DigitalTwin."""
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

