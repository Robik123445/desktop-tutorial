import time
from cam_slicer.digital_twin import WorkshopTwin

class DummyConn:
    def __init__(self, lines):
        self.lines = lines
    def readline(self):
        if self.lines:
            return self.lines.pop(0)
        time.sleep(0.01)
        return ""

def test_workshop_twin_states():
    w = WorkshopTwin()
    c1 = DummyConn(["X0 Y0 Z0", "X1 Y0 Z0"])
    c2 = DummyConn(["X0 Y0 Z0", "X0 Y2 Z0"])
    w.add_machine("m1", c1)
    w.add_machine("m2", c2)
    w.start_all()
    time.sleep(0.05)
    w.stop_all()
    states = w.get_states()
    assert states["m1"]["x"] == 1.0
    assert states["m2"]["y"] == 2.0

def test_workshop_simulation():
    w = WorkshopTwin()
    w.add_machine("m1", None)
    w.twins["m1"].simulate_toolpath([(0,0,0),(2,0,0)], interval=0)
    states = w.get_states()
    assert states["m1"]["x"] == 2.0
