import types
from cam_slicer.machines import Machine, MachineManager
import cam_slicer.sender.serial_streamer as streamer

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


def test_parallel_jobs(monkeypatch, tmp_path):
    """Run jobs on two machines at once."""
    g1 = tmp_path / "a.gcode"
    g2 = tmp_path / "b.gcode"
    g1.write_text("G0 X0\nG1 X1")
    g2.write_text("G0 X0\nG1 X1")
    ser1 = DummySerial()
    ser2 = DummySerial()
    def serial_factory(port, baud, timeout=1):
        return ser1 if port == "P1" else ser2
    monkeypatch.setattr(streamer, "serial", types.SimpleNamespace(Serial=serial_factory))
    mgr = MachineManager()
    m1 = Machine("m1", "P1")
    m2 = Machine("m2", "P2")
    mgr.add_machine(m1)
    mgr.add_machine(m2)
    m1.add_job(str(g1))
    m2.add_job(str(g2))
    mgr.start_all_parallel()
    assert ser1.written == ["G0 X0", "G1 X1"]
    assert ser2.written == ["G0 X0", "G1 X1"]

def test_serial_jobs_with_error(monkeypatch, tmp_path):
    """Serial job execution handles streaming errors."""
    g1 = tmp_path / "a.gcode"
    g2 = tmp_path / "b.gcode"
    g1.write_text("G0 X0\nG1 X1")
    g2.write_text("G0 X0\nG1 X1")
    class ErrorSerial(DummySerial):
        def __init__(self):
            super().__init__()
            self.count = 0
        def readline(self):
            self.count += 1
            if self.count == 1:
                return b"error:1\n"
            return b"ok\n"
    err_ser = ErrorSerial()
    ok_ser = DummySerial()
    def serial_factory(port, baud, timeout=1):
        return err_ser if port == "P1" else ok_ser
    monkeypatch.setattr(streamer, "serial", types.SimpleNamespace(Serial=serial_factory))
    mgr = MachineManager()
    m1 = Machine("m1", "P1")
    m2 = Machine("m2", "P2")
    mgr.add_machine(m1)
    mgr.add_machine(m2)
    m1.add_job(str(g1))
    m2.add_job(str(g2))
    mgr.start_serial()
    # error machine should not crash manager and second job should run
    assert ok_ser.written == ["G0 X0", "G1 X1"]

def test_assign_jobs_optimized():
    """Jobs are distributed to balance estimated time."""
    mgr = MachineManager()
    m1 = Machine("m1", "P1")
    m2 = Machine("m2", "P2")
    mgr.add_machine(m1)
    mgr.add_machine(m2)
    jobs = ["a.gcode", "b.gcode", "c.gcode"]
    mgr.assign_jobs_optimized(jobs, lambda p: 1 if p == "a.gcode" else 2)
    assert len(m1.jobs) + len(m2.jobs) == 3
