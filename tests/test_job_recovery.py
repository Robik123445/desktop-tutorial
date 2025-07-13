import json
import types
import pytest
from cam_slicer.sender import job_recovery

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


def test_stream_with_recovery(monkeypatch, tmp_path):
    """Stream G-code and store checkpoint."""
    gfile = tmp_path / "test.gcode"
    gfile.write_text("G1 X0\nG1 X1\nG1 X2")
    ckpt = tmp_path / "state.json"
    dummy = DummySerial()
    monkeypatch.setattr(job_recovery, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy))
    import cam_slicer.sender.serial_streamer as streamer
    monkeypatch.setattr(streamer, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy))

    job_recovery.stream_with_recovery(str(gfile), "COM1", checkpoint=str(ckpt))

    data = json.loads(ckpt.read_text())
    assert data["line"] == 2
    assert len(dummy.written) == 3

    dummy2 = DummySerial()
    monkeypatch.setattr(job_recovery, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy2))
    import cam_slicer.sender.serial_streamer as streamer
    monkeypatch.setattr(streamer, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy2))
    job_recovery.resume_job(str(gfile), "COM1", checkpoint=str(ckpt))
    # file already finished, nothing to send
    assert dummy2.written == []
