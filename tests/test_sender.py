import types
import pytest
from cam_slicer.sender import serial_stream, serial_streamer, grbl_streamer, live_gcode_streamer
from cam_slicer.sender.serial_stream import stream_gcode_live
from cam_slicer.sender.serial_streamer import stream_gcode_to_grbl
from cam_slicer.sender.grbl_streamer import stream_gcode_interactive, StreamController
from cam_slicer.sender.live_gcode_streamer import LiveGcodeStreamer

class DummySerial:
    def __init__(self):
        self.written = []
    def write(self, data):
        self.written.append(data.decode().strip())
    def readline(self):
        return b"ok\n"
    def close(self):
        pass
    def __enter__(self):
        return self
    def __exit__(self, exc_type, exc, tb):
        pass

def test_stream_gcode_live(monkeypatch, tmp_path):
    """Test streaming G-code over serial in live mode."""
    gfile = tmp_path / "cmd.gcode"
    gfile.write_text("G0 X0\nG1 X1")
    dummy = DummySerial()
    monkeypatch.setattr(serial_stream, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy))
    stream_gcode_live(str(gfile), "COM1", 115200)
    assert dummy.written == ["G0 X0", "G1 X1"]


def test_stream_gcode_to_grbl(monkeypatch, tmp_path):
    """Test blocking streaming to a GRBL controller."""
    gfile = tmp_path / "cmd.gcode"
    gfile.write_text("G0 X0\nG1 X1")
    dummy = DummySerial()
    monkeypatch.setattr(serial_streamer, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy))
    stream_gcode_to_grbl(str(gfile), "COM1")
    assert dummy.written == ["G0 X0", "G1 X1"]


def test_stream_gcode_to_grbl_error(monkeypatch, tmp_path):
    """Test error handling when GRBL returns an error."""
    gfile = tmp_path / "cmd.gcode"
    gfile.write_text("G0 X0\nG1 X1")

    class ErrorSerial(DummySerial):
        def __init__(self):
            super().__init__()
            self.step = 0
        def readline(self):
            self.step += 1
            if self.step == 1:
                return b"error:1\n"
            return b"ok\n"

    err_dummy = ErrorSerial()
    monkeypatch.setattr(serial_streamer, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: err_dummy))
    with pytest.raises(RuntimeError):
        stream_gcode_to_grbl(str(gfile), "COM1")


def test_stream_gcode_interactive(monkeypatch, tmp_path):
    """Test interactive streaming with pause/resume support."""
    gfile = tmp_path / "cmd.gcode"
    gfile.write_text("G0 X0\nG1 X1")
    dummy = DummySerial()
    monkeypatch.setattr(grbl_streamer, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy))
    monkeypatch.setattr(serial_streamer, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy))
    ctrl = StreamController()
    stream_gcode_interactive(str(gfile), "COM1", controller=ctrl)
    assert dummy.written == ["G0 X0", "G1 X1"]


def test_live_gcode_streamer(monkeypatch, tmp_path):
    """Test LiveGcodeStreamer streaming all lines."""
    gfile = tmp_path / "cmd.gcode"
    gfile.write_text("G0 X0\nG1 X1")
    dummy = DummySerial()
    monkeypatch.setattr(live_gcode_streamer, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy))
    monkeypatch.setattr(serial_streamer, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy))
    streamer = LiveGcodeStreamer(str(gfile), "COM1")
    streamer.stream()
    assert dummy.written == ["G0 X0", "G1 X1"]


def test_stream_gcode_with_feedback(monkeypatch, tmp_path):
    """Test streaming with live measurement correction."""
    gfile = tmp_path / "cmd.gcode"
    gfile.write_text("G1 X0 Y0 Z0\nG1 X1 Y0 Z-1\nG1 X2 Y0 Z-1")

    dummy = DummySerial()
    monkeypatch.setattr(serial_streamer, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy))
    from cam_slicer.sender import feedback_streamer
    monkeypatch.setattr(feedback_streamer, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy))
    stream_gcode_with_feedback = feedback_streamer.stream_gcode_with_feedback

    def measure(x, y):
        if x < 1:
            return 0.0
        if x < 2:
            return -0.1
        return -0.2

    stream_gcode_with_feedback(str(gfile), "COM1", measure, threshold=0.05)

    # third line should include corrected Z based on measurement of previous move
    assert dummy.written == [
        "G1 X0 Y0 Z0.000",
        "G1 X1 Y0 Z-1.000",
        "G1 X2 Y0 Z-1.900",
    ]
