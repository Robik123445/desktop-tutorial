import types
from cam_slicer.sender import feedback_streamer


def test_parse_xyz():
    assert feedback_streamer._parse_xyz("G1 X1 Y2 Z-3") == (1.0, 2.0, -3.0)


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


def test_feedback_threshold(monkeypatch, tmp_path):
    gfile = tmp_path / "cmd.gcode"
    gfile.write_text("G1 X0 Y0 Z0\nG1 X1 Y0 Z-1\nG1 X2 Y0 Z-1")
    dummy = DummySerial()
    monkeypatch.setattr(feedback_streamer, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy))
    import cam_slicer.sender.serial_streamer as streamer
    monkeypatch.setattr(streamer, "serial", types.SimpleNamespace(Serial=lambda *a, **kw: dummy))

    def measure(x, y):
        return -0.1 if x >= 1 else 0.0

    feedback_streamer.stream_gcode_with_feedback(
        str(gfile), "COM1", measure, threshold=1.0
    )

    assert dummy.written == [
        "G1 X0 Y0 Z0.000",
        "G1 X1 Y0 Z-1.000",
        "G1 X2 Y0 Z-1.000",
    ]
