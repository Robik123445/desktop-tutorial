import sys

from cam_slicer.robotics.sensor_stream import SensorStream


class DummySerial:
    def __init__(self, lines):
        self.lines = [l.encode() for l in lines]
        self.index = 0

    def readline(self):
        if self.index >= len(self.lines):
            return b""
        line = self.lines[self.index]
        self.index += 1
        return line

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        pass


def test_sensor_stream_parses_lines(monkeypatch):
    """SensorStream should parse incoming lines into poses."""
    data = ["1,2,3,4,5,6"]
    dummy = DummySerial(data)

    def fake_serial(*args, **kwargs):
        return dummy

    monkeypatch.setitem(sys.modules, 'serial', __import__('types').SimpleNamespace(Serial=fake_serial))
    poses = []
    stream = SensorStream('COM1', mode='serial', callback=poses.append)
    stream.start()
    stream.stop()
    assert poses and poses[0] == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
