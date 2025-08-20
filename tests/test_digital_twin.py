import pytest

from cam_slicer.digital_twin import DigitalTwin, preview_points


def test_simulate_toolpath_no_delay(monkeypatch):
    """No sleep should occur when delay is False."""
    called = []

    def fake_sleep(_):
        called.append(True)

    import cam_slicer.digital_twin as dt

    monkeypatch.setattr(dt.time, "sleep", fake_sleep)
    twin = DigitalTwin(None)
    twin.simulate_toolpath([(0, 0, 0), (1, 0, 0)], interval=0.1, delay=False)

    assert not called


def test_preview_points_buffering():
    """Buffered points should be exposed via ``preview_points``."""
    twin = DigitalTwin("demo")
    preview_points.clear()
    twin.simulate_toolpath([(0, 0, 0), (1, 1, 1)], interval=0, delay=False)
    assert preview_points == [(0, 0, 0), (1, 1, 1)]
    assert twin.get_live_state()["x"] == 1.0
