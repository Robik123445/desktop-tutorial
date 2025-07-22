def test_simulate_toolpath_no_delay(monkeypatch):
    """No sleep should occur when delay is False."""
    pytest.importorskip("matplotlib")

    called = []

    def fake_sleep(_):
        called.append(True)

    import cam_slicer.digital_twin as dt

    monkeypatch.setattr(dt.time, "sleep", fake_sleep)
    twin = dt.DigitalTwin(None)
    twin.simulate_toolpath([(0, 0, 0), (1, 0, 0)], interval=0.1, delay=False)

    assert not called
