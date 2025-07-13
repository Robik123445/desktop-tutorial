import logging
import time
from cam_slicer.performance import profiled


def test_profiled_warning(caplog):
    """Test that the profiled decorator logs a warning for slow calls."""
    @profiled
    def slow():
        time.sleep(0.15)

    caplog.set_level(logging.WARNING)
    slow()
    assert any("Performance warning" in r.message for r in caplog.records)
