import logging
import pytest

from cam_slicer.core.gcode_export import toolpath_to_gcode, ControllerConfig, TransformConfig
from cam_slicer.digital_twin import DigitalTwin
from cam_slicer.robotics.safety import DEFAULT_AXIS_RANGE


def test_toolpath_to_gcode_axis_limit():
    """Should raise error when toolpath exceeds configured axis limits."""
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    transform = TransformConfig()
    tp = [(0, 0, 0), (500, 0, 0)]  # X=500 exceeds limit
    with pytest.raises(ValueError):
        toolpath_to_gcode(tp, cfg, transform, axis_range={
            "X": (-100, 100),
            "Y": (-100, 100),
            "Z": (-50, 50),
        })


def test_simulate_toolpath_warning(caplog):
    """Should log a warning when Z drop is too large during simulation."""
    caplog.set_level(logging.WARNING)
    twin = DigitalTwin(None)
    tp = [(0, 0, 0), (0, 0, -20), (0, 1, -20)]
    twin.simulate_toolpath(tp, interval=0, axis_range=DEFAULT_AXIS_RANGE)
    assert any("Z drop" in rec.message for rec in caplog.records)
