from cam_slicer.ai.hsm import optimize_high_speed_toolpath
from cam_slicer.core.gcode_export import ControllerConfig


def test_optimize_high_speed_toolpath_basic():
    """HSM optimizer smooths corners and applies lookahead feedrates."""
    path = [(0, 0, 0), (10, 0, 0), (10, 5, 0)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    gcode = optimize_high_speed_toolpath(path, cfg, feedrate=1000, lookahead=2)
    assert any(line.startswith("G3") or line.startswith("G2") for line in gcode)
    assert any("F" in line for line in gcode)

