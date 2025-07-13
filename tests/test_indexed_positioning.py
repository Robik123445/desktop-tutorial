from cam_slicer.robotics.indexed_positioning import generate_indexed_gcode
from cam_slicer.core.header_footer import ControllerConfig


def test_generate_indexed_gcode_basic():
    """Generate G-code with two orientations."""
    tp = [(0, 0, -1), (1, 0, -1), (2, 0, -1), (3, 0, -1)]
    orientations = [(0, 0, 0), (90, 0, 0)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    lines = generate_indexed_gcode(tp, orientations, cfg, segment_length=2)
    assert any(line.startswith("G0 A90.000") for line in lines)
    assert lines[0] == "grbl-header"
    assert lines[-1] == "grbl-footer"
