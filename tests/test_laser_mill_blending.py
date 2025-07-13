from cam_slicer.utils import blend_laser_mill
from cam_slicer.core.header_footer import ControllerConfig


def test_blend_laser_mill():
    """Hybrid laser/mill blend inserts macros and laser power."""
    laser_tp = [[(0, 0, 0), (1, 0, 0)]]
    mill_tp = [[(0, 0, -1), (1, 0, -1)]]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    macros = {"laser": "TOOLCHANGE_laser", "router": "TOOLCHANGE_router"}
    gcode, summary = blend_laser_mill(
        laser_tp,
        mill_tp,
        cfg,
        laser_power=300.0,
        feedrate_laser=1200,
        feedrate_mill=800,
        head_macros=macros,
    )
    joined = "\n".join(gcode)
    assert "T2 M6" in joined  # laser macro
    assert "M3 S300.0" in joined
    assert summary == {"laser": 1, "router": 1}
    assert gcode[0].startswith("grbl-header")
    assert gcode[-1] == "grbl-footer"

