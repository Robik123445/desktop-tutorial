from cam_slicer.utils import batch_process_toolpaths
from cam_slicer.core.header_footer import ControllerConfig


def test_batch_process_toolpaths():
    """Batch processing inserts macros and combines paths."""
    paths = [[(0, 0, -1)], [(1, 0, -1)]]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    gcode = batch_process_toolpaths(paths, cfg, start_macro="START_SPINDLE", between_macro="PAUSE", end_macro="END_SPINDLE")
    joined = "\n".join(gcode)
    assert "M3 S1000" in joined  # start macro
    assert "M0" in joined  # between macro
    assert "M5" in joined  # end macro
    assert gcode[0].startswith("grbl-header")
    assert gcode[-1] == "grbl-footer"
