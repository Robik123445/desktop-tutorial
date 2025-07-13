from cam_slicer.utils import blend_additive_subtractive
from cam_slicer.core.header_footer import ControllerConfig


def test_blend_additive_subtractive():
    """Hybrid blend should insert tool change macros for print and mill."""
    print_tp = [[(0, 0, 0), (1, 0, 0)]]
    mill_tp = [[(0, 0, -1), (1, 0, -1)]]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    macros = {"print_head": "TOOLCHANGE_print_head", "router": "TOOLCHANGE_router"}
    gcode, summary = blend_additive_subtractive(print_tp, mill_tp, cfg, head_macros=macros)
    joined = "\n".join(gcode)
    assert "T4 M6" in joined  # print head macro
    assert summary == {"print_head": 1, "router": 1}
    assert gcode[0].startswith("grbl-header")
    assert gcode[-1] == "grbl-footer"
