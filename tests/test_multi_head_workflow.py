from cam_slicer.utils import assign_hybrid_operations, schedule_multi_head_job
from cam_slicer.core.header_footer import ControllerConfig


def test_schedule_multi_head_job_with_macros(tmp_path):
    """G-code should include tool change macros when switching heads."""
    paths = [[(0, 0, -1)], [(1, 0, -1)]]
    ops = {0: "cut", 1: "engrave"}
    heads = ["router", "laser"]
    segments, _ = assign_hybrid_operations(paths, heads, ops)
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    macros = {"router": "START_SPINDLE", "laser": "END_SPINDLE"}
    gcode = schedule_multi_head_job(segments, cfg, head_macros=macros)
    joined = "\n".join(gcode)
    assert "grbl-header" in gcode[0]
    assert "M3 S1000" in joined  # start spindle macro
    assert "M5" in joined  # end spindle macro
    assert gcode[-1] == "grbl-footer"

def test_prepare_hybrid_job(tmp_path):
    """prepare_hybrid_job should assign heads and return combined G-code."""
    paths = [[(0,0,-1)], [(1,0,-1)], [(2,0,-1)]]
    ops = {0:"cut",1:"engrave",2:"cut"}
    heads = ["router","laser"]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    macros = {"router":"TOOLCHANGE_router","laser":"TOOLCHANGE_laser"}
    from cam_slicer.utils import prepare_hybrid_job
    gcode, summary = prepare_hybrid_job(paths, cfg, ops, heads, head_macros=macros)
    assert summary["router"] == 2
    assert summary["laser"] == 1
    joined = "\n".join(gcode)
    assert "T1" in joined and "T2" in joined

