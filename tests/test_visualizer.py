import os
import sys
import pytest

sys.path.insert(0, os.path.abspath("."))

from cam_slicer.visualizer import (
    parse_gcode,
    preview_gcode,
    render_surface_and_toolpath,
    align_toolpath_to_surface,
    simulate_motion_over_surface,
    analyze_toolpath_vs_surface,
    export_preview_image,
    backplot_gcode,
)


def test_parse_gcode_basic():
    gcode = ["G1 X1 Y2 Z-0.5", "G0 X0 Y0"]
    pts = parse_gcode(gcode)
    assert pts[0] == (1.0, 2.0, -0.5)
    assert pts[1][0] == 0.0 and pts[1][1] == 0.0


def test_parse_gcode_arc():
    gcode = ["G0 X0 Y0", "G2 X1 Y1 I0 J1"]
    pts = parse_gcode(gcode)
    from pytest import approx
    assert pts[-1] == approx((1.0, 1.0, 0.0))
    assert len(pts) > 3


def test_preview_gcode():
    gcode = ["G1 X0 Y0 Z0", "G1 X1 Y0 Z-1"]
    result = preview_gcode(gcode, laser_mode=False)
    assert result is not None


def test_render_surface_and_toolpath(tmp_path):
    import json
    hmap = tmp_path / "map.json"
    hmap.write_text(json.dumps({
        "points": [
            {"x": 0, "y": 0, "z": 0},
            {"x": 1, "y": 0, "z": 0},
            {"x": 0, "y": 1, "z": 0},
        ]
    }))
    gcode_file = tmp_path / "path.gcode"
    gcode_file.write_text("G1 X0 Y0 Z0\nG1 X1 Y0 Z0")
    fig = render_surface_and_toolpath(str(hmap), str(gcode_file))
    assert fig is not None


def test_align_toolpath_to_surface(tmp_path):
    import json
    surface = tmp_path / "surf.json"
    surface.write_text(json.dumps({
        "points": [{"x": 0, "y": 0, "z": 1}, {"x": 1, "y": 0, "z": 1}]
    }))
    toolpath = [(0, 0, 0), (1, 0, 0)]
    aligned = align_toolpath_to_surface(toolpath, str(surface))
    assert aligned[0][2] == 1.0


# Extra vizuálne funkcie – ak máš matplotlib alebo PIL
@pytest.mark.skip(reason="Optional visual test – enable if needed")
def test_export_preview_image(tmp_path):
    gcode = ["G1 X0 Y0 Z0", "G1 X1 Y0 Z0"]
    path = tmp_path / "preview.png"
    export_preview_image(gcode, path)
    assert path.exists()
