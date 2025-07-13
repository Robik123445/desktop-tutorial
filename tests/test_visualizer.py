import sys, os
sys.path.insert(0, os.path.abspath("."))

import pytest

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


def test_parse_gcode():
    """Test converting G-code lines to coordinate tuples."""
    gcode = ["G1 X1 Y2 Z-0.5", "G0 X0 Y0"]
    pts = parse_gcode(gcode)
    assert pts[0] == (1.0, 2.0, -0.5)
    assert pts[1][0] == 0.0
    assert pts[1][1] == 0.0


def test_preview_gcode():
    """Test preview rendering of G-code paths."""
    gcode = ["G1 X0 Y0 Z0", "G1 X1 Y0 Z-1"]
    result = preview_gcode(gcode, laser_mode=False)
    assert result is not None



def test_render_surface_and_toolpath(tmp_path):
    """Test 3D overlay of heightmap and toolpath."""
    import json
    hmap = tmp_path / "map.json"
    data = {"points": [{"x": 0, "y": 0, "z": 0}, {"x": 1, "y": 0, "z": 0}, {"x": 0, "y": 1, "z": 0}]}
    hmap.write_text(json.dumps(data))

    gcode_file = tmp_path / "path.gcode"
    gcode_file.write_text("G1 X0 Y0 Z0\nG1 X1 Y0 Z0")

    fig = render_surface_and_toolpath(str(hmap), str(gcode_file))
    assert fig is not None


def test_align_toolpath_to_surface(tmp_path):
    """Test aligning a tool to a scanned surface mesh."""
    import json
    pytest.importorskip("numpy")
    pytest.importorskip("trimesh")
    import numpy as np
    import trimesh

    surface = tmp_path / "surface.json"
    pts = {"points": [{"x": 0, "y": 0, "z": 0}, {"x": 1, "y": 0, "z": 0}, {"x": 2, "y": 0, "z": 0}]}
    surface.write_text(json.dumps(pts))

    vertices = np.array([[1, 0, 0], [2, 0, 0], [3, 0, 0]])
    faces = np.array([[0, 1, 2]])
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
    tool = tmp_path / "tool.stl"
    mesh.export(tool)

    aligned, sur = align_toolpath_to_surface(str(tool), str(surface))
    assert len(aligned) == 3
    assert abs(aligned[0][0] - 0.0) < 0.2


def test_simulate_motion_over_surface(tmp_path):
    """Test animation of tool motion over a surface."""
    import json
    hmap = tmp_path / "map.json"
    data = {"points": [{"x": 0, "y": 0, "z": 0}, {"x": 1, "y": 0, "z": 0}]}
    hmap.write_text(json.dumps(data))

    gcode_file = tmp_path / "path.gcode"
    gcode_file.write_text("G1 X0 Y0 Z0\nG1 X1 Y0 Z0")

    ani = simulate_motion_over_surface(str(hmap), str(gcode_file))
    assert ani is not None


def test_analyze_toolpath_vs_surface(tmp_path):
    """Test collision and clearance detection against a heightmap."""
    hmap = {
        "points": [
            {"x": 0, "y": 0, "z": 0},
            {"x": 1, "y": 0, "z": 0},
            {"x": 0, "y": 1, "z": 0},
        ]
    }

    gcode_file = tmp_path / "path.gcode"
    gcode_file.write_text("G1 X0 Y0 Z-1\nG1 X1 Y0 Z1")

    result = analyze_toolpath_vs_surface(hmap, str(gcode_file))
    assert any(r["type"] == "COLLISION" for r in result)
    assert any(r["type"] == "NO_CONTACT" for r in result)


def test_export_preview_image(tmp_path):
    """Test saving a preview image from G-code."""
    pytest.importorskip("matplotlib")
    gcode_file = tmp_path / "part.gcode"
    gcode_file.write_text("G1 X0 Y0\nG1 X1 Y0")
    out = tmp_path / "preview.png"
    export_preview_image(str(gcode_file), str(out))
    assert out.exists() and out.stat().st_size > 0


def test_backplot_gcode(tmp_path):
    """Test animation creation from a G-code file."""
    pytest.importorskip("matplotlib")
    gcode_file = tmp_path / "path.gcode"
    gcode_file.write_text("G1 X0 Y0 Z0\nG1 X1 Y0 Z0")
    ani = backplot_gcode(str(gcode_file))
    assert ani is not None
