import pytest
from cam_slicer.core.model_io import import_model, export_gcode
from cam_slicer.utils import ZMap, apply_heightmap_to_gcode
from cam_slicer.ai.gcode_cleaner import optimize_toolpath
from cam_slicer.digital_twin import DigitalTwin


def test_import_model_svg(tmp_path):
    """Import SVG polyline and verify coordinates."""
    svg = tmp_path / "shape.svg"
    svg.write_text('<svg><polyline points="0,0 1,0 1,1"/></svg>')
    data = import_model(str(svg))
    assert data[0][0] == (0.0, 0.0)


def test_import_model_dxf(tmp_path):
    """Import DXF file using ezdxf."""
    ezdxf = pytest.importorskip("ezdxf")
    doc = ezdxf.new()
    doc.modelspace().add_line((0, 0), (1, 0))
    path = tmp_path / "a.dxf"
    doc.saveas(path)
    pts = import_model(str(path))
    assert (0.0, 0.0) in pts


def test_import_model_stl(tmp_path):
    """Import STL mesh using trimesh."""
    trimesh = pytest.importorskip("trimesh")
    mesh = trimesh.creation.box()
    stl = tmp_path / "mesh.stl"
    mesh.export(stl)
    pts = import_model(str(stl))
    assert len(pts) == len(mesh.vertices)


def test_export_gcode_basic():
    """Export simple linear toolpath to G-code."""
    gcode = export_gcode([(0, 0, 0), (1, 0, 0)])
    assert "G1" in gcode


def test_apply_heightmap_to_gcode():
    """Applies ZMap (heightmap) to raw G-code lines."""
    lines = ["G1 X0 Y0 Z0", "G1 X1 Y0 Z0"]
    zmap = ZMap(points=[(0, 0, 0.1), (1, 0, 0.2)])
    out = apply_heightmap_to_gcode(lines, zmap)
    assert out[0].startswith("G1 X0.000 Y0.000 Z0.100")
    assert out[1].startswith("G1 X1.000 Y0.000 Z0.200")


def test_simulate_toolpath_heightmap():
    """Simulate toolpath with DigitalTwin + heightmap."""
    twin = DigitalTwin(None)
    zmap = ZMap(points=[(0, 0, 1.0)])
    twin.simulate_toolpath([(0, 0, 0)], interval=0, heightmap=zmap)
    assert twin.get_live_state()["z"] == 1.0


def test_integration_import_optimize_export(tmp_path):
    """End-to-end test: import → optimize → export G-code."""
    svg = tmp_path / "shape.svg"
    svg.write_text('<svg><polyline points="0,0 1,0 2,0"/></svg>')
    data = import_model(str(svg))[0]
    tp = [(x, y, 0.0) for x, y in data]
    segments = optimize_toolpath(tp, angle_threshold=30)
    flat = [pt for seg in segments for pt in seg]
    gcode = export_gcode(flat)
    assert "G1" in gcode
