import sys, os
sys.path.insert(0, os.path.abspath("."))

import pytest
ezdxf = pytest.importorskip('ezdxf')
np = pytest.importorskip('numpy')
trimesh = pytest.importorskip('trimesh')
cv2 = pytest.importorskip('cv2')

from cam_slicer.importers import (
    import_dwg,
    import_obj,
    import_stl,
    analyze_stl,
    mesh_simplify,
    repair_mesh,
    import_mesh_parametric,
    analyze_dxf,
)


def test_import_dwg(tmp_path):
    """Test importing a simple DWG file."""
    doc = ezdxf.new()
    msp = doc.modelspace()
    msp.add_line((0, 0), (1, 0))
    dxf = tmp_path / "demo.dxf"
    doc.saveas(dxf)

    pts = import_dwg(str(dxf))
    assert (0.0, 0.0) in pts
    assert (1.0, 0.0) in pts


def test_import_obj(tmp_path):
    """Test importing an OBJ mesh into point data."""
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    faces = np.array([[0, 1, 2]])
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
    objf = tmp_path / "mesh.obj"
    mesh.export(objf)

    pts = import_obj(str(objf))
    assert len(pts) == 3

import numpy as np
import cv2

from cam_slicer.importers import import_and_vectorize_image, import_svg


def test_vectorize_image_outline(tmp_path):
    """Test vectorizing an image in outline mode."""
    img = np.zeros((10, 10), dtype=np.uint8)
    cv2.rectangle(img, (2, 2), (7, 7), 255, -1)
    f = tmp_path / "img.png"
    cv2.imwrite(str(f), img)

    paths = import_and_vectorize_image(str(f), mode="outline")
    assert paths


def test_import_svg(tmp_path):
    """Test importing a simple SVG polyline."""
    svg_content = '<svg height="10" width="10"><polyline points="0,0 5,0 5,5"/></svg>'
    f = tmp_path / "shape.svg"
    f.write_text(svg_content)
    paths = import_svg(str(f))
    assert paths[0][0] == (0.0, 0.0)


def test_analyze_dxf(tmp_path):
    """Analyze a minimal DXF file."""
    doc = ezdxf.new()
    doc.modelspace().add_line((0, 0), (1, 0))
    path = tmp_path / "a.dxf"
    doc.saveas(path)
    report = analyze_dxf(str(path))
    assert report["entities"] == 1


def test_analyze_and_simplify_stl(tmp_path):
    """Analyze and simplify a small STL mesh."""
    mesh = trimesh.creation.icosphere(subdivisions=1, radius=1.0)
    stl = tmp_path / "mesh.stl"
    mesh.export(stl)

    report = analyze_stl(str(stl))
    assert report["polygons"] == len(mesh.faces)

    out = tmp_path / "simplified.stl"
    mesh_simplify(str(stl), target_polycount=10, out_path=str(out))
    simple = trimesh.load(out, force="mesh")
    assert len(simple.faces) <= 10


def test_repair_and_parametric(tmp_path):
    """Repair a mesh and apply parametric scaling."""
    mesh = trimesh.creation.box()
    mesh.delete_faces([0])  # create a hole
    stl = tmp_path / "broken.stl"
    mesh.export(stl)

    report = repair_mesh(str(stl), out_path=str(stl))
    assert report["holes_filled"] >= 1

    fixed = import_mesh_parametric(str(stl), operations=[{"scale": 2.0}])
    assert fixed.bounds[1][0] - fixed.bounds[0][0] > 1.9
