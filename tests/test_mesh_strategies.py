import pytest
trimesh = pytest.importorskip('trimesh')
from cam_slicer.core.mesh_strategies import (
    generate_roughing_paths,
    generate_finishing_paths,
    generate_carving_paths,
    generate_zlevel_roughing_paths,
    generate_zlevel_finishing_paths,
    generate_parallel_finishing_paths,
    generate_offset_finishing_paths,
    generate_high_polish_paths,
    generate_radial_finishing_paths,
    project_curves_to_surface,
    plan_mesh_operations,
)


def _cube_mesh():
    return trimesh.creation.box(extents=(1, 1, 1))


def test_generate_roughing_paths():
    mesh = _cube_mesh()
    paths = generate_roughing_paths(mesh, stepdown=0.5, stepover=0.2)
    assert paths


def test_generate_finishing_paths():
    mesh = _cube_mesh()
    paths = generate_finishing_paths(mesh)
    assert paths


def test_generate_carving_paths():
    mesh = _cube_mesh()
    paths = generate_carving_paths(mesh, depth=-1.0)
    assert paths


def test_generate_zlevel_paths():
    mesh = _cube_mesh()
    rough = generate_zlevel_roughing_paths(mesh, stepdown=0.5)
    finish = generate_zlevel_finishing_paths(mesh, stepdown=0.5)
    assert any(abs(pt[2] - 0.5) < 1e-6 for pt in rough[0])
    assert finish


def test_generate_parallel_finishing():
    """Test planar raster finishing generation."""
    mesh = _cube_mesh()
    paths = generate_parallel_finishing_paths(mesh, stepover=0.2, axis="x")
    assert paths


def test_plan_mesh_operations():
    mesh = _cube_mesh()
    ops = plan_mesh_operations(mesh, {"roughing": "tool1", "finishing": "tool2"})
    assert any(o["operation"] == "roughing" for o in ops)
    assert any(o["operation"] == "finishing" for o in ops)


def test_generate_offset_finishing_paths():
    mesh = _cube_mesh()
    paths = generate_offset_finishing_paths(mesh, offset=0.1, stepover=0.1, passes=2)
    assert paths


def test_generate_high_polish_paths():
    """Ensure ultra-fine passes are produced for mirror finishing."""
    mesh = _cube_mesh()
    paths = generate_high_polish_paths(mesh, stepover=0.05, passes=3)
    assert paths


def test_generate_radial_finishing_paths():
    mesh = _cube_mesh()
    paths = generate_radial_finishing_paths(mesh, angle_step=45)
    assert paths


def test_project_curves_to_surface():
    mesh = trimesh.creation.icosphere(radius=1.0)
    curve = [[(0.5, 0.0, 0.0), (0.5, 0.5, 0.0)]]
    projected = project_curves_to_surface(curve, mesh)
    assert projected
    assert projected[0][0][2] > 0

