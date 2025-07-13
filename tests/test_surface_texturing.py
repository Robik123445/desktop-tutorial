import pytest

from cam_slicer.ai.surface_texturing import generate_surface_texture

try:
    import trimesh  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    trimesh = None


def test_generate_surface_texture():
    """Toolpaths are generated for stippling pattern."""
    if trimesh is None:
        pytest.skip("trimesh missing")
    mesh = trimesh.creation.box(extents=(1, 1, 1))
    paths = generate_surface_texture(mesh, pattern="stippling", spacing=0.5)
    assert paths
