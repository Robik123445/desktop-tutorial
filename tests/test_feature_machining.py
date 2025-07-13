import pytest

try:
    import trimesh
except Exception:  # pragma: no cover - optional dependency
    trimesh = None

from cam_slicer.ai.feature_machining import generate_feature_toolpaths


def test_generate_feature_toolpaths_basic():
    """Generate toolpaths from detected features."""
    if trimesh is None:
        pytest.skip("trimesh not available")
    mesh = trimesh.creation.box(extents=(2, 2, 1))
    paths = generate_feature_toolpaths(mesh, depth=-1.0, tool_diameter=0.5)
    assert paths
