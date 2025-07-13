import pytest
from cam_slicer.ai.rest_machining import (
    generate_rest_machining_paths,
    Polygon,
)
from cam_slicer.shapes import generate_adaptive_path


def test_generate_rest_machining_paths_basic():
    """Ensure rest machining returns cleanup paths."""
    if Polygon is None:
        pytest.skip("shapely not available")
    boundary = [(0,0), (20,0), (20,20), (0,20)]
    rough_paths = generate_adaptive_path(boundary, depth=-2, stepdown=1, stepover=5)
    rest = generate_rest_machining_paths(boundary, rough_paths, roughing_radius=5.0, rest_radius=2.0, depth=-2)
    assert rest, "No rest paths generated"
