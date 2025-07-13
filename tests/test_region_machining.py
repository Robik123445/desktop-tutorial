from cam_slicer.utils.region_machining import clip_toolpath_to_regions, Polygon
import pytest


def test_clip_toolpath_basic():
    """Toolpath points outside regions are removed."""
    toolpath = [(0, 0, -1), (2, 0, -1), (2, 2, -1), (0, 2, -1)]
    region = [(0.5, 0.5), (1.5, 0.5), (1.5, 1.5), (0.5, 1.5)]
    if Polygon is None:
        pytest.skip("shapely not available")
    clipped = clip_toolpath_to_regions(toolpath, [region])
    assert all(0.5 <= x <= 1.5 and 0.5 <= y <= 1.5 for x, y, _ in clipped)
