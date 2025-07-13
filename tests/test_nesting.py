import pytest
from cam_slicer.utils import nest_shapes, Placement


def test_nest_shapes_basic():
    """Test simple nesting of shapes within sheet bounds."""
    shapes = [
        [(0, 0), (2, 0), (2, 1), (0, 1)],  # w=2, h=1
        [(0, 0), (1, 0), (1, 2), (0, 2)],  # w=1, h=2
    ]
    placements = nest_shapes(shapes, sheet_width=3, sheet_height=3)
    assert len(placements) == 2
    for p in placements:
        assert isinstance(p, Placement)
        x, y = p.position
        assert 0 <= x <= 3
        assert 0 <= y <= 3

