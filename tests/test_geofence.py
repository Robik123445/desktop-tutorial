from cam_slicer.utils import GeoFence


def test_is_inside():
    """Test detection of points inside forbidden zones."""
    fence = GeoFence(forbidden_zones=[(0, 0, 1, 1), (5, 5, 6, 6)])
    assert fence.is_inside(0.5, 0.5)
    assert fence.is_inside(5.5, 5.5)
    assert not fence.is_inside(2, 2)


def test_filter_and_air_moves():
    """Test filtering and air-move adjustment of toolpath."""
    fence = GeoFence(forbidden_zones=[(0, 0, 1, 1)], air_move_zones=[(2, 0, 3, 1)])
    tp = [(0, 0, -1), (0.5, 0.5, -1), (2.5, 0.5, -1)]
    result = fence.filter_toolpath(tp)
    assert result == [(2.5, 0.5, -1)]
    adjusted = fence.adjust_toolpath_for_air_moves(result, safe_z=5)
    assert adjusted == [(2.5, 0.5, 5)]


def test_add_ai_zones():
    """Test adding forbidden and air-move zones from AI detections."""
    fence = GeoFence()
    detections = [
        {"label": "clamp", "bbox": (0, 0, 1, 1)},
        {"label": "hand", "bbox": (2, 0, 3, 1)},
    ]
    fence.add_ai_zones_from_yolo(detections)
    assert fence.is_inside(0.5, 0.5)
    fence.add_ai_zones_from_yolo(detections, zone_type="air_move")
    adjusted = fence.adjust_toolpath_for_air_moves([(2.5, 0.5, -1)], safe_z=4)
    assert adjusted[0][2] == 4
