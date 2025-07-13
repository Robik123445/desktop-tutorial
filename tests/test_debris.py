from cam_slicer.ai.debris import add_debris_zones_from_image, plan_toolpath_avoiding_debris
from cam_slicer.utils import GeoFence


def test_add_debris_zones(monkeypatch):
    """Debris zones are added and reminder is triggered."""
    monkeypatch.setattr('cam_slicer.ai.debris.detect_debris', lambda p: [(0,0,1,1), (2,2,3,3)])
    fence = GeoFence()
    remind = add_debris_zones_from_image('img.jpg', fence, reminder_threshold=1)
    assert remind
    assert fence.is_inside(0.5, 0.5)


def test_plan_toolpath_avoiding_debris():
    """Toolpath filtered and raised over air-move zone."""
    fence = GeoFence(forbidden_zones=[(0,0,1,1)], air_move_zones=[(2,0,3,1)])
    tp = [(0.5, 0.5, -1), (2.5, 0.5, -1)]
    result = plan_toolpath_avoiding_debris(tp, fence, safe_z=5)
    assert result == [(2.5, 0.5, 5)]
