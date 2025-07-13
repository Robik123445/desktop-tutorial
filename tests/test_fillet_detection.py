import math
import pytest
from cam_slicer.ai.fillet_detection import detect_fillet_features, generate_fillet_tracing


def test_detect_fillet_features_simple():
    """Detect fillet in simple L-shaped path."""
    path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]
    idx = detect_fillet_features(path, cutter_radius=1.0)
    assert idx == [1]


def test_generate_fillet_tracing():
    """Arc is generated for detected fillet."""
    path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]
    arcs = generate_fillet_tracing(path, cutter_radius=1.0, segments=4)
    assert len(arcs) == 1
    assert len(arcs[0]) == 5
    # ensure arc radius roughly equals cutter radius
    center = path[1]
    first_point = arcs[0][0]
    r = math.hypot(first_point[0]-center[0], first_point[1]-center[1])
    assert r == pytest.approx(1.0, rel=1e-3)
