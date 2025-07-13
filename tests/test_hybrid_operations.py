from cam_slicer.utils import assign_hybrid_operations


def test_assign_hybrid_operations_default():
    """Test mapping operations to default tool heads."""
    paths = [[(0, 0)], [(1, 0)], [(2, 0)]]
    ops = {0: "cut", 1: "engrave", 2: "drill"}
    heads = ["router", "laser", "engraver"]
    segments, summary = assign_hybrid_operations(paths, heads, ops)
    assert segments[0]["head"] == "router"
    assert segments[1]["head"] == "laser"
    assert segments[2]["head"] == "engraver"
    assert summary == {"router": 1, "laser": 1, "engraver": 1}


def test_assign_hybrid_operations_override():
    """Test overriding the default operation mapping."""
    paths = [[(0, 0)], [(1, 0)]]
    ops = {0: "cut", 1: "cut"}
    heads = ["router", "laser"]
    segments, summary = assign_hybrid_operations(paths, heads, ops, {"cut": "laser"})
    assert all(seg["head"] == "laser" for seg in segments)
    assert summary == {"laser": 2}
