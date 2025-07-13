from cam_slicer.robotics.operation_mapper import map_operations


def test_map_operations_defaults():
    """SVG maps to cut, STL to mill."""
    files = ["shape.svg", "model.stl"]
    mapping = map_operations(files)
    assert mapping["shape.svg"] == "cut"
    assert mapping["model.stl"] == "mill"


def test_map_operations_override():
    """Override mapping for a file."""
    files = ["shape.svg"]
    mapping = map_operations(files, {"shape.svg": "engrave"})
    assert mapping["shape.svg"] == "engrave"
