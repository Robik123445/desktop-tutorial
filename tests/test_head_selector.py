from cam_slicer.robotics.head_selector import HeadSelector


def test_default_head_selection():
    """Laser selected for cutting plywood."""
    sel = HeadSelector()
    head = sel.select_head("cut", material="plywood")
    assert head == "laser"


def test_override_head():
    """Manual override returns provided head."""
    sel = HeadSelector()
    head = sel.select_head("cut", material="steel", override="spindle")
    assert head == "spindle"
