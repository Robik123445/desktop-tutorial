from cam_slicer.ai.chip_coolant import ChipCoolantAdvisor, apply_chip_evacuations


def test_analyze_evacuations():
    """Advisor suggests coolant and evacuation moves."""
    tp = [(0, 0, 0), (30, 0, 0), (60, 0, 0)]
    advisor = ChipCoolantAdvisor("steel", safe_height=5.0, segment_threshold=40.0)
    result = advisor.analyze(tp, feedrate=100.0)
    assert result["coolant"] == "flood"
    assert result["evac_moves"]  # at least one move
    new_tp = apply_chip_evacuations(tp, result["evac_moves"])
    assert len(new_tp) > len(tp)
