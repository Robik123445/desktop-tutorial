from cam_slicer.ai.feedback_advisor import AIFeedbackAdvisor

def test_analyze_toolpath(tmp_path):
    """Test AI feedback analysis for sharp angles and collisions."""
    tp = [(0,0,0), (1,0,0), (1,1,-5), (2,1,-5)]
    report_file = tmp_path / "report.txt"
    advisor = AIFeedbackAdvisor(report_file=str(report_file), collision_z=-4)
    result = advisor.analyze_toolpath(tp, feedrate=1200)
    reasons = {r['reason'] for r in result['recommendations']}
    assert 'sharp_angle' in reasons
    assert 'potential_collision' in reasons
    assert 'repeated_cut' not in reasons
    assert result['tool_recommendation']
    assert report_file.exists()


def test_wear_and_repeated_cut(tmp_path):
    """Detect repeated cuts and tool wear recommendations."""
    tp = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)] * 3
    report_file = tmp_path / "report.txt"
    advisor = AIFeedbackAdvisor(report_file=str(report_file), wear_length=2)
    result = advisor.analyze_toolpath(tp, feedrate=1000)
    reasons = {r['reason'] for r in result['recommendations']}
    assert 'repeated_cut' in reasons
    assert result['tool_recommendation'].startswith('Replace')
