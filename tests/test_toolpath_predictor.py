from cam_slicer.ai.toolpath_predictor import ToolpathPredictor


def test_predict_time_and_wear():
    """Predict machining time and wear."""
    tp = [(0, 0, 0), (1, 0, 0), (1, 1, 0)]
    pred = ToolpathPredictor(wear_factor=0.01)
    stats = pred.predict(tp, feedrate=60)
    assert stats["time_sec"] == 2.0  # 2 mm at 1 mm/s
    assert abs(stats["tool_wear"] - 0.02) < 1e-6
