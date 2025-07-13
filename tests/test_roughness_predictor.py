from cam_slicer.ai.roughness_predictor import predict_surface_roughness


def test_predict_surface_roughness():
    """Compute roughness from feed and tool size."""
    rough = predict_surface_roughness(feedrate=1200, tool_diameter=6, rpm=10000)
    assert rough > 0
