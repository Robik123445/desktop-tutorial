from cam_slicer.robotics.joint_load_predictor import JointLoadPredictor


def test_predict_load():
    """Predictor should return load values per joint."""
    pred = JointLoadPredictor()
    loads = pred.predict_load([10.0, -20.0], [1.0, 2.0])
    assert len(loads) == 2
    assert loads[0] > 0 and loads[1] > 0
