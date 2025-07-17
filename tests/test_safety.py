def test_toolpath_to_gcode_axis_limit():
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    transform = TransformConfig()
    tp = [(0, 0, 0), (500, 0, 0)]  # extrémne mimo rozsahu X
    with pytest.raises(ValueError, match="out of axis range"):
        toolpath_to_gcode(tp, cfg, transform, axis_range={"X": (-100, 100), "Y": (-100, 100), "Z": (-50, 50)})
