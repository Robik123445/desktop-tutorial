from cam_slicer.utils import ZMap


def test_zmap_bilinear_interpolation():
    """Test bilinear interpolation in ZMap between four points in a square."""
    points = [
        (0, 0, 0.0),
        (1, 0, 1.0),
        (0, 1, 1.0),
        (1, 1, 2.0),
    ]
    zmap = ZMap(points)
    val = zmap.get_offset(0.5, 0.5)
    assert abs(val - 1.0) < 1e-6  # Expected average of corner Zs
