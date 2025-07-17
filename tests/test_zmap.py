from cam_slicer.utils import ZMap

def test_zmap_bilinear_interpolation():
    """Ensure ZMap interpolates between probe points."""
    points = [
        (0, 0, 0.0),
        (1, 0, 1.0),
        (0, 1, 1.0),
        (1, 1, 2.0),
    ]
    zmap = ZMap(points)
    val = zmap.get_offset(0.5, 0.5)
    assert abs(val - 1.0) < 1e-6
