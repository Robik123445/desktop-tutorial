"""Basic tests for vision_to_cnc utilities."""

from pathlib import Path

import numpy as np

from scripts import vision_to_cnc


def test_px_to_xy_and_gcode(tmp_path: Path) -> None:
    """px_to_xy maps using global affine matrix and writes G-code."""
    vision_to_cnc.M_affine = np.array([[1, 0, 0], [0, 1, 0]], dtype=float)
    x, y = vision_to_cnc.px_to_xy((10, 20))
    assert (x, y) == (10.0, 20.0)
    gfile = tmp_path / "move.gcode"
    path = vision_to_cnc.write_move_gcode(x, y, path=str(gfile))
    data = Path(path).read_text().splitlines()
    assert "G0 X10.000 Y20.000 F1500.0" in data
