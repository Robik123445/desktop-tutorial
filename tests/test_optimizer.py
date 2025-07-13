import json
from pathlib import Path
from cam_slicer.ai.feedrate_optimizer import optimize_toolpath_based_on_surface
from cam_slicer.ai.gcode_cleaner import (
    optimize_toolpath,
    optimize_toolpath_sequence,
    optimize_toolpath_sequence_tsp,
)
from cam_slicer.ai.trajectory_planner import smooth_toolpath_corners


def test_optimize_toolpath_based_on_surface(tmp_path):
    """Test adjusting feedrate based on surface analysis."""
    gcode_file = tmp_path / "path.gcode"
    gcode_file.write_text("G1 X0 Y0 Z-1 F100\nG1 X1 Y0 Z-1 F100")

    hmap = tmp_path / "map.json"
    data = {"points": [{"x": 0, "y": 0, "z": 0}, {"x": 1, "y": 0, "z": 1}]}
    hmap.write_text(json.dumps(data))

    out_path = optimize_toolpath_based_on_surface(str(gcode_file), str(hmap), min_depth=-0.5)
    result = Path(out_path).read_text().splitlines()
    assert result[0].endswith("F80.000")


def test_optimize_toolpath():
    """Test arc insertion and air move removal."""
    path = [(0, 0, 1), (0.05, 0, 1), (1, 0, 0), (1, 1, 0)]
    segments = optimize_toolpath(path, angle_threshold=45, distance_threshold=0.1)
    # first short air segment removed
    assert segments[0] == [(0, 0, 1), (1, 0, 0)]
    # arc inserted at the corner
    assert len(segments[1]) == 3


def test_optimize_toolpath_sequence():
    """Test reordering segments to minimize travel."""
    paths = [
        [(0, 0, 0), (1, 0, 0)],
        [(5, 0, 0), (5, 1, 0)],
        [(2, 0, 0), (2, 1, 0)],
    ]
    result = optimize_toolpath_sequence(paths)
    assert result[0] == paths[0]
    assert result[2] == paths[2]
    assert result[-1] == paths[1]
    # inserted rapid move between path0 end and path2 start
    assert result[1] == [paths[0][-1], paths[2][0]]


def test_optimize_toolpath_sequence_tsp():
    """Test 2-opt ordering for complex jobs."""
    paths = [
        [(0, 0, 0), (1, 0, 0)],
        [(5, 5, 0), (6, 5, 0)],
        [(2, 2, 0), (2, 3, 0)],
        [(8, 0, 0), (8, 1, 0)],
    ]
    result = optimize_toolpath_sequence_tsp(paths)
    assert len(result) > len(paths)
    # ensure first rapid connects path0 to nearest path
    assert result[1][0] == paths[0][-1]


def test_smooth_toolpath_corners():
    """Test that sharp corners are replaced by arc instructions."""
    path = [(0, 0, 0), (1, 0, 0), (1, 1, 0)]
    smoothed = smooth_toolpath_corners(path, angle_threshold=45)
    assert smoothed[0] == (0, 0, 0)
    assert smoothed[1][0] == "arc"
    params = smoothed[1][1]
    assert params["start"] == (0, 0, 0)
    assert params["end"] == (1, 1, 0)
    assert "radius" in params


def test_optimize_toolpath_large():
    """Handle large toolpaths without failure."""
    path = [(x, 0.0, 0.0) for x in range(1000)]
    result = optimize_toolpath(path, angle_threshold=30)
    assert result
    assert isinstance(result[0], list) or isinstance(result[0], tuple)
