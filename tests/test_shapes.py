import pytest
from cam_slicer.shapes import (
    generate_adaptive_path,
    generate_spiral_helix,
    morph_between_curves,
)
from cam_slicer.ai.adaptive_steps import compute_adaptive_steps


def test_generate_adaptive_path_basic():
    """Test generating a simple adaptive toolpath."""
    boundary = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    path = generate_adaptive_path(boundary, depth=-2, stepdown=1, stepover=0.5)
    assert path, "no segments generated"
    first = path[0]
    assert first[0][2] == 0.0
    assert path[-1][-1][2] == -2

def test_generate_adaptive_path_zigzag():
    """Ensure zigzag mode returns line segments."""
    boundary = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    path = generate_adaptive_path(boundary, depth=-1, stepdown=1, stepover=0.5, mode="zigzag")
    assert path
    assert len(path[0]) == 2


def test_generate_adaptive_path_experimental():
    """Adaptive mode tweaks stepdown for surface following."""
    boundary = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    normal = generate_adaptive_path(boundary, depth=-2, stepdown=1, stepover=0.5)
    adaptive = generate_adaptive_path(
        boundary, depth=-2, stepdown=1, stepover=0.5, adaptive_mode=True
    )
    assert len(adaptive) >= len(normal)

def test_generate_spiral_helix_basic():
    """Generate a basic helical path descending to depth."""
    path = generate_spiral_helix((0.0, 0.0), radius=1.0, depth=-2.0, pitch=0.5)
    assert path
    assert path[-1][2] == -2.0


def test_morph_between_curves_basic():
    """Interpolates curves between two boundaries."""
    a = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    b = [(0.0, 1.0, 0.0), (1.0, 1.0, 0.0)]
    paths = morph_between_curves([a, b], layers=1)
    assert len(paths) == 3
    mid = paths[1]
    assert mid[0][1] == pytest.approx(0.5)


def test_compute_adaptive_steps_reduces_values():
    """Adaptive step calculation lowers values for harder materials."""
    sd, so = compute_adaptive_steps(1.0, 0.5, material_hardness=2.0, tool_load=0.8)
    assert sd < 1.0
    assert so < 0.5


def test_generate_adaptive_path_material_params():
    """Path generation accepts material parameters for adaptive mode."""
    boundary = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    path = generate_adaptive_path(
        boundary,
        depth=-2,
        stepdown=1,
        stepover=0.5,
        adaptive_mode=True,
        material_hardness=2.0,
        geometry_factor=0.6,
        tool_load=0.7,
    )
    assert path[-1][-1][2] <= -2
