from pathlib import Path
from cam_slicer.ai.force_prediction import ForcePredictor, simulate_force_profile


def test_predict_forces():
    """Predict cutting forces for path segments."""
    tp = [(0, 0, 0), (1, 0, 0), (1, 1, 0)]
    pred = ForcePredictor("plywood")
    forces = pred.predict_forces(tp, feedrate=100.0, depth=1.0)
    assert len(forces) == 2
    assert forces[0] > 0


def test_simulate_force_profile(tmp_path):
    """Simulation saves overlay and returns warnings for high forces."""
    import pytest
    pytest.importorskip("matplotlib")
    tp = [(0, 0, 0), (1, 0, 0), (1, 1, 0)]
    out_file = tmp_path / "force.png"
    out, warnings = simulate_force_profile(tp, 100.0, 1.0, output_file=str(out_file), force_limit=0.01)
    assert Path(out).exists()
    assert warnings

