from pathlib import Path
from cam_slicer.ai.material_simulator import MaterialSimulator


def test_simulate_creates_image(tmp_path):
    """Simulation saves preview image."""
    tp = [(0,0,0), (1,0,0), (1,1,0)]
    sim = MaterialSimulator("plywood")
    out = tmp_path / "sim.png"
    result = sim.simulate(tp, output_file=str(out))
    assert Path(result).exists()
