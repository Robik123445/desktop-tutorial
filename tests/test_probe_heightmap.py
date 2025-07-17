from cam_slicer.probing import probe_heightmap
from cam_slicer.utils import ZMap


def test_probe_heightmap_simulated(tmp_path):
    # Simulovaná sonda – jednoduchá funkcia: výška = x + y
    def fake_probe(x, y):
        return x + y

    out_path = tmp_path / "hm.json"
    zmap = probe_heightmap((0, 1), (0, 1), step=1.0, probe_func=fake_probe, save_path=out_path)

    assert isinstance(zmap, ZMap)
    assert len(zmap.points) == 4  # 2x2 grid
    assert out_path.exists()
    assert zmap.get_offset(1, 0) == 1.0  # x + y = 1 + 0
