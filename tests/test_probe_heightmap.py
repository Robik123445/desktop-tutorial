from cam_slicer.probing import probe_heightmap
from cam_slicer.utils import ZMap


def test_probe_heightmap_simulated(tmp_path):
    def fake_probe(x, y):
        return x + y

    zmap = probe_heightmap((0, 1), (0, 1), step=1.0, probe_func=fake_probe, save_path=tmp_path / "hm.json")
    assert isinstance(zmap, ZMap)
    assert len(zmap.points) == 4
    assert (tmp_path / "hm.json").exists()
    assert zmap.get_offset(1, 0) == 1
