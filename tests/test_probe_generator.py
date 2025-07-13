import json

from cam_slicer.sensors import generate_heightmap, export_heightmap_to_json


def test_generate_and_export_heightmap(tmp_path):
    """Test probing grid points and exporting heightmap to JSON."""
    def probe(x, y):
        return x + y

    out = tmp_path / "map.json"
    heightmap = generate_heightmap((0, 1), (0, 1), 1, probe)
    assert len(heightmap) == 4
    export_heightmap_to_json(heightmap, out)
    assert out.exists()
    data = json.loads(out.read_text())
    assert len(data["points"]) == 4


def test_generate_heightmap_with_density():
    """Ensure density parameter calculates grid resolution."""
    def probe(x, y):
        return x + y

    hm = generate_heightmap((0, 1), (0, 1), step=None, probe_func=probe, density=3)
    # 3 points per axis -> 9 points total
    assert len(hm) == 9
