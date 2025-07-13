from cam_slicer.workspace_scanner import scan_workspace, apply_scan_to_toolpath
from cam_slicer.utils.zmap import ZMap


def test_scan_workspace(monkeypatch):
    """Test that camera and probe results are combined."""
    called = {
        "debris": False,
        "probe": False,
    }

    def fake_debris(path):
        called["debris"] = True
        return [(0, 0, 10, 10)]

    def fake_probe(xr, yr, step, pf):
        called["probe"] = True
        return {(0, 0): 1.0}

    monkeypatch.setattr("cam_slicer.ai.debris.detect_debris", fake_debris)
    monkeypatch.setattr("cam_slicer.sensors.probe_generator.generate_heightmap", fake_probe)

    data = scan_workspace(mode="hybrid", probe_func=lambda x, y: 0)
    assert called["debris"] and called["probe"]
    assert data["debris"] == [(0, 0, 10, 10)]
    assert data["heightmap"].get_offset(0, 0) == 1.0


def test_apply_scan_to_toolpath():
    """Toolpath is adjusted using debris zones and heightmap."""
    toolpath = [(0, 0, 0), (5, 0, 0)]
    data = {
        "debris": [(4, -1, 6, 1)],
        "heightmap": ZMap([(0,0,0.5)]),
    }
    out = apply_scan_to_toolpath(toolpath, data, safe_z=1.0)
    assert out[0][2] == 0.5  # heightmap applied
    # second point filtered because inside debris zone
    assert len(out) == 1
