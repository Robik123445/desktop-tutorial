import json

from cam_slicer.vision import camera_config as cc


def test_set_get_current_index(tmp_path, monkeypatch):
    cfg = tmp_path / "camera.json"
    monkeypatch.setattr(cc, "CONFIG_PATH", cfg)
    cc.set_current_index(2)
    assert cc.get_current_index() == 2
    cc.set_current_index(5)
    data = json.loads(cfg.read_text())
    assert data["index"] == 5
