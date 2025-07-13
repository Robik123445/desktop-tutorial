import os, sys
import pytest

sys.path.insert(0, os.path.abspath("."))

pytest.importorskip("fastapi")
pytest.importorskip("httpx")
from fastapi.testclient import TestClient
from cam_slicer.api_server import create_app

os.environ["API_TOKEN"] = "testtoken"
app = create_app()
client = TestClient(app)


def test_list_plugins():
    """Plugins endpoint lists available plugins."""
    resp = client.get("/plugins", headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    assert any(p["name"] == "reverse_path" for p in resp.json())


def test_run_plugin():
    """Running reverse_path plugin via API."""
    payload = {"toolpath": [[0, 0, 0], [1, 1, 1]]}
    resp = client.post("/plugins/reverse_path", json=payload, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    assert resp.json()[0] == [1, 1, 1]


def test_robot_optimize():
    """Optimize a robotic trajectory via API."""
    payload = {
        "points": [[0, 0, 0], [2, 0, 0]],
        "profile": {
            "name": "p",
            "link_lengths": [1, 1],
            "joint_types": ["revolute", "revolute"],
            "joint_limits": [[-180, 180], [-180, 180]],
        },
    }
    resp = client.post("/robot_optimize", json=payload, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    data = resp.json()
    assert "points" in data and "warnings" in data

def test_export_gcode():
    """Export G-code via API."""
    payload = {"points": [[0, 0, 0], [1, 0, -1]]}
    resp = client.post("/export", json=payload, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    assert any(line.startswith("G1") or line.startswith("grbl") for line in resp.json())


def test_optimize_toolpath_api():
    """Optimize toolpath via API."""
    payload = {"points": [[0, 0, 0], [1, 1, 0], [2, 1, 0]]}
    resp = client.post("/optimize", json=payload, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    data = resp.json()
    assert len(data) == len(payload["points"])


def test_stream_robotic_error(monkeypatch):
    """Streaming errors return HTTP 500."""
    def fail(*a, **kw):
        raise RuntimeError("port error")
    monkeypatch.setattr("cam_slicer.api_server.stream_robotic_toolpath", fail)
    payload = {
        "points": [[0, 0, 0]],
        "port": "COM1",
        "baud": 115200,
        "profile": {"name": "basic", "link_lengths": [], "joint_types": [], "joint_limits": []},
    }
    resp = client.post("/stream_robotic", json=payload, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 500
