import sys
import os
import pytest
from pathlib import Path

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
    """Running reverse_path plugin via new /plugins/run API."""
    payload = {"name": "reverse_path", "toolpath": [[0, 0, 0], [1, 1, 1]]}
    resp = client.post("/plugins/run", json=payload, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    assert resp.json()[0] == [1, 1, 1]


def test_run_plugin_named():
    """Run plugin via /plugins/{name}."""
    payload = {"toolpath": [[0, 0, 0], [1, 1, 1]]}
    resp = client.post("/plugins/reverse_path", json=payload, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    assert resp.json()[0] == [1, 1, 1]


def test_run_plugin_not_found():
    """Unknown plugin returns 404."""
    resp = client.post("/plugins/unknown", json={}, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 404


def test_run_plugin_generic_not_found():
    resp = client.post("/plugins/run", json={"name": "bad"}, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 404


def test_plugins_requires_token():
    """Endpoints require auth token."""
    resp = client.get("/plugins")
    assert resp.status_code == 403


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


def test_robot_optimize_invalid_profile():
    resp = client.post(
        "/robot_optimize",
        json={"points": [[0,0,0]], "profile": "bad"},
        headers={"X-Access-Token": "testtoken"},
    )
    assert resp.status_code == 422


def test_export_gcode():
    """Export G-code via API."""
    payload = {"points": [[0, 0, 0], [1, 0, -1]]}
    resp = client.post("/export", json=payload, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    lines = resp.text.splitlines()
    assert any(line.startswith("G1") or line.startswith("grbl") for line in lines)


def test_export_invalid():
    """Invalid payload to /export returns 422."""
    resp = client.post("/export", json={"points": "oops"}, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 422


def test_optimize_toolpath_api():
    """Run trajectory cleaner analyzer via API."""
    payload = {
        "analyzer": "Trajectory Cleaner",
        "points": [[0, 0, 0], [1, 0, 0], [1, 0, 0], [2, 0, 0]],
    }
    resp = client.post("/optimize", json=payload, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    data = resp.json()
    assert data["removed"] == 1
    assert len(data["points"]) == 3


def test_optimize_unknown_analyzer():
    resp = client.post("/optimize", json={"analyzer": "bad", "points": [[0,0,0]]}, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 400


def test_stream_robotic_success(monkeypatch):
    """Streaming returns status ok."""
    def nop(*a, **kw):
        return None
    monkeypatch.setattr("cam_slicer.api_server.stream_robotic_toolpath", nop)
    payload = {"points": [[0,0,0]], "port": "COM1", "baud": 115200, "profile": {"name": "basic"}}
    resp = client.post("/stream_robotic", json=payload, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    assert resp.json() == {"status": "ok"}


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


def test_send_gcode(monkeypatch):
    """Sending raw G-code over serial returns log output."""
    def fake_send(gcode: str, port: str):
        assert port == "COM1"
        return "sent: G0 X0"

    monkeypatch.setattr("cam_slicer.api_server.send_gcode_over_serial", fake_send)
    payload = {"gcode": "G0 X0", "port": "COM1"}
    resp = client.post("/send", json=payload, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    data = resp.json()
    assert data["status"] == "ok" and "sent:" in data["log"]


def test_send_gcode_error(monkeypatch):
    def fail(gcode: str, port: str):
        raise RuntimeError("oops")
    monkeypatch.setattr("cam_slicer.api_server.send_gcode_over_serial", fail)
    payload = {"gcode": "G0 X0", "port": "COM1"}
    resp = client.post("/send", json=payload, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    assert resp.json()["status"] == "error"


def test_get_central_log():
    """Central log file should be returned as text."""
    resp = client.get("/logs/central.log", headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    assert "INFO" in resp.text


def test_get_central_log_missing(monkeypatch):
    from pathlib import Path
    orig_exists = Path.exists
    def fake_exists(self):
        if self.as_posix().endswith("logs/central.log"):
            return False
        return orig_exists(self)
    monkeypatch.setattr(Path, "exists", fake_exists)
    resp = client.get("/logs/central.log", headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 404


def test_list_serial_ports(monkeypatch):
    """Serial ports endpoint returns a list of ports."""
    monkeypatch.setattr(
        "cam_slicer.api_server.list_available_ports",
        lambda: ["/dev/ttyUSB0", "/dev/ttyACM0"],
    )
    resp = client.get("/ports", headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    assert resp.json() == ["/dev/ttyUSB0", "/dev/ttyACM0"]


def test_ports_failure(monkeypatch):
    def fail():
        raise RuntimeError("fail")
    monkeypatch.setattr("cam_slicer.api_server.list_available_ports", fail)
    resp = client.get("/ports", headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 500

# ------------------------- NOVÝ TEST NA PROBE HEIGHTMAP -------------------------

def test_probe_heightmap_endpoint(monkeypatch):
    from cam_slicer.api_server import app
    from cam_slicer.utils.zmap import ZMap
    client = TestClient(app)

    def fake_probe(*args, **kwargs):
        return ZMap([(0, 0, 0.0)])

    monkeypatch.setattr("cam_slicer.api_server.probe_heightmap", fake_probe)
    payload = {"x_start": 0, "x_end": 1, "y_start": 0, "y_end": 1, "step": 1}
    resp = client.post("/probe_heightmap", json=payload, headers={"X-Access-Token": "testtoken"})
    assert resp.status_code == 200
    assert resp.json()["points"] == [(0, 0, 0.0)]
