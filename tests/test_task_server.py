import types
import pytest
pytest.importorskip("fastapi")
from fastapi.testclient import TestClient
from cam_slicer.task_server import create_app

app = create_app()
client = TestClient(app)


def test_run_vizualizuj(monkeypatch):
    monkeypatch.setattr("cam_slicer.run_live_detection", lambda: None, raising=False)
    resp = client.post("/run/vizualizuj")
    assert resp.status_code == 200
    assert resp.json()["detail"] == "ok"


def test_run_unknown():
    resp = client.post("/run/unknown")
    assert resp.status_code == 400
