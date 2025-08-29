from fastapi.testclient import TestClient

from cam_slicer.api_server import create_app
from cam_slicer.vision import camera_router, camera_config as cc


class DummyCap:
    def __init__(self, idx: int) -> None:
        self.idx = idx

    def isOpened(self) -> bool:
        return self.idx == 1

    def read(self):
        return (self.idx == 1, None)

    def release(self) -> None:
        pass


class DummyCV2:
    def VideoCapture(self, idx: int) -> DummyCap:  # type: ignore
        return DummyCap(idx)


def test_camera_endpoints(monkeypatch, tmp_path):
    monkeypatch.setattr(camera_router, "cv2", DummyCV2())
    monkeypatch.setattr(cc, "CONFIG_PATH", tmp_path / "camera.json")
    client = TestClient(create_app())

    resp = client.get("/vision/cameras")
    assert resp.status_code == 200
    data = resp.json()
    assert len(data["available"]) == 11

    resp = client.post("/vision/camera/select", json={"index": 1})
    assert resp.status_code == 200
    assert cc.get_current_index() == 1

    resp = client.get("/vision/camera/current")
    assert resp.json()["index"] == 1

    resp = client.post("/vision/camera/select", json={"index": 2})
    assert resp.status_code == 400
