import time
from fastapi.testclient import TestClient
from cam_slicer import task_server

app = task_server.create_app()
client = TestClient(app)

def test_run_dummy_task():
    resp = client.post("/run/dummy_task")
    assert resp.status_code == 200
    tid = resp.json()["task_id"]
    assert tid in task_server.statuses
    # wait a bit for the worker to finish
    for _ in range(10):
        if task_server.statuses.get(tid) == "done":
            break
        time.sleep(0.5)
    assert task_server.statuses.get(tid) == "done"

def test_run_unknown_task():
    resp = client.post("/run/bad")
    assert resp.status_code == 400
