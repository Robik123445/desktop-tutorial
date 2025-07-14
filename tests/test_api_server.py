diff --git a/tests/test_api_server.py b/tests/test_api_server.py
index a33d8364f3fc4c01d5b8cc400fab0ed4720812c2..7f1599206a8ff0c8acdeb1c2eddc785dfd0f0ba1 100644
--- a/tests/test_api_server.py
+++ b/tests/test_api_server.py
@@ -27,50 +27,51 @@ def test_run_plugin():
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
-    assert any(line.startswith("G1") or line.startswith("grbl") for line in resp.json())
+    lines = resp.text.splitlines()
+    assert any(line.startswith("G1") or line.startswith("grbl") for line in lines)
 
 
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
