diff --git a/tests/test_api_server.py b/tests/test_api_server.py
index 553db604332cfcff8dc4d6722e7897951b536e50..9faa04ef59a89e470ecb53fa868a186bb975ea74 100644
--- a/tests/test_api_server.py
+++ b/tests/test_api_server.py
@@ -32,56 +32,60 @@ def test_robot_optimize():
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
     lines = resp.text.splitlines()
     assert any(line.startswith("G1") or line.startswith("grbl") for line in lines)
 
 
 def test_optimize_toolpath_api():
-    """Optimize toolpath via API."""
-    payload = {"points": [[0, 0, 0], [1, 1, 0], [2, 1, 0]]}
+    """Run trajectory cleaner analyzer via API."""
+    payload = {
+        "analyzer": "Trajectory Cleaner",
+        "points": [[0, 0, 0], [1, 0, 0], [1, 0, 0], [2, 0, 0]],
+    }
     resp = client.post("/optimize", json=payload, headers={"X-Access-Token": "testtoken"})
     assert resp.status_code == 200
     data = resp.json()
-    assert len(data) == len(payload["points"])
+    assert data["removed"] == 1
+    assert len(data["points"]) == 3
 
 
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
