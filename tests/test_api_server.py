diff --git a/tests/test_api_server.py b/tests/test_api_server.py
index 7f1599206a8ff0c8acdeb1c2eddc785dfd0f0ba1..553db604332cfcff8dc4d6722e7897951b536e50 100644
--- a/tests/test_api_server.py
+++ b/tests/test_api_server.py
@@ -53,25 +53,39 @@ def test_export_gcode():
     assert any(line.startswith("G1") or line.startswith("grbl") for line in lines)
 
 
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
+
+
+def test_send_gcode(monkeypatch):
+    """Sending raw G-code over serial returns log output."""
+    def fake_send(gcode: str, port: str):
+        assert port == "COM1"
+        return "sent: G0 X0"
+
+    monkeypatch.setattr("cam_slicer.api_server.send_gcode_over_serial", fake_send)
+    payload = {"gcode": "G0 X0", "port": "COM1"}
+    resp = client.post("/send", json=payload, headers={"X-Access-Token": "testtoken"})
+    assert resp.status_code == 200
+    data = resp.json()
+    assert data["status"] == "ok" and "sent:" in data["log"]
