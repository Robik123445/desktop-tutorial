diff --git a/tests/test_api_server.py b/tests/test_api_server.py
index 9faa04ef59a89e470ecb53fa868a186bb975ea74..030418ad4efd3bb938345f999a1e37936fbe20d3 100644
--- a/tests/test_api_server.py
+++ b/tests/test_api_server.py
@@ -71,25 +71,32 @@ def test_stream_robotic_error(monkeypatch):
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
+
+
+def test_get_central_log():
+    """Central log file should be returned as text."""
+    resp = client.get("/logs/central.log", headers={"X-Access-Token": "testtoken"})
+    assert resp.status_code == 200
+    assert "INFO" in resp.text
