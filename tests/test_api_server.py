diff --git a/tests/test_api_server.py b/tests/test_api_server.py
index 030418ad4efd3bb938345f999a1e37936fbe20d3..c29f2d66918e9461b070b8079742da94b0d6793b 100644
--- a/tests/test_api_server.py
+++ b/tests/test_api_server.py
@@ -78,25 +78,36 @@ def test_stream_robotic_error(monkeypatch):
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
 
 
 def test_get_central_log():
     """Central log file should be returned as text."""
     resp = client.get("/logs/central.log", headers={"X-Access-Token": "testtoken"})
     assert resp.status_code == 200
     assert "INFO" in resp.text
+
+
+def test_list_serial_ports(monkeypatch):
+    """Serial ports endpoint returns a list of ports."""
+    monkeypatch.setattr(
+        "cam_slicer.api_server.list_available_ports",
+        lambda: ["/dev/ttyUSB0", "/dev/ttyACM0"],
+    )
+    resp = client.get("/ports", headers={"X-Access-Token": "testtoken"})
+    assert resp.status_code == 200
+    assert resp.json() == ["/dev/ttyUSB0", "/dev/ttyACM0"]
