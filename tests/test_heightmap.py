diff --git a//dev/null b/tests/test_heightmap.py
index 0000000000000000000000000000000000000000..ec18c4ea6c4cfdc548da78f80e58ff62b82dcd73 100644
--- a//dev/null
+++ b/tests/test_heightmap.py
@@ -0,0 +1,25 @@
+import os, sys
+sys.path.insert(0, os.path.abspath('.'))
+
+from cam_slicer.heightmap import HeightMap, apply_heightmap_to_gcode
+from cam_slicer.api_server import create_app
+from fastapi.testclient import TestClient
+
+
+def test_apply_heightmap():
+    gcode = "G1 X0 Y0 Z0\nG1 X1 Y0 Z-1"
+    hm = HeightMap.from_text("0,0,0.1\n1,0,0.2", fmt="csv")
+    out = apply_heightmap_to_gcode(gcode, hm)
+    lines = out.splitlines()
+    assert lines[0].endswith("Z0.100")
+    assert lines[1].endswith("Z-0.800")
+
+
+def test_heightmap_endpoint():
+    os.environ['API_TOKEN'] = 'testtoken'
+    app = create_app()
+    client = TestClient(app)
+    payload = {"gcode": "G1 X0 Y0 Z0", "heightmap": "0,0,0.5", "format": "csv"}
+    resp = client.post('/heightmap', json=payload, headers={'X-Access-Token': 'testtoken'})
+    assert resp.status_code == 200
+    assert "Z0.500" in resp.json()['gcode']
