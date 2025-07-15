+import os, sys
+sys.path.insert(0, os.path.abspath('.'))
+
+from cam_slicer.toolpath_simulator import parse_toolpath, simulate_toolpath
+from cam_slicer.api_server import create_app
+from fastapi.testclient import TestClient
+
+
+def test_parse_toolpath():
+    gcode = 'G0 X0 Y0\nG1 X1 Y1'
+    pts = parse_toolpath(gcode)
+    assert pts == [(0.0, 0.0), (1.0, 1.0)]
+
+
+def test_simulate_endpoint():
+    os.environ['API_TOKEN'] = 'testtoken'
+    app = create_app()
+    client = TestClient(app)
+    resp = client.post('/simulate', json={'gcode': 'G0 X0 Y0\nG1 X1 Y1', 'include_plot': False}, headers={'X-Access-Token': 'testtoken'})
+    assert resp.status_code == 200
+    assert resp.json()['points'][-1] == [1.0, 1.0]
