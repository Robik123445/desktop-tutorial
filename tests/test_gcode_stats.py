+import sys, os
+sys.path.insert(0, os.path.abspath('.'))
+
+from cam_slicer.gcode_stats import parse_gcode_text, compute_gcode_stats
+
+
+def test_compute_stats():
+    gcode = "G0 X0 Y0\nG1 X1 Y1 F1000\nG1 X2 Y2 F1500"
+    lines = parse_gcode_text(gcode)
+    stats = compute_gcode_stats(lines)
+    assert stats['line_count'] == 3
+    assert stats['move_count'] == 3
+    assert stats['max_feed_rate'] == 1500
+    assert stats['bounds']['max_x'] == 2
+    assert stats['bounds']['max_y'] == 2
