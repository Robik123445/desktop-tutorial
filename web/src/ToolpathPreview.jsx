+import React, { useMemo, useState, useEffect, useRef } from 'react';
+import log from '../log';
+
+/**
+ * Display a simple 2D preview of G-code moves.
+ * Parses X/Y coordinates from G0/G1 commands and draws them as SVG lines.
+ * Play/Pause animates through the path and logs the action to web/log.txt.
+ */
+export default function ToolpathPreview({ gcode = '' }) {
+  // Extract coordinates from G-code
+  const points = useMemo(() => {
+    const coords = [];
+    let x = 0;
+    let y = 0;
+    const re = /^(?:G0|G1)\s+.*?(?:X([-+]?\d*\.?\d+))?.*?(?:Y([-+]?\d*\.?\d+))?/i;
+    gcode.split(/\r?\n/).forEach((line) => {
+      const m = line.match(re);
+      if (m) {
+        if (m[1] !== undefined) x = parseFloat(m[1]);
+        if (m[2] !== undefined) y = parseFloat(m[2]);
+        coords.push({ x, y });
+      }
+    });
+    return coords;
+  }, [gcode]);
+
+  // Compute bounds for scaling
+  const bounds = useMemo(() => {
+    let minX = Infinity,
+      minY = Infinity,
+      maxX = -Infinity,
+      maxY = -Infinity;
+    points.forEach(({ x, y }) => {
+      minX = Math.min(minX, x);
+      minY = Math.min(minY, y);
+      maxX = Math.max(maxX, x);
+      maxY = Math.max(maxY, y);
+    });
+    if (!isFinite(minX)) {
+      minX = minY = 0;
+      maxX = maxY = 1;
+    }
+    return { minX, minY, maxX, maxY };
+  }, [points]);
+
+  const scale = 380 / Math.max(bounds.maxX - bounds.minX || 1, bounds.maxY - bounds.minY || 1);
+  const offX = -bounds.minX;
+  const offY = -bounds.minY;
+
+  const [playing, setPlaying] = useState(false);
+  const [index, setIndex] = useState(0);
+  const timerRef = useRef(null);
+
+  useEffect(() => setIndex(0), [gcode]);
+
+  useEffect(() => {
+    if (playing) {
+      log('play');
+      timerRef.current = setInterval(() => {
+        setIndex((i) => Math.min(i + 1, points.length - 1));
+      }, 300);
+    } else {
+      if (timerRef.current) clearInterval(timerRef.current);
+      log('pause');
+    }
+    return () => timerRef.current && clearInterval(timerRef.current);
+  }, [playing, points.length]);
+
+  return (
+    <div className="p-4">
+      <div className="flex items-center gap-2 mb-2">
+        <button
+          onClick={() => setPlaying((p) => !p)}
+          className="px-3 py-1 rounded bg-blue-600 text-white"
+        >
+          {playing ? 'Pause' : 'Play'}
+        </button>
+        <span className="text-sm">
+          Move {index + 1} / {points.length}
+        </span>
+      </div>
+      <svg width="400" height="400" className="bg-white border rounded">
+        {points.slice(1, index + 1).map((pt, i) => {
+          const prev = points[i];
+          const x1 = (prev.x + offX) * scale + 10;
+          const y1 = 390 - (prev.y + offY) * scale;
+          const x2 = (pt.x + offX) * scale + 10;
+          const y2 = 390 - (pt.y + offY) * scale;
+          return <line key={i} x1={x1} y1={y1} x2={x2} y2={y2} stroke="black" />;
+        })}
+      </svg>
+    </div>
+  );
+}
