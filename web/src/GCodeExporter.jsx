+import React, { useState } from 'react';
+
+/**
+ * GCodeExporter converts a toolpath JSON to G-code and optionally sends it
+ * to the selected controller. All actions are logged via `/log`.
+ */
+export default function GCodeExporter() {
+  const [toolpath, setToolpath] = useState('');
+  const [filename, setFilename] = useState('');
+  const [controller, setController] = useState('GRBL');
+  const [port, setPort] = useState('/dev/ttyUSB0');
+  const [gcode, setGcode] = useState('');
+  const [output, setOutput] = useState('');
+  const [loading, setLoading] = useState(false);
+  const [sending, setSending] = useState(false);
+
+  // Log helper
+  const postLog = (message) =>
+    fetch('/log', {
+      method: 'POST',
+      headers: { 'Content-Type': 'application/json' },
+      body: JSON.stringify({ message })
+    }).catch(() => {});
+
+  // Request gcode generation
+  const generate = async () => {
+    setLoading(true);
+    postLog(`export ${controller}`);
+    try {
+      const body = {
+        toolpath: JSON.parse(toolpath || '[]'),
+        filename,
+        controller
+      };
+      const res = await fetch('/export', {
+        method: 'POST',
+        headers: { 'Content-Type': 'application/json' },
+        body: JSON.stringify(body)
+      });
+      const data = await res.json();
+      setGcode(data.gcode || '');
+      setOutput(JSON.stringify(data, null, 2));
+    } catch (err) {
+      console.error('export failed', err);
+      setOutput('Failed to export');
+    } finally {
+      setLoading(false);
+    }
+  };
+
+  // Send gcode to machine
+  const send = async () => {
+    if (!gcode) return;
+    setSending(true);
+    postLog(`send ${port}`);
+    try {
+      const res = await fetch('/send', {
+        method: 'POST',
+        headers: { 'Content-Type': 'application/json' },
+        body: JSON.stringify({ gcode, port })
+      });
+      const data = await res.json();
+      setOutput(JSON.stringify(data, null, 2));
+    } catch (err) {
+      console.error('send failed', err);
+      setOutput('Failed to send');
+    } finally {
+      setSending(false);
+    }
+  };
+
+  return (
+    <div className="bg-gray-900 text-gray-100 p-4 rounded space-y-2">
+      <h2 className="text-lg font-semibold">G-code Exporter</h2>
+      <textarea
+        className="bg-gray-800 p-2 w-full rounded text-sm font-mono"
+        rows="4"
+        placeholder="Toolpath JSON"
+        value={toolpath}
+        onChange={(e) => setToolpath(e.target.value)}
+      />
+      <input
+        type="text"
+        className="bg-gray-800 p-2 w-full rounded"
+        placeholder="File name"
+        value={filename}
+        onChange={(e) => setFilename(e.target.value)}
+      />
+      <select
+        className="bg-gray-800 p-2 w-full rounded"
+        value={controller}
+        onChange={(e) => setController(e.target.value)}
+      >
+        <option>GRBL</option>
+        <option>Smoothie</option>
+        <option>LinuxCNC</option>
+      </select>
+      <button
+        className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded w-full"
+        onClick={generate}
+        disabled={loading}
+      >
+        {loading ? 'Generating...' : 'Generate G-code'}
+      </button>
+      {gcode && (
+        <>
+          <select
+            className="bg-gray-800 p-2 w-full rounded"
+            value={port}
+            onChange={(e) => setPort(e.target.value)}
+          >
+            <option value="/dev/ttyUSB0">/dev/ttyUSB0</option>
+            <option value="/dev/ttyACM0">/dev/ttyACM0</option>
+          </select>
+          <button
+            className="bg-green-600 hover:bg-green-700 text-white px-4 py-2 rounded w-full"
+            onClick={send}
+            disabled={sending}
+          >
+            {sending ? 'Sending...' : 'Send to Machine'}
+          </button>
+        </>
+      )}
+      {output && (
+        <pre className="bg-gray-800 p-2 rounded text-sm whitespace-pre-wrap">
+          {output}
+        </pre>
+      )}
+    </div>
+  );
+}
