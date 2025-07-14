diff --git a/web/src/GCodeExporter.jsx b/web/src/GCodeExporter.jsx
index 483ccda5a3dfb7d13917bcbe2b4bf4f304930ea5..6bb5a3f18e45f5c09aeb336882d11849b4ccd032 100644
--- a/web/src/GCodeExporter.jsx
+++ b/web/src/GCodeExporter.jsx
@@ -1,58 +1,65 @@
 import React, { useState, useEffect } from 'react';
 
 /**
  * GCodeExporter converts a toolpath JSON to G-code and optionally sends it
  * to the selected controller. All actions are logged via `/log`.
  */
 //------------------------------------------------------------------------------
 // GCodeExporter component
 //------------------------------------------------------------------------------
 export default function GCodeExporter() {
   const [toolpath, setToolpath] = useState('');
   const [filename, setFilename] = useState('');
   const [controller, setController] = useState('GRBL');
   const [ports, setPorts] = useState(['/dev/ttyUSB0', '/dev/ttyACM0']);
   const [port, setPort] = useState('/dev/ttyUSB0');
   const [gcode, setGcode] = useState('');
   const [output, setOutput] = useState('');
   const [loading, setLoading] = useState(false);
   const [sending, setSending] = useState(false);
 
-  // Fetch available serial ports once on mount. If it fails we keep defaults.
+  /**
+   * Fetch list of serial ports from the backend. When it fails we keep
+   * the current list so the user can still select a default port.
+   */
+  const fetchPorts = async () => {
+    try {
+      const res = await fetch('/ports');
+      const data = await res.json();
+      if (Array.isArray(data) && data.length) {
+        setPorts(data);
+        setPort(data[0]);
+      }
+    } catch {
+      // keep defaults on failure
+    }
+  };
+
+  // Fetch ports once on mount so the dropdown shows the latest devices.
   useEffect(() => {
-    fetch('/ports')
-      .then((res) => res.json())
-      .then((data) => {
-        if (Array.isArray(data) && data.length) {
-          setPorts(data);
-          setPort(data[0]);
-        }
-      })
-      .catch(() => {
-        /* ignore errors, use defaults */
-      });
+    fetchPorts();
   }, []);
 
   /**
    * Send a message to /log asynchronously.
    */
   const postLog = (message) =>
     fetch('/log', {
       method: 'POST',
       headers: { 'Content-Type': 'application/json' },
       body: JSON.stringify({ message })
     }).catch(() => {});
 
   /**
    * Convert the toolpath JSON to G-code by calling /export.
    */
   const generate = async () => {
     setLoading(true);
     postLog(`export ${controller}`);
     try {
       let pathData = [];
       try {
         pathData = JSON.parse(toolpath || '[]');
       } catch {
         setOutput('Invalid toolpath JSON');
         setLoading(false);
diff --git a/web/src/GCodeExporter.jsx b/web/src/GCodeExporter.jsx
index 483ccda5a3dfb7d13917bcbe2b4bf4f304930ea5..6bb5a3f18e45f5c09aeb336882d11849b4ccd032 100644
--- a/web/src/GCodeExporter.jsx
+++ b/web/src/GCodeExporter.jsx
@@ -115,53 +122,65 @@ export default function GCodeExporter() {
       <input
         type="text"
         className="bg-gray-800 p-2 w-full rounded"
         placeholder="File name"
         value={filename}
         onChange={(e) => setFilename(e.target.value)}
       />
       <select
         className="bg-gray-800 p-2 w-full rounded"
         value={controller}
         onChange={(e) => setController(e.target.value)}
       >
         <option>GRBL</option>
         <option>Smoothie</option>
         <option>LinuxCNC</option>
       </select>
       <button
         className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded w-full"
         onClick={generate}
         disabled={loading}
       >
         {loading ? 'Generating...' : 'Generate G-code'}
       </button>
       {gcode && (
         <>
-          <select
-            className="bg-gray-800 p-2 w-full rounded"
-            value={port}
-            onChange={(e) => setPort(e.target.value)}
-          >
-            {ports.map((p) => (
-              <option key={p} value={p}>
-                {p}
-              </option>
-            ))}
-          </select>
+          <div className="flex space-x-2">
+            <select
+              className="bg-gray-800 p-2 flex-1 rounded"
+              value={port}
+              onChange={(e) => setPort(e.target.value)}
+            >
+              {ports.map((p) => (
+                <option key={p} value={p}>
+                  {p}
+                </option>
+              ))}
+            </select>
+            <button
+              className="bg-gray-700 hover:bg-gray-600 text-white px-4 rounded"
+              onClick={() => {
+                postLog('refresh ports');
+                fetchPorts();
+              }}
+              type="button"
+            >
+              Refresh
+            </button>
+          </div>
           <button
             className="bg-green-600 hover:bg-green-700 text-white px-4 py-2 rounded w-full"
             onClick={send}
             disabled={sending}
           >
             {sending ? 'Sending...' : 'Send to Machine'}
           </button>
         </>
       )}
       {output && (
         <pre className="bg-gray-800 p-2 rounded text-sm whitespace-pre-wrap">
           {output}
         </pre>
       )}
     </div>
   );
 }
