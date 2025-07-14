diff --git a//dev/null b/web/src/PluginPanel.jsx
index 0000000000000000000000000000000000000000..c3db0478122d6a15b96b42ec153955cc306f7a40 100644
--- a//dev/null
+++ b/web/src/PluginPanel.jsx
@@ -0,0 +1,80 @@
+import React, { useState, useEffect } from 'react';
+
+/**
+ * PluginPanel loads available plugins from `/plugins` and allows running them.
+ * Each run posts the plugin name to `/plugins/run` and logs the action.
+ */
+//------------------------------------------------------------------------------
+// PluginPanel component
+//------------------------------------------------------------------------------
+export default function PluginPanel() {
+  const [plugins, setPlugins] = useState([]);
+  const [output, setOutput] = useState('');
+  const [running, setRunning] = useState('');
+
+  // Fetch plugin list on mount
+  useEffect(() => {
+    fetch('/plugins')
+      .then((res) => res.json())
+      .then((data) => {
+        if (Array.isArray(data)) setPlugins(data);
+      })
+      .catch((err) => console.error('failed to load plugins', err));
+  }, []);
+
+  // Post a log message without awaiting the result
+  const postLog = (message) =>
+    fetch('/log', {
+      method: 'POST',
+      headers: { 'Content-Type': 'application/json' },
+      body: JSON.stringify({ message })
+    }).catch(() => {});
+
+  // Run the selected plugin
+  const runPlugin = async (name) => {
+    setRunning(name);
+    postLog(`run plugin ${name}`);
+    try {
+      const res = await fetch('/plugins/run', {
+        method: 'POST',
+        headers: { 'Content-Type': 'application/json' },
+        body: JSON.stringify({ name })
+      });
+      const data = await res.json();
+      setOutput(JSON.stringify(data, null, 2));
+    } catch (err) {
+      console.error('run plugin failed', err);
+      setOutput('Failed to run plugin');
+    } finally {
+      setRunning('');
+    }
+  };
+
+  return (
+    <div className="bg-gray-900 text-gray-100 p-4 rounded space-y-3">
+      <h2 className="text-lg font-semibold">Plugins</h2>
+      <ul className="space-y-2">
+        {plugins.map((p) => (
+          <li key={p.name} className="flex items-start justify-between">
+            <div className="flex-1 mr-2">
+              <div className="font-medium">{p.name}</div>
+              <div className="text-gray-400 text-sm">{p.description}</div>
+            </div>
+            <button
+              className="bg-blue-600 hover:bg-blue-700 text-white px-3 py-1 rounded"
+              onClick={() => runPlugin(p.name)}
+              disabled={running === p.name}
+            >
+              {running === p.name ? 'Running...' : 'Run'}
+            </button>
+          </li>
+        ))}
+      </ul>
+      {output && (
+        <pre className="bg-gray-800 p-2 rounded text-sm whitespace-pre-wrap">
+          {output}
+        </pre>
+      )}
+    </div>
+  );
+}
