+const fs = require('fs');
+const path = require('path');
+
+const logFile = path.join(__dirname, 'log.txt');
+
+/**
+ * Append a timestamped message to web/log.txt.
+ */
+function log(message) {
+  const line = `${new Date().toISOString()} ${message}\n`;
+  try {
+    fs.appendFileSync(logFile, line);
+  } catch (err) {
+    console.error('Failed to write log.txt', err);
+  }
+}
+
+module.exports = log;
