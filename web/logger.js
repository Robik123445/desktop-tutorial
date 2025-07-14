diff --git a/web/logger.js b/web/logger.js
index 0db377d83f92f7a2270f31e08e8864c156f990ce..7426d609cb67d3074624e7ea9960aeeca1f3d009 100644
--- a/web/logger.js
+++ b/web/logger.js
@@ -1,49 +1,54 @@
-const fs = require('fs');
+const fs = require('fs').promises;
 const path = require('path');
 
 const logPath = path.join(__dirname, '..', 'logs', 'central.log');
 const logDir = path.dirname(logPath);
 const MAX_SIZE = 1024 * 1024; // 1 MB
 
 const LEVELS = { DEBUG: 0, INFO: 1, WARNING: 2, ERROR: 3 };
 let currentLevel = LEVELS[(process.env.LOG_LEVEL || 'INFO').toUpperCase()] || LEVELS.INFO;
 
 /**
  * Append a log entry to the shared central log file.
  */
-function log(message, level = 'INFO', error) {
+async function log(message, level = 'INFO', error) {
   const lvl = LEVELS[level.toUpperCase()] ?? LEVELS.INFO;
   if (lvl < currentLevel) return;
 
   let entry = `${new Date().toISOString()} ${level}: ${message}`;
   if (error) {
     entry += ` - ${error.stack || error}`;
   }
   entry += '\n';
   try {
-    fs.mkdirSync(logDir, { recursive: true });
-    if (fs.existsSync(logPath) && fs.statSync(logPath).size > MAX_SIZE) {
-      fs.renameSync(logPath, `${logPath}.1`);
+    await fs.mkdir(logDir, { recursive: true });
+    try {
+      const stat = await fs.stat(logPath);
+      if (stat.size > MAX_SIZE) {
+        await fs.rename(logPath, `${logPath}.1`);
+      }
+    } catch (err) {
+      if (err.code !== 'ENOENT') throw err;
     }
-    fs.appendFileSync(logPath, entry);
+    await fs.appendFile(logPath, entry);
   } catch (err) {
     console.error('Failed to write log:', err);
   }
 }
 
 /**
  * Read log file content for export.
  */
 function getLogContent() {
   try {
     fs.mkdirSync(logDir, { recursive: true });
     return fs.existsSync(logPath) ? fs.readFileSync(logPath, 'utf8') : '';
   } catch (err) {
     console.error('Failed to read log:', err);
     return '';
   }
 }
 
 module.exports = log;
 module.exports.getLogContent = getLogContent;
 module.exports.logPath = logPath;
