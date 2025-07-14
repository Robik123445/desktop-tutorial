+import React from 'react';
+import { LayoutDashboard, FileCode, Brain, Terminal, Upload } from 'lucide-react';
+
+/**
+ * Sidebar navigation for the CAM dashboard.
+ * Calls onNavigate(section) when a menu item is clicked.
+ */
+export default function Sidebar({ onNavigate = () => {} }) {
+  const items = [
+    { label: 'Dashboard', section: 'dashboard', icon: LayoutDashboard },
+    { label: 'Toolpath Viewer', section: 'toolpath', icon: FileCode },
+    { label: 'AI Analyzer', section: 'ai', icon: Brain },
+    { label: 'Log Viewer', section: 'logs', icon: Terminal },
+    { label: 'G-code Export', section: 'export', icon: Upload },
+  ];
+
+  return (
+    <aside className="w-56 bg-gray-900 text-gray-100 min-h-screen p-4">
+      <nav className="space-y-1">
+        {items.map(({ label, section, icon: Icon }) => (
+          <button
+            key={section}
+            onClick={() => onNavigate(section)}
+            className="flex items-center gap-3 w-full px-3 py-2 rounded hover:bg-gray-700 text-left"
+          >
+            {Icon && <Icon className="w-5 h-5" />}
+            <span className="text-sm">{label}</span>
+          </button>
+        ))}
+      </nav>
+    </aside>
+  );
+}
