-import React from 'react';
+import React, { useState } from 'react';
 import { LayoutDashboard, FileCode, Brain, Terminal, Upload } from 'lucide-react';
 
 /**
  * Sidebar navigation for the CAM dashboard.
  * Calls onNavigate(section) when a menu item is clicked.
  */
 export default function Sidebar({ onNavigate = () => {} }) {
+  // currently active section for highlighting
+  const [active, setActive] = useState('dashboard');
+
+  // menu definition
   const items = [
     { label: 'Dashboard', section: 'dashboard', icon: LayoutDashboard },
     { label: 'Toolpath Viewer', section: 'toolpath', icon: FileCode },
     { label: 'AI Analyzer', section: 'ai', icon: Brain },
     { label: 'Log Viewer', section: 'logs', icon: Terminal },
     { label: 'G-code Export', section: 'export', icon: Upload },
   ];
 
+  const postLog = (msg) =>
+    fetch('/log', {
+      method: 'POST',
+      headers: { 'Content-Type': 'application/json' },
+      body: JSON.stringify({ message: msg })
+    }).catch(() => {});
+
+  // handle click: update active state, log, and notify parent
+  const handleClick = (section) => {
+    setActive(section);
+    postLog(`navigate ${section}`);
+    onNavigate(section);
+  };
+
   return (
     <aside className="w-56 bg-gray-900 text-gray-100 min-h-screen p-4">
       <nav className="space-y-1">
         {items.map(({ label, section, icon: Icon }) => (
           <button
             key={section}
-            onClick={() => onNavigate(section)}
-            className="flex items-center gap-3 w-full px-3 py-2 rounded hover:bg-gray-700 text-left"
+            onClick={() => handleClick(section)}
+            className={`flex items-center gap-3 w-full px-3 py-2 rounded text-left ${
+              active === section ? 'bg-gray-800' : 'hover:bg-gray-700'
+            }`}
           >
             {Icon && <Icon className="w-5 h-5" />}
             <span className="text-sm">{label}</span>
           </button>
         ))}
       </nav>
     </aside>
   );
 }
