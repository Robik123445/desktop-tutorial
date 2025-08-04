diff --git a/web/BugReportForm.jsx b/web/BugReportForm.jsx
index ba896aa21145c2670d2a65206c9d7a1b339d58ec..dc3b47fbeb10a2a1c58436262a5943708f20c54d 100644
--- a/web/BugReportForm.jsx
+++ b/web/BugReportForm.jsx
@@ -1,27 +1,27 @@
 import React, { useState } from 'react';
-import log from './logger';
+import log from './src/log.js';
 
 /**
  * Allow users to submit bug reports or suggestions.
  */
 export default function BugReportForm() {
   const [text, setText] = useState('');
   const send = () => {
     const msg = text.trim();
     if (!msg) return;
     log(`bug report: ${msg}`);
     alert('Report submitted. Thanks!');
     setText('');
   };
   return (
     <div className="p-2 space-y-2">
       <textarea
         value={text}
         onChange={e => setText(e.target.value)}
         className="w-full p-1 text-black rounded"
         placeholder="Describe the issue"
         data-testid="bug-input"
       />
       <button
         onClick={send}
         className="bg-red-600 text-white px-3 py-1 rounded"
