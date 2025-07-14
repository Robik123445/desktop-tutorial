import React, { useState } from 'react';
 
 /**
  * Simple AI optimization panel.
  * Select an analyzer and POST to /optimize when run.
  * Click events are logged via /log.
  */
+//------------------------------------------------------------------------------
+// AIAnalyzer component
+//------------------------------------------------------------------------------
 export default function AIAnalyzer() {
-  const [tool, setTool] = useState('Feedrate Advisor');
+  const [analyzer, setAnalyzer] = useState('Feedrate Advisor');
   const [result, setResult] = useState('');
   const [loading, setLoading] = useState(false);
 
   const runAnalysis = async () => {
     setLoading(true);
     try {
       await fetch('/log', {
         method: 'POST',
         headers: { 'Content-Type': 'application/json' },
-        body: JSON.stringify({ message: `ai run: ${tool}` })
+        body: JSON.stringify({ message: `ai run: ${analyzer}` })
       });
       const res = await fetch('/optimize', {
         method: 'POST',
         headers: { 'Content-Type': 'application/json' },
-        body: JSON.stringify({ tool })
+        body: JSON.stringify({ analyzer, data: {} })
       });
       const data = await res.json();
       setResult(JSON.stringify(data, null, 2));
     } catch (err) {
       console.error('optimize failed', err);
       setResult('Failed to run');
     } finally {
       setLoading(false);
     }
   };
 
   return (
     <div className="bg-gray-900 text-gray-100 p-4 rounded space-y-2">
       <h2 className="text-lg font-semibold">AI Optimization</h2>
       <select
         className="bg-gray-800 text-white p-2 rounded w-full"
-        value={tool}
-        onChange={(e) => setTool(e.target.value)}
+        value={analyzer}
+        onChange={(e) => setAnalyzer(e.target.value)}
       >
         <option>Feedrate Advisor</option>
         <option>Trajectory Cleaner</option>
         <option>Surface Comparator</option>
       </select>
       <button
         className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded w-full"
         onClick={runAnalysis}
         disabled={loading}
       >
         {loading ? 'Running...' : 'Run Analysis'}
       </button>
       {result && (
         <pre className="bg-gray-800 p-2 rounded text-sm whitespace-pre-wrap">
           {result}
         </pre>
       )}
     </div>
   );
 }
