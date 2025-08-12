import React, { useState } from 'react';
import log from './src/log.js';

export default function RobotTrajectoryOptimizer({ toolpath = [], profile = {} }) {
  const [optimized, setOptimized] = useState([]);
  const [warnings, setWarnings] = useState([]);
  const [show, setShow] = useState(false);

  const runOptimization = async () => {
    log('robot trajectory optimize');
    try {
      const resp = await fetch('/robot_optimize', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ points: toolpath, profile }),
      });
      const data = await resp.json();
      setOptimized(data.points || []);
      setWarnings(data.warnings || []);
      setShow(true);
    } catch (err) {
      log('robot optimize failed', 'ERROR', err);
    }
  };

  const preview = optimized.slice(0, 3).map(p => p.join(', ')).join('\n');

  return (
    <div className="p-4 space-y-2">
      <button onClick={runOptimization} className="bg-blue-600 text-white px-2 py-1 rounded">Optimize Trajectory</button>
      {show && (
        <div className="space-y-2">
          <div>
            <div className="font-medium">Suggestions</div>
            <ul className="list-disc ml-5 text-sm">
              {warnings.map((w, i) => <li key={i}>{w}</li>)}
            </ul>
          </div>
          <div>
            <div className="font-medium">Preview</div>
            <pre className="text-xs border p-2">{preview}</pre>
          </div>
          <div className="flex space-x-2">
            <button onClick={() => log('trajectory accepted')} className="bg-green-600 text-white px-2 py-1 rounded">Accept</button>
            <button onClick={() => setShow(false)} className="bg-gray-600 text-white px-2 py-1 rounded">Reject</button>
          </div>
        </div>
      )}
    </div>
  );
}
