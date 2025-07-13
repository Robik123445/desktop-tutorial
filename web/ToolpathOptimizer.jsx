import React, { useState } from 'react';
import log from './logger';

/**
 * Apply AI optimization to a toolpath using a mock API call.
 */
export default function ToolpathOptimizer({ gcode = '', heightmap = {} }) {
  const [material, setMaterial] = useState('Plywood');
  const [depth, setDepth] = useState(1);
  const [risk, setRisk] = useState('Low');
  const [optimized, setOptimized] = useState('');
  const [loading, setLoading] = useState(false);

  const optimize = () => {
    log(`optimize request: ${material} depth ${depth} risk ${risk}`);
    setLoading(true);
    setTimeout(() => {
      const lines = gcode.split(/\r?\n/).map(l => l.trim()).filter(Boolean);
      const out = lines.map(line => `${line} ;OPT`);
      setOptimized(out.join('\n'));
      setLoading(false);
      log('optimization complete');
    }, 300);
  };

  const preview = gcode.split(/\r?\n/).slice(0, 5).join('\n');
  const optimizedPreview = optimized.split(/\r?\n/).slice(0, 5).join('\n');

  return (
    <div className="p-4 space-y-4 max-w-md mx-auto">
      <div>
        <label className="block text-sm font-medium">Material</label>
        <select
          value={material}
          onChange={(e) => setMaterial(e.target.value)}
          className="border rounded p-2 w-full"
          data-testid="material"
        >
          {['Plywood', 'MDF', 'Aluminum', 'Acrylic'].map((m) => (
            <option key={m} value={m}>{m}</option>
          ))}
        </select>
      </div>
      <div>
        <label className="block text-sm font-medium">Max Depth per Pass (mm)</label>
        <input
          type="number"
          value={depth}
          onChange={(e) => setDepth(Number(e.target.value))}
          className="border rounded p-2 w-full"
          data-testid="depth"
        />
      </div>
      <div>
        <label className="block text-sm font-medium">Risk Level</label>
        <select
          value={risk}
          onChange={(e) => setRisk(e.target.value)}
          className="border rounded p-2 w-full"
          data-testid="risk"
        >
          {['Low', 'Medium', 'High'].map((r) => (
            <option key={r} value={r}>{r}</option>
          ))}
        </select>
      </div>
      <button
        onClick={optimize}
        disabled={loading}
        className="bg-blue-600 text-white px-3 py-1 rounded hover:bg-blue-700"
        data-testid="opt-button"
      >
        {loading ? 'Optimizing...' : 'Optimize'}
      </button>
      <div className="flex space-x-4">
        <div className="flex-1">
          <div className="font-medium">Before</div>
          <pre className="text-xs border p-2" data-testid="before">{preview}</pre>
        </div>
        <div className="flex-1">
          <div className="font-medium">After</div>
          <pre className="text-xs border p-2" data-testid="after">{optimizedPreview}</pre>
        </div>
      </div>
    </div>
  );
}
