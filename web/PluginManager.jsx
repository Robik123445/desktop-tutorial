import React, { useState } from 'react';
import log from './src/log.js';

/**
 * Manage plugins and AI actions.
 * Displays plugin list with enable toggles and provides simple
 * buttons for AI analysis and optimization.
 */
export default function PluginManager({ plugins = [] }) {
  const [list, setList] = useState(
    plugins.map((p) => ({ ...p, enabled: true }))
  );
  const [name, setName] = useState('');

  const toggle = (idx) => {
    setList((prev) => {
      const next = [...prev];
      next[idx].enabled = !next[idx].enabled;
      log(`plugin ${next[idx].name} -> ${next[idx].enabled ? 'on' : 'off'}`);
      return next;
    });
  };

  const analyze = () => {
    log('AI analysis requested');
  };

  const optimize = () => {
    log('AI auto optimize');
  };

  const install = () => {
    if (!name) return;
    log(`install plugin ${name}`);
    setList((prev) => [...prev, { name, enabled: true }]);
    setName('');
  };

  const remove = (idx) => {
    const plug = list[idx];
    setList((prev) => prev.filter((_, i) => i !== idx));
    log(`remove plugin ${plug.name}`);
  };

  return (
    <div className="p-4 space-y-4 max-w-md mx-auto">
      <h2 className="font-medium">Plugins</h2>
      <div className="space-y-1">
        {list.map((p, idx) => (
          <label key={p.name} className="flex items-center space-x-2">
            <input
              type="checkbox"
              checked={p.enabled}
              onChange={() => toggle(idx)}
              data-testid={`plug-${idx}`}
            />
            <span>{p.name}</span>
            <button
              onClick={() => remove(idx)}
              className="ml-auto text-red-500"
              title="Remove"
            >
              Ã
            </button>
          </label>
        ))}
      </div>
      <div className="pt-2 space-x-2">
        <button
          onClick={analyze}
          className="bg-blue-600 text-white px-3 py-1 rounded"
          data-testid="ai-analyze"
        >
          Run Analysis
        </button>
        <button
          onClick={optimize}
          className="bg-green-600 text-white px-3 py-1 rounded"
          data-testid="ai-opt"
        >
          Auto Optimize
        </button>
        <input
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
          placeholder="plugin name"
          className="border px-2 py-1 rounded"
        />
        <button
          onClick={install}
          className="bg-gray-700 text-white px-3 py-1 rounded"
        >
          Install
        </button>
      </div>
    </div>
  );
}
