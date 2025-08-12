import React, { useState, useEffect } from 'react';
import log from './src/log.js';

/**
 * List and manage available tools. Tools are persisted in localStorage
 * under the key "tools". Each tool entry contains name, diameter,
 * type and max RPM. Items can be added or removed.
 */
export default function ToolList() {
  const [tools, setTools] = useState(() => {
    const saved = localStorage.getItem('tools');
    return saved ? JSON.parse(saved) : [];
  });

  const [form, setForm] = useState({ name: '', diameter: '', type: 'flat', rpm: '' });

  // Save tools to localStorage whenever they change
  useEffect(() => {
    localStorage.setItem('tools', JSON.stringify(tools));
  }, [tools]);

  const handleAdd = () => {
    if (!form.name || !form.diameter || !form.rpm) return;
    const newTool = { ...form, diameter: parseFloat(form.diameter), rpm: parseInt(form.rpm, 10) };
    setTools((t) => [...t, newTool]);
    log(`Tool added: ${form.name}`);
    setForm({ name: '', diameter: '', type: 'flat', rpm: '' });
  };

  const handleRemove = (index) => {
    setTools((t) => t.filter((_, i) => i !== index));
    log(`Tool removed at index ${index}`);
  };

  return (
    <div className="p-4 space-y-4 max-w-md mx-auto">
      <h2 className="text-xl font-bold">Tools</h2>
      <div className="space-y-2">
        {tools.map((tool, idx) => (
          <div key={idx} className="flex items-center justify-between border p-2 rounded">
            <div>
              <p className="font-medium" data-testid={`name-${idx}`}>{tool.name}</p>
              <p className="text-sm">{tool.diameter} mm, {tool.type}, {tool.rpm} RPM</p>
            </div>
            <button
              onClick={() => handleRemove(idx)}
              className="text-red-600 text-sm"
              data-testid={`remove-${idx}`}
            >Remove</button>
          </div>
        ))}
      </div>
      <div className="space-y-2 border-t pt-4">
        <input
          type="text"
          placeholder="Tool name"
          value={form.name}
          onChange={(e) => setForm({ ...form, name: e.target.value })}
          className="border rounded p-2 w-full"
          data-testid="name-input"
        />
        <input
          type="number"
          placeholder="Diameter (mm)"
          value={form.diameter}
          onChange={(e) => setForm({ ...form, diameter: e.target.value })}
          className="border rounded p-2 w-full"
          data-testid="diameter-input"
        />
        <select
          value={form.type}
          onChange={(e) => setForm({ ...form, type: e.target.value })}
          className="border rounded p-2 w-full"
          data-testid="type-select"
        >
          <option value="flat">Flat</option>
          <option value="ball">Ball</option>
          <option value="V-bit">V-bit</option>
        </select>
        <input
          type="number"
          placeholder="Max RPM"
          value={form.rpm}
          onChange={(e) => setForm({ ...form, rpm: e.target.value })}
          className="border rounded p-2 w-full"
          data-testid="rpm-input"
        />
        <button
          onClick={handleAdd}
          className="bg-blue-500 text-white px-4 py-2 rounded"
          data-testid="add-btn"
        >Add Tool</button>
      </div>
    </div>
  );
}
