import React, { useState } from 'react';

/**
 * Form to compute G-code statistics via API.
 */
export default function GcodeStats() {
  const [gcode, setGcode] = useState('');
  const [stats, setStats] = useState(null);
  const [error, setError] = useState('');

  const analyze = async () => {
    setError('');
    setStats(null);
    try {
      const resp = await fetch('/gcode/stats', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Access-Token': 'changeme',
        },
        body: JSON.stringify({ gcode }),
      });
      if (!resp.ok) {
        throw new Error('request failed');
      }
      const data = await resp.json();
      setStats(data);
    } catch (err) {
      setError('Failed to analyze');
    }
  };

  return (
    <div className="p-4 space-y-4 max-w-xl mx-auto">
      <textarea
        value={gcode}
        onChange={(e) => setGcode(e.target.value)}
        placeholder="Paste G-code"
        className="w-full border p-2 rounded h-40"
      />
      <button
        onClick={analyze}
        className="bg-blue-600 text-white px-3 py-1 rounded"
      >
        Analyze
      </button>
      {error && <div className="text-red-600">{error}</div>}
      {stats && (
        <pre className="bg-gray-100 p-2 text-sm rounded">
{JSON.stringify(stats, null, 2)}
        </pre>
      )}
    </div>
  );
}
