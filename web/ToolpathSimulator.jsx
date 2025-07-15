import React, { useState } from 'react';

/**
 * Upload G-code to the /simulate endpoint and display result.
 */
export default function ToolpathSimulator() {
  const [gcode, setGcode] = useState('');
  const [points, setPoints] = useState(null);
  const [img, setImg] = useState('');
  const [error, setError] = useState('');

  const simulate = async () => {
    setError('');
    setPoints(null);
    setImg('');
    try {
      const resp = await fetch('/simulate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Access-Token': 'changeme',
        },
        body: JSON.stringify({ gcode, include_plot: true }),
      });
      if (!resp.ok) throw new Error('fail');
      const data = await resp.json();
      setPoints(data.points);
      if (data.plot) setImg(`data:image/png;base64,${data.plot}`);
    } catch (err) {
      setError('Simulation failed');
    }
  };

  return (
    <div className="space-y-4 p-4 max-w-xl mx-auto">
      <textarea
        value={gcode}
        onChange={(e) => setGcode(e.target.value)}
        placeholder="Paste G-code"
        className="w-full border p-2 rounded h-40"
      />
      <button onClick={simulate} className="bg-blue-600 text-white px-3 py-1 rounded">
        Simulate
      </button>
      {error && <div className="text-red-600">{error}</div>}
      {points && (
        <pre className="bg-gray-100 p-2 text-sm rounded" data-testid="points">
          {JSON.stringify(points)}
        </pre>
      )}
      {img && <img src={img} alt="plot" data-testid="plot" className="border" />}
    </div>
  );
}
