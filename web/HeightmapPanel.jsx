import React, { useState } from 'react';

/**
 * Upload G-code and a heightmap file then download adjusted G-code.
 */
export default function HeightmapPanel() {
  const [gcodeFile, setGcodeFile] = useState(null);
  const [mapFile, setMapFile] = useState(null);
  const [result, setResult] = useState('');
  const [error, setError] = useState('');

  const apply = async () => {
    if (!gcodeFile || !mapFile) {
      setError('Please select both files');
      return;
    }
    setError('');
    const gcode = await gcodeFile.text();
    const heightmap = await mapFile.text();
    try {
      const resp = await fetch('/heightmap', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Access-Token': 'changeme',
        },
        body: JSON.stringify({ gcode, heightmap, format: mapFile.name.split('.').pop() }),
      });
      if (!resp.ok) throw new Error('fail');
      const data = await resp.json();
      setResult(data.gcode);
    } catch {
      setError('Failed to apply heightmap');
    }
  };

  return (
    <div className="p-4 space-y-4 max-w-xl mx-auto">
      <input type="file" accept=".gcode,.nc,.txt" onChange={(e) => setGcodeFile(e.target.files[0])} />
      <input type="file" accept=".csv,.json" onChange={(e) => setMapFile(e.target.files[0])} />
      <button onClick={apply} className="bg-blue-600 text-white px-3 py-1 rounded">
        Apply Heightmap
      </button>
      {error && <div className="text-red-600">{error}</div>}
      {result && <textarea value={result} readOnly className="w-full border rounded p-2 h-40" />}
    </div>
  );
}
