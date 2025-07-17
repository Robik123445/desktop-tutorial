import React, { useState } from 'react';
import HeightmapEditor from './HeightmapEditor';

/**
 * HeightmapProbe triggers a backend probing routine and displays
 * the resulting map using HeightmapEditor.
 */
export default function HeightmapProbe() {
  const [data, setData] = useState([]);
  const [running, setRunning] = useState(false);

  const startProbe = async () => {
    setRunning(true);
    try {
      const res = await fetch('/probe_heightmap', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x_start: 0, x_end: 10, y_start: 0, y_end: 10, step: 5 })
      });
      const json = await res.json();
      const pts = json.points || [];
      if (pts.length) {
        const xs = [...new Set(pts.map(p => p[0]))].sort((a, b) => a - b);
        const ys = [...new Set(pts.map(p => p[1]))].sort((a, b) => a - b);
        const grid = ys.map(y => xs.map(x => {
          const p = pts.find(pt => pt[0] === x && pt[1] === y);
          return p ? p[2] : 0;
        }));
        setData(grid);
      }
    } catch (err) {
      console.error('probe failed', err);
    } finally {
      setRunning(false);
    }
  };

  return (
    <div className="bg-gray-900 text-gray-100 p-4 rounded space-y-2">
      <h2 className="text-lg font-semibold">Heightmap Probe</h2>
      <button
        className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded"
        onClick={startProbe}
        disabled={running}
      >
        {running ? 'Probing...' : 'Start Probing'}
      </button>
      {data.length > 0 && <HeightmapEditor heightmapData={data} />}
    </div>
  );
}
