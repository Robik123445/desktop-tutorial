import React, { useState, useEffect } from 'react';

/**
 * HeightmapEditor renders a color coded grid where each cell represents
 * a height value. Clicking a cell prompts for a new Z value and logs
 * the change via `/log`.
 */
export default function HeightmapEditor({ heightmapData = [[]] }) {
  const [map, setMap] = useState(heightmapData);

  // Update map if the prop changes
  useEffect(() => setMap(heightmapData), [heightmapData]);

  const rows = map.length;
  const cols = rows ? map[0].length : 0;
  const flat = map.flat();
  const min = Math.min(...flat);
  const max = Math.max(...flat);
  const cell = 20;

  // Convert height to HSL color ranging blue->red
  const colorFor = (z) => {
    const t = (z - min) / (max - min || 1);
    const hue = 240 - 240 * t; // 240=blue, 0=red
    return `hsl(${hue},100%,50%)`;
  };

  const updateCell = (r, c) => {
    const current = map[r][c];
    const val = prompt('Height', current);
    const num = parseFloat(val);
    if (isNaN(num)) return;
    const next = map.map((row) => row.slice());
    next[r][c] = num;
    setMap(next);
    fetch('/log', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ message: `height ${r} ${c} ${num}` })
    }).catch(() => {});
  };

  return (
    <div className="bg-gray-900 text-gray-100 p-4 rounded inline-block">
      <h2 className="text-lg font-semibold mb-2">Heightmap Editor</h2>
      <svg
        data-testid="grid"
        width={cols * cell}
        height={rows * cell}
        className="border"
      >
        {map.map((row, r) =>
          row.map((z, c) => (
            <g key={`${r}-${c}`}>
              <rect
                data-testid={`cell-${r}-${c}`}
                data-value={z}
                x={c * cell}
                y={r * cell}
                width={cell}
                height={cell}
                fill={colorFor(z)}
                stroke="black"
                onClick={() => updateCell(r, c)}
              />
              <text
                x={c * cell + cell / 2}
                y={r * cell + cell / 2 + 4}
                fontSize="10"
                textAnchor="middle"
                className="pointer-events-none select-none"
              >
                {z}
              </text>
            </g>
          ))
        )}
      </svg>
    </div>
  );
}
