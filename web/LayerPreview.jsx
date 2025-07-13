import React, { useState, useRef, useEffect } from 'react';
import log from './logger';

/**
 * Display sliced toolpath layers with a simple preview.
 * Allows playing through layers and toggling laser/3D mode.
 */
export default function LayerPreview({ layers = [] }) {
  const [layer, setLayer] = useState(0);
  const [laserMode, setLaserMode] = useState(true);
  const timerRef = useRef(null);
  const max = Math.max(layers.length - 1, 0);

  const play = () => {
    if (timerRef.current || max === 0) return;
    log('Play preview');
    timerRef.current = setInterval(() => {
      setLayer((prev) => (prev < max ? prev + 1 : 0));
    }, 500);
  };

  useEffect(() => () => clearInterval(timerRef.current), []);

  const current = layers[layer] || [];

  // convert coordinates to viewBox 0-100 for simplicity
  const lines = current.map(([p1, p2], i) => {
    const color = laserMode ? 'blue' : `hsl(${(p1[2] || 0) * 30},100%,50%)`;
    return (
      <line
        key={i}
        x1={p1[0]}
        y1={100 - p1[1]}
        x2={p2[0]}
        y2={100 - p2[1]}
        stroke={color}
        strokeWidth="2"
        data-testid="line"
      />
    );
  });

  return (
    <div className="space-y-4 p-4 max-w-sm mx-auto">
      <div>
        <input
          type="range"
          min="0"
          max={max}
          value={layer}
          onChange={(e) => setLayer(Number(e.target.value))}
          className="w-full"
        />
        <div className="text-sm">Layer {layer} / {max}</div>
      </div>
      <svg viewBox="0 0 100 100" className="border w-full h-48 bg-white">
        {lines}
      </svg>
      <div className="flex items-center space-x-2">
        <button
          onClick={play}
          className="bg-blue-600 text-white px-3 py-1 rounded hover:bg-blue-700"
        >
          Play preview
        </button>
        <label className="flex items-center space-x-1">
          <input
            type="checkbox"
            checked={!laserMode}
            onChange={(e) => setLaserMode(!e.target.checked)}
          />
          <span className="text-sm">Laser mode</span>
        </label>
      </div>
    </div>
  );
}
