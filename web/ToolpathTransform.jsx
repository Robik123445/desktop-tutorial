import React, { useState, useEffect } from 'react';
import log from './src/log.js';

/**
 * 2D toolpath transformation controls.
 * Rotates, scales and translates a toolpath.
 */
export default function ToolpathTransform({ onChange }) {
  const [rotation, setRotation] = useState(0);
  const [scale, setScale] = useState(100);
  const [offsetX, setOffsetX] = useState(0);
  const [offsetY, setOffsetY] = useState(0);

  useEffect(() => {
    const transform = { rotation, scale: scale / 100, x: offsetX, y: offsetY };
    log(`transform updated: ${JSON.stringify(transform)}`);
    if (onChange) onChange(transform);
  }, [rotation, scale, offsetX, offsetY, onChange]);

  const reset = () => {
    setRotation(0);
    setScale(100);
    setOffsetX(0);
    setOffsetY(0);
  };

  return (
    <div className="space-y-4 p-4 max-w-sm mx-auto">
      <div>
        <label className="block text-sm font-medium" htmlFor="rotation">Rotation ({rotation}Â°)</label>
        <input
          id="rotation"
          type="range"
          min="0"
          max="360"
          value={rotation}
          onChange={(e) => setRotation(Number(e.target.value))}
          className="w-full"
        />
      </div>
      <div>
        <label className="block text-sm font-medium" htmlFor="scale">Scale ({scale}%)</label>
        <input
          id="scale"
          type="range"
          min="1"
          max="500"
          value={scale}
          onChange={(e) => setScale(Number(e.target.value))}
          className="w-full"
        />
      </div>
      <div className="flex space-x-2">
        <div className="flex-1">
          <label className="block text-sm font-medium" htmlFor="offset-x">X Offset (mm)</label>
          <input
            id="offset-x"
            type="number"
            value={offsetX}
            onChange={(e) => setOffsetX(Number(e.target.value))}
            className="w-full border rounded p-1"
          />
        </div>
        <div className="flex-1">
          <label className="block text-sm font-medium" htmlFor="offset-y">Y Offset (mm)</label>
          <input
            id="offset-y"
            type="number"
            value={offsetY}
            onChange={(e) => setOffsetY(Number(e.target.value))}
            className="w-full border rounded p-1"
          />
        </div>
      </div>
      <button
        className="bg-gray-300 text-gray-800 px-4 py-2 rounded hover:bg-gray-400"
        onClick={reset}
      >
        Reset Transform
      </button>
    </div>
  );
}
