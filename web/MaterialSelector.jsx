import React, { useState } from 'react';
import log from './logger';

// Predefined materials with suggested spindle speeds and feedrates
const MATERIAL_DATA = {
  Plywood: { speed: 18000, feed: 1000 },
  MDF: { speed: 16000, feed: 900 },
  Aluminum: { speed: 12000, feed: 300 },
  Acrylic: { speed: 14000, feed: 500 },
};

const MATERIALS = [...Object.keys(MATERIAL_DATA), 'Custom'];

/**
 * Dropdown to select material and display recommended spindle speed
 * and feedrate. Selecting "Custom" reveals an input for a custom
 * material name. All selections are logged to logs/central.log.
 */
export default function MaterialSelector() {
  const [material, setMaterial] = useState('Plywood');
  const [customName, setCustomName] = useState('');

  const handleChange = (e) => {
    const value = e.target.value;
    setMaterial(value);
    log(`Material selected: ${value}`);
  };

  const info = MATERIAL_DATA[material] || { speed: 'N/A', feed: 'N/A' };

  return (
    <div className="p-4 space-y-2 max-w-sm mx-auto">
      <label className="block font-medium mb-1">Material</label>
      <select
        value={material}
        onChange={handleChange}
        className="border rounded p-2 w-full"
        data-testid="material-select"
      >
        {MATERIALS.map((m) => (
          <option key={m} value={m}>
            {m}
          </option>
        ))}
      </select>
      {material === 'Custom' && (
        <input
          type="text"
          value={customName}
          onChange={(e) => setCustomName(e.target.value)}
          placeholder="Custom material"
          className="border rounded p-2 w-full"
          data-testid="custom-input"
        />
      )}
      <div className="pt-2">
        <p className="text-sm">Spindle Speed: <span data-testid="speed">{info.speed}</span> RPM</p>
        <p className="text-sm">Feedrate: <span data-testid="feed">{info.feed}</span> mm/min</p>
      </div>
    </div>
  );
}
