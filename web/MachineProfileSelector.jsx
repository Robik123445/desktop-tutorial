import React, { useState, useEffect } from 'react';
import log from './src/log.js';

/**
 * Select CNC machine profile and show its parameters.
 * Predefined profiles are read-only while "Custom" allows editing
 * of axis limits, max feedrate and acceleration. Selection is saved
 * to localStorage under the key "machineProfile" and each change is
 * logged to logs/central.log.
 */
export default function MachineProfileSelector() {
  const DEFAULT_CUSTOM = {
    axis: { X: 300, Y: 300, Z: 50 },
    feedrate: 1500,
    acceleration: 800,
  };

  const PRESETS = {
    'GRBL Small': {
      axis: { X: 300, Y: 300, Z: 50 },
      feedrate: 1500,
      acceleration: 800,
    },
    'GRBL Large': {
      axis: { X: 600, Y: 600, Z: 100 },
      feedrate: 2500,
      acceleration: 1200,
    },
    Smoothieware: {
      axis: { X: 500, Y: 500, Z: 80 },
      feedrate: 3000,
      acceleration: 1500,
    },
  };

  const stored = localStorage.getItem('machineProfile');
  const initial = stored ? JSON.parse(stored) : { name: 'GRBL Small', config: PRESETS['GRBL Small'] };

  const [profile, setProfile] = useState(initial.name);
  const [config, setConfig] = useState(initial.name === 'Custom' ? initial.config : PRESETS[initial.name] || DEFAULT_CUSTOM);

  useEffect(() => {
    localStorage.setItem('machineProfile', JSON.stringify({ name: profile, config }));
  }, [profile, config]);

  const handleSelect = (e) => {
    const name = e.target.value;
    setProfile(name);
    if (name !== 'Custom') {
      setConfig(PRESETS[name]);
    }
    log(`Machine profile selected: ${name}`);
  };

  const handleConfigChange = (field, axis) => (e) => {
    const value = parseFloat(e.target.value);
    setConfig((prev) => {
      const updated = { ...prev };
      if (axis) {
        updated.axis = { ...prev.axis, [axis]: value };
      } else {
        updated[field] = value;
      }
      return updated;
    });
  };

  const renderInput = (label, value, onChange) => (
    <div className="flex items-center space-x-2">
      <label className="w-32">{label}</label>
      {profile === 'Custom' ? (
        <input
          type="number"
          value={value}
          onChange={onChange}
          className="border rounded p-1 w-24"
          data-testid={`input-${label}`}
        />
      ) : (
        <span data-testid={`value-${label}`}>{value}</span>
      )}
    </div>
  );

  return (
    <div className="p-4 space-y-4 max-w-sm mx-auto">
      <label className="block font-medium mb-1">Machine Profile</label>
      <select
        value={profile}
        onChange={handleSelect}
        className="border rounded p-2 w-full"
        data-testid="profile-select"
      >
        {['GRBL Small', 'GRBL Large', 'Smoothieware', 'Custom'].map((name) => (
          <option key={name} value={name}>{name}</option>
        ))}
      </select>

      {renderInput('X limit', config.axis.X, handleConfigChange('axis', 'X'))}
      {renderInput('Y limit', config.axis.Y, handleConfigChange('axis', 'Y'))}
      {renderInput('Z limit', config.axis.Z, handleConfigChange('axis', 'Z'))}
      {renderInput('Feedrate', config.feedrate, handleConfigChange('feedrate'))}
      {renderInput('Acceleration', config.acceleration, handleConfigChange('acceleration'))}
    </div>
  );
}
