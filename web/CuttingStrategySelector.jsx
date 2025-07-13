import React, { useState } from 'react';
import log from './logger';

/**
 * Radio group to choose a cutting strategy. Each option includes
 * a small icon and description. Selection is logged to logs/central.log.
 */
export default function CuttingStrategySelector({ onChange }) {
  const [strategy, setStrategy] = useState('climb');

  const OPTIONS = [
    { value: 'climb', label: 'Climb milling', icon: 'â»', desc: 'Tool rotates with feed' },
    { value: 'conventional', label: 'Conventional milling', icon: 'âº', desc: 'Tool rotates against feed' },
    { value: 'spiral', label: 'Spiral pocket', icon: 'ð', desc: 'Continuous spiral inwards' },
    { value: 'adaptive', label: 'Adaptive (AI)', icon: 'ð¤', desc: 'AI adjusts cutting parameters' },
  ];

  const handleChange = (e) => {
    const value = e.target.value;
    setStrategy(value);
    log(`Cutting strategy: ${value}`);
    if (onChange) onChange(value);
  };

  return (
    <div className="p-4 space-y-2" data-testid="strategy-selector">
      {OPTIONS.map((opt) => (
        <label key={opt.value} className="flex items-start space-x-2">
          <input
            type="radio"
            name="cutting"
            value={opt.value}
            checked={strategy === opt.value}
            onChange={handleChange}
            className="mt-1"
            data-testid={`radio-${opt.value}`}
          />
          <span className="text-xl">{opt.icon}</span>
          <span>
            <span className="font-medium">{opt.label}</span>
            <span className="block text-sm text-gray-600">{opt.desc}</span>
          </span>
        </label>
      ))}
    </div>
  );
}
