import React, { useState } from 'react';
import log from './src/log.js';

/**
 * Step-by-step wizard guiding a new user through setup.
 * Steps: 1) import design, 2) select material, 3) choose tool,
 * 4) set machine, 5) preview and export G-code.
 * Actions are logged to logs/central.log.
 */
export default function SetupWizard({ gcodeLines = [] }) {
  const [step, setStep] = useState(1);
  const [design, setDesign] = useState(null);
  const [material, setMaterial] = useState('Plywood');
  const [tool, setTool] = useState('Flat 3mm');
  const [machine, setMachine] = useState('GRBL Small');

  const next = () => setStep((s) => Math.min(s + 1, 5));
  const back = () => setStep((s) => Math.max(s - 1, 1));

  const handleFile = (e) => {
    const f = e.target.files[0];
    if (f) {
      setDesign(f);
      log(`design imported: ${f.name}`);
    }
  };

  const exportGcode = () => {
    const blob = new Blob([gcodeLines.join('\n')], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = 'output.gcode';
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
    log('wizard export gcode');
  };

  const progress = ((step - 1) / 4) * 100;

  return (
    <div className="max-w-lg mx-auto p-4 space-y-4">
      {/* Progress bar */}
      <div className="h-2 bg-gray-200 rounded">
        <div
          className="h-2 bg-blue-600 rounded"
          style={{ width: `${progress}%` }}
          data-testid="progress"
        />
      </div>
      {step === 1 && (
        <div className="space-y-2">
          <p className="font-medium">Import design (SVG or STL)</p>
          <input
            type="file"
            accept=".svg,.stl"
            onChange={handleFile}
            data-testid="design-input"
          />
          {design && <p data-testid="design-name">{design.name}</p>}
        </div>
      )}
      {step === 2 && (
        <div className="space-y-2">
          <label className="block font-medium mb-1">Select material</label>
          <select
            value={material}
            onChange={(e) => {
              setMaterial(e.target.value);
              log(`wizard material: ${e.target.value}`);
            }}
            className="border rounded p-2 w-full"
            data-testid="material-select"
          >
            {['Plywood', 'MDF', 'Aluminum', 'Acrylic'].map((m) => (
              <option key={m} value={m}>{m}</option>
            ))}
          </select>
        </div>
      )}
      {step === 3 && (
        <div className="space-y-2">
          <label className="block font-medium mb-1">Choose tool</label>
          <select
            value={tool}
            onChange={(e) => {
              setTool(e.target.value);
              log(`wizard tool: ${e.target.value}`);
            }}
            className="border rounded p-2 w-full"
            data-testid="tool-select"
          >
            {['Flat 3mm', 'Ball 2mm', 'V-bit 60deg'].map((t) => (
              <option key={t} value={t}>{t}</option>
            ))}
          </select>
        </div>
      )}
      {step === 4 && (
        <div className="space-y-2">
          <label className="block font-medium mb-1">Machine</label>
          <select
            value={machine}
            onChange={(e) => {
              setMachine(e.target.value);
              log(`wizard machine: ${e.target.value}`);
            }}
            className="border rounded p-2 w-full"
            data-testid="machine-select"
          >
            {['GRBL Small', 'GRBL Large', 'Smoothieware'].map((m) => (
              <option key={m} value={m}>{m}</option>
            ))}
          </select>
        </div>
      )}
      {step === 5 && (
        <div className="space-y-2" data-testid="summary">
          <p className="font-medium">Preview</p>
          <ul className="text-sm list-disc list-inside">
            <li>Design: {design ? design.name : 'n/a'}</li>
            <li>Material: {material}</li>
            <li>Tool: {tool}</li>
            <li>Machine: {machine}</li>
          </ul>
          <pre className="border p-2 overflow-auto text-xs h-24">
{gcodeLines.join('\n')}
          </pre>
          <button
            onClick={exportGcode}
            className="bg-green-600 text-white px-3 py-1 rounded hover:bg-green-700"
            data-testid="export-btn"
          >
            Export G-code
          </button>
        </div>
      )}
      <div className="flex justify-between pt-4">
        <button
          onClick={back}
          disabled={step === 1}
          className="px-3 py-1 rounded border disabled:opacity-50"
        >
          Back
        </button>
        <button
          onClick={next}
          disabled={step === 5 || (step === 1 && !design)}
          className="px-3 py-1 rounded bg-blue-600 text-white disabled:opacity-50"
          data-testid="next-btn"
        >
          Next
        </button>
      </div>
    </div>
  );
}
