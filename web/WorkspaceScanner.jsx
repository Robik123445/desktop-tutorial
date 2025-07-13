import React, { useState } from 'react';
import log from './logger';

/**
 * Select scanning mode and trigger workspace scanning.
 */
export default function WorkspaceScanner({ onScan }) {
  const [mode, setMode] = useState('hybrid');
  const [busy, setBusy] = useState(false);

  const start = () => {
    log(`workspace scan start: ${mode}`);
    setBusy(true);
    // Mock async scanning
    setTimeout(() => {
      setBusy(false);
      log('workspace scan complete');
      onScan && onScan({ mode });
    }, 300);
  };

  return (
    <div className="space-y-2 p-4" data-testid="scanner">
      <label className="block text-sm font-medium">Scan mode</label>
      <select
        value={mode}
        onChange={e => setMode(e.target.value)}
        className="border rounded p-2 w-full"
      >
        <option value="camera">Camera only</option>
        <option value="probe">Probe only</option>
        <option value="hybrid">Hybrid (both)</option>
      </select>
      <button
        onClick={start}
        disabled={busy}
        className="bg-blue-600 text-white px-2 py-1 rounded w-full"
      >
        {busy ? 'Scanning...' : 'Start Scan'}
      </button>
    </div>
  );
}
