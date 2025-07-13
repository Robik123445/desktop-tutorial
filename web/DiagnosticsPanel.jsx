import React from 'react';
import log from './logger';

/**
 * DiagnosticsPanel
 * ----------------
 * Run project tests and generate system report. Only logs the actions.
 */
export default function DiagnosticsPanel() {
  const runTests = () => {
    log('diagnostics run tests');
  };
  const genReport = () => {
    log('diagnostics report');
  };
  return (
    <div className="p-4 space-y-2 max-w-md mx-auto">
      <button
        onClick={runTests}
        className="bg-blue-600 text-white px-3 py-1 rounded"
        data-testid="run-tests"
      >
        Run Tests
      </button>
      <button
        onClick={genReport}
        className="bg-green-600 text-white px-3 py-1 rounded"
        data-testid="gen-report"
      >
        System Report
      </button>
    </div>
  );
}
