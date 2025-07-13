import React, { useState, useEffect } from 'react';
import log, { getLogContent } from './logger';

/**
 * Display basic statistics parsed from central.log.
 */
export default function RobotDiagnosticsDashboard() {
  const [stats, setStats] = useState({ runs: 0, warnings: 0, errors: 0, feedback: 0 });

  const parseLogs = () => {
    const text = getLogContent();
    const lines = text.split(/\n/);
    let runs = 0, warnings = 0, errors = 0, feedback = 0;
    lines.forEach(line => {
      if (line.includes('Robot run')) runs += line.includes('started') ? 1 : 0;
      if (line.includes('WARNING') || line.includes('Robot warning')) warnings += 1;
      if (line.includes('ERROR') || line.includes('Robot error')) errors += 1;
      if (line.includes('feedback')) feedback += 1;
    });
    setStats({ runs, warnings, errors, feedback });
  };

  useEffect(() => {
    parseLogs();
  }, []);

  return (
    <div className="p-4 space-y-2">
      <button onClick={parseLogs} className="bg-gray-700 text-white px-2 py-1 rounded">Refresh</button>
      <div className="space-y-1">
        <div>Runs: {stats.runs}</div>
        <div>Warnings: {stats.warnings}</div>
        <div>Errors: {stats.errors}</div>
        <div>Feedback: {stats.feedback}</div>
      </div>
    </div>
  );
}
