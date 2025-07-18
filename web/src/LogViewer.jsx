import React, { useState, useEffect } from 'react';

/**
 * Live log viewer.
 * Fetches central.log every 5 seconds and highlights important lines.
 */
export default function LogViewer() {
  const [text, setText] = useState('');

  useEffect(() => {
    // fetch log file from server
    const fetchLogs = () => {
      fetch('/logs/central.log', { cache: 'no-store' })
        .then((res) => res.text())
        .then(setText)
        .catch((err) => console.error('Failed to fetch logs', err));
    };
    fetchLogs();
    const timer = setInterval(fetchLogs, 5000);
    return () => clearInterval(timer);
  }, []);

  // split log into individual lines for highlighting
  const lines = text.split(/\n/);

  return (
    <div className="p-4">
      <h2 className="text-lg font-semibold mb-2">Live Logs</h2>
      <div className="bg-gray-900 text-gray-100 rounded h-96 overflow-y-auto p-2">
        <pre className="font-mono whitespace-pre-wrap text-sm">
          {lines.map((line, idx) => (
            <span
              key={idx}
              className={line.match(/WARNING|ERROR/) ? 'text-red-400' : ''}
            >
              {line}
              {'\n'}
            </span>
          ))}
        </pre>
      </div>
    </div>
  );
}
