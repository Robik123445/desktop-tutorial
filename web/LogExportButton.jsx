import React from 'react';
import { saveAs } from 'file-saver';
import log, { getLogContent } from './src/log.js';

/**
 * Button that downloads `central.log` for support.
 */
export default function LogExportButton() {
  const handleExport = () => {
    try {
      const content = getLogContent();
      const blob = new Blob([content], { type: 'text/plain' });
      saveAs(blob, 'central.log');
      log('User exported logs');
    } catch (err) {
      console.error('Failed to export logs', err);
    }
  };

  return (
    <button
      onClick={handleExport}
      className="bg-gray-700 text-white px-3 py-1 rounded hover:bg-gray-800"
    >
      Export Logs
    </button>
  );
}
