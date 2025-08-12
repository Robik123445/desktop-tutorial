import React, { useState } from 'react';
import log from './src/log.js';

/**
 * Form to save a project and export G-code. Shows optional notes field
 * and a toast after saving.
 */
export default function ProjectSaver({ gcodeLines = [] }) {
  const [name, setName] = useState('');
  const [withNotes, setWithNotes] = useState(false);
  const [notes, setNotes] = useState('');
  const [toast, setToast] = useState(false);
  const [error, setError] = useState('');

  const saveProject = () => {
    log(`Save project: ${name}`);
    if (withNotes) {
      log(`Notes: ${notes}`);
    }
    setToast(true);
    setTimeout(() => setToast(false), 2000);
  };

  const exportGcode = () => {
    try {
      const blob = new Blob([gcodeLines.join('\n')], { type: 'text/plain' });
      const url = URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      link.download = `${name || 'project'}.gcode`;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      URL.revokeObjectURL(url);
      setError('');
      log(`Exported G-code for ${name || 'project'}`);
    } catch (err) {
      setError('Failed to export G-code');
      log(`Failed to export G-code: ${err.message}`);
    }
  };

  return (
    <div className="p-4 space-y-4 max-w-md mx-auto">
      <div>
        <label className="block font-medium mb-1">Project Name</label>
        <input
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
          className="w-full border rounded p-2"
          data-testid="project-name"
        />
      </div>
      <div className="flex items-center space-x-2">
        <input
          id="notes"
          type="checkbox"
          checked={withNotes}
          onChange={(e) => setWithNotes(e.target.checked)}
        />
        <label htmlFor="notes">Add notes</label>
      </div>
      {withNotes && (
        <textarea
          value={notes}
          onChange={(e) => setNotes(e.target.value)}
          className="w-full border rounded p-2"
          rows="3"
          data-testid="notes-area"
        />
      )}
      <div className="flex space-x-2">
        <button
          onClick={saveProject}
          className="bg-green-600 text-white px-3 py-1 rounded hover:bg-green-700"
        >
          Save Project
        </button>
        <button
          onClick={exportGcode}
          className="bg-blue-600 text-white px-3 py-1 rounded hover:bg-blue-700"
        >
          Export G-code
        </button>
      </div>
      {toast && (
        <div className="bg-gray-800 text-white p-2 rounded" data-testid="toast">
          Project saved successfully.
        </div>
      )}
      {error && (
        <div className="text-red-600" data-testid="ps-error">{error}</div>
      )}
    </div>
  );
}
