import React, { useRef, useState } from 'react';
import { saveAs } from 'file-saver';
import log from './src/log.js';

/**
 * ProjectManager allows saving the full CAM project to a JSON file
 * and loading it back later. It logs all actions to logs/central.log.
 */
export default function ProjectManager({
  toolpaths = [],
  material = '',
  machineProfile = {},
  settings = {},
  onLoad,
}) {
  const inputRef = useRef(null);
  const [error, setError] = useState('');

  const saveProject = () => {
    try {
      const data = { toolpaths, material, machineProfile, settings };
      const blob = new Blob([JSON.stringify(data, null, 2)], {
        type: 'application/json',
      });
      saveAs(blob, 'cam_project.json');
      setError('');
      log('Project saved to cam_project.json');
    } catch (err) {
      setError('Failed to save project');
      log(`Failed to save project: ${err.message}`);
    }
  };

  const handleLoad = (e) => {
    const file = e.target.files[0];
    if (!file) return;
    if (file.size > 10 * 1024 * 1024) {
      setError('File too large');
      log(`Project file too large: ${file.name}`);
      return;
    }
    const reader = new FileReader();
    reader.onerror = () => {
      setError('Failed to read file');
      log(`Failed to read project: ${file.name}`);
    };
    reader.onload = () => {
      try {
        const data = JSON.parse(reader.result);
        setError('');
        log(`Project loaded: ${file.name}`);
        onLoad && onLoad(data);
      } catch (err) {
        setError('Corrupted project file');
        log(`Failed to load project: ${err.message}`);
      }
    };
    reader.readAsText(file);
  };

  return (
    <div className="p-4 space-x-2">
      <button
        onClick={saveProject}
        className="bg-green-600 text-white px-3 py-1 rounded hover:bg-green-700"
      >
        Save Project
      </button>
      <input
        type="file"
        accept="application/json"
        ref={inputRef}
        onChange={handleLoad}
        className="hidden"
        data-testid="load-input"
      />
      <button
        onClick={() => inputRef.current && inputRef.current.click()}
        className="bg-blue-600 text-white px-3 py-1 rounded hover:bg-blue-700"
        data-testid="load-button"
      >
        Load Project
      </button>
      {error && (
        <p className="text-red-600 mt-2" data-testid="pm-error">{error}</p>
      )}
    </div>
  );
}
