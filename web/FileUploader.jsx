import React, { useState, useCallback } from 'react';
import log from './logger';

/**
 * Drag-and-drop file importer with basic preview.
 * Supports SVG, DXF, DWG, STL, OBJ, GCODE and bitmap images.
 */
export default function FileUploader({ onContinue }) {
  const [file, setFile] = useState(null);
  const [error, setError] = useState('');

  const MAX_SIZE = 10 * 1024 * 1024; // 10MB
  const allowed = ['svg','dxf','dwg','stl','obj','gcode','nc','png','jpg','jpeg'];

  const handleDrop = useCallback((e) => {
    e.preventDefault();
    if (!e.dataTransfer.files.length) return;
    const f = e.dataTransfer.files[0];
    const ext = f.name.split('.').pop().toLowerCase();
    if (!allowed.includes(ext)) {
      setError('Unsupported file type.');
      log(`Unsupported file type: ${f.name}`);
      return;
    }
    if (f.size > MAX_SIZE) {
      setError('File too large.');
      log(`File too large: ${f.name}`);
      return;
    }
    setError('');
    setFile(f);
    log(`File selected via drop: ${f.name}`);
  }, []);

  const handleInput = useCallback((e) => {
    if (!e.target.files.length) return;
    const f = e.target.files[0];
    const ext = f.name.split('.').pop().toLowerCase();
    if (!allowed.includes(ext)) {
      setError('Unsupported file type.');
      log(`Unsupported file type: ${f.name}`);
      return;
    }
    if (f.size > MAX_SIZE) {
      setError('File too large.');
      log(`File too large: ${f.name}`);
      return;
    }
    setError('');
    setFile(f);
    log(`File selected via input: ${f.name}`);
  }, []);

  const preview = () => {
    if (!file) return null;
    const ext = file.name.split('.').pop().toLowerCase();
    switch (ext) {
      case 'svg':
        return (
          <img
            src={URL.createObjectURL(file)}
            alt="SVG preview"
            className="h-32 w-auto mx-auto"
          />
        );
      case 'dxf':
      case 'dwg':
        return <p>2D CAD Drawing</p>;
      case 'stl':
        return <p>3D Model (STL)</p>;
      case 'obj':
        return <p>3D Model (OBJ)</p>;
      case 'png':
      case 'jpg':
      case 'jpeg':
        return (
          <img
            src={URL.createObjectURL(file)}
            alt="Image preview"
            className="h-32 w-auto mx-auto"
          />
        );
      case 'gcode':
      case 'nc':
        return <p>G-code Instructions</p>;
      default:
        return <p>Unsupported file</p>;
    }
  };

  return (
    <div className="max-w-lg mx-auto p-4">
      <div
        onDragOver={(e) => e.preventDefault()}
        onDrop={handleDrop}
        className={`border-2 border-dashed rounded-md p-8 text-center cursor-pointer ${error ? 'border-red-500' : ''}`}
        data-testid="drop-zone"
      >
        <input
          id="file-input"
          type="file"
          accept=".svg,.dxf,.dwg,.stl,.obj,.gcode,.nc,.png,.jpg,.jpeg"
          className="hidden"
          onChange={handleInput}
        />
        <label htmlFor="file-input" className="block text-gray-500">
          Drag &amp; drop file here or click to select
        </label>
        {file && (
          <div className="mt-4">
            <p className="font-semibold" data-testid="file-name">{file.name}</p>
            {preview()}
          </div>
        )}
      </div>
      {error && (
        <div className="mt-2 text-red-600" data-testid="error-msg">
          {error} â please retry, use our template or contact support.
        </div>
      )}
      {file && !error && (
        <button
          className="mt-4 bg-blue-600 text-white px-4 py-2 rounded hover:bg-blue-700"
          onClick={() => {
            log(`Continue to editor: ${file.name}`);
            onContinue && onContinue(file);
          }}
        >
          Continue to Toolpath Editor
        </button>
      )}
    </div>
  );
}
