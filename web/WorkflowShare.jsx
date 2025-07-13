import React, { useRef } from 'react';
import { saveAs } from 'file-saver';
import log from './logger';

/**
 * Allow exporting and importing workflows as JSON files.
 */
export default function WorkflowShare({ workflow = {}, onImport }) {
  const inputRef = useRef(null);

  const save = () => {
    const blob = new Blob([JSON.stringify(workflow, null, 2)], { type: 'application/json' });
    saveAs(blob, 'workflow.json');
    log('export workflow');
  };

  const load = (e) => {
    const file = e.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const data = JSON.parse(reader.result);
        log('import workflow');
        onImport && onImport(data);
      } catch (err) {
        log('failed import workflow', 'ERROR', err);
      }
    };
    reader.readAsText(file);
  };

  return (
    <div className="space-x-2 p-2">
      <button onClick={save} className="bg-green-600 text-white px-3 py-1 rounded">Share Workflow</button>
      <input ref={inputRef} data-testid="wf-input" type="file" accept="application/json" onChange={load} className="hidden" />
      <button onClick={() => inputRef.current && inputRef.current.click()} className="bg-blue-600 text-white px-3 py-1 rounded">Import Workflow</button>
    </div>
  );
}
