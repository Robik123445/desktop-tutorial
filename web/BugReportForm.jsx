import React, { useState } from 'react';
import log from './logger';

/**
 * Allow users to submit bug reports or suggestions.
 */
export default function BugReportForm() {
  const [text, setText] = useState('');
  const send = () => {
    const msg = text.trim();
    if (!msg) return;
    log(`bug report: ${msg}`);
    alert('Report submitted. Thanks!');
    setText('');
  };
  return (
    <div className="p-2 space-y-2">
      <textarea
        value={text}
        onChange={e => setText(e.target.value)}
        className="w-full p-1 text-black rounded"
        placeholder="Describe the issue"
        data-testid="bug-input"
      />
      <button
        onClick={send}
        className="bg-red-600 text-white px-3 py-1 rounded"
        data-testid="bug-send"
      >
        Submit Bug Report
      </button>
    </div>
  );
}
