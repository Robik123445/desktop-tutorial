import React, { useEffect, useState } from 'react';
import log from './logger';

/**
 * Show details from last_block.json and allow resuming.
 */
export default function ResumeJobButton({ onResume, checkpoint = 'last_block.json' }) {
  const [info, setInfo] = useState(null);

  useEffect(() => {
    fetch(checkpoint)
      .then((r) => r.json())
      .then(setInfo)
      .catch(() => setInfo(null));
  }, [checkpoint]);

  const resume = () => {
    log('resume job clicked');
    if (onResume) onResume(info);
  };

  if (!info) {
    return (
      <button disabled className="bg-gray-400 text-white px-3 py-1 rounded">
        No checkpoint
      </button>
    );
  }

  return (
    <div className="space-y-2">
      <div className="text-sm">Last line: {info.line}</div>
      <button
        onClick={resume}
        className="bg-green-600 text-white px-3 py-1 rounded hover:bg-green-700"
      >
        Resume interrupted job
      </button>
    </div>
  );
}
