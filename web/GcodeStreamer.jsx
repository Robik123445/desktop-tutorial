import React, { useState, useEffect, useRef } from 'react';
import log from './src/log.js';

/**
 * Stream G-code lines in blocks of 5 with basic controls.
 */
export default function GcodeStreamer({ gcodeLines = [], fileName = 'unknown' }) {
  const [index, setIndex] = useState(0); // next line index
  const [streaming, setStreaming] = useState(false);
  const [paused, setPaused] = useState(false);
  const timerRef = useRef(null);
  const [error, setError] = useState('');

  // start streaming lines
  const start = () => {
    if (streaming) return;
    if (!gcodeLines.length) {
      setError('No G-code to stream');
      return;
    }
    log(`Start streaming ${fileName}`);
    setStreaming(true);
    setPaused(false);
    setError('');
  };

  // pause streaming
  const pause = () => {
    if (!streaming) return;
    log('stream paused');
    setPaused(true);
  };

  // stop streaming and reset
  const stop = () => {
    if (!streaming && index === 0) return;
    log('stream stopped');
    clearInterval(timerRef.current);
    setStreaming(false);
    setPaused(false);
    setIndex(0);
  };

  // restart from beginning
  const restart = () => {
    log('stream restarted');
    clearInterval(timerRef.current);
    setIndex(0);
    setPaused(false);
    setStreaming(true);
  };

  // handle streaming timer
  useEffect(() => {
    if (streaming && !paused) {
      timerRef.current = setInterval(() => {
        try {
          setIndex((prev) => {
            const next = Math.min(prev + 5, gcodeLines.length);
            log(`stream lines ${prev}-${next}`);
            if (next === gcodeLines.length) {
              clearInterval(timerRef.current);
              setStreaming(false);
            }
            return next;
          });
        } catch (err) {
          setError('Streaming error');
          log(`Streaming error: ${err.message}`);
          clearInterval(timerRef.current);
          setStreaming(false);
        }
      }, 200);
    }
    return () => clearInterval(timerRef.current);
  }, [streaming, paused, gcodeLines.length]);

  const progress = gcodeLines.length
    ? Math.round((index / gcodeLines.length) * 100)
    : 0;
  const recent = gcodeLines.slice(Math.max(0, index - 3), index);

  return (
    <div className="p-4 space-y-4 max-w-md mx-auto">
      <div className="text-lg font-semibold">{fileName}</div>
      <div>Total lines: {gcodeLines.length}</div>
      <div className="w-full bg-gray-200 h-4 rounded">
        <div
          className="bg-green-500 h-4 rounded"
          style={{ width: `${progress}%` }}
          data-testid="progress-bar"
        />
      </div>
      <div className="flex space-x-2">
        <button
          onClick={start}
          className="bg-blue-600 text-white px-3 py-1 rounded hover:bg-blue-700"
        >
          Start Streaming
        </button>
        <button
          onClick={pause}
          className="bg-yellow-500 text-white px-3 py-1 rounded hover:bg-yellow-600"
        >
          Pause
        </button>
        <button
          onClick={stop}
          className="bg-gray-500 text-white px-3 py-1 rounded hover:bg-gray-600"
        >
          Stop
        </button>
        <button
          onClick={restart}
          className="bg-purple-600 text-white px-3 py-1 rounded hover:bg-purple-700"
        >
          Restart
        </button>
      </div>
      <div>
        <h3 className="font-medium">Last lines</h3>
        <ul className="text-sm font-mono" data-testid="last-lines">
          {recent.map((line, i) => (
            <li key={i}>{line}</li>
          ))}
        </ul>
      </div>
      {error && (
        <div className="text-red-600" data-testid="stream-error">{error}</div>
      )}
    </div>
  );
}
