import React, { useState, useEffect } from 'react';
import log from './src/log.js';

// Supported operations and available tool heads
const OPERATIONS = ['Cut', 'Engrave', 'Mill', 'Print'];
const TOOL_HEADS = ['Laser', 'Spindle', 'Print Head'];
// Default head for each operation
const DEFAULT_MAP = {
  Cut: 'Spindle',
  Engrave: 'Laser',
  Mill: 'Spindle',
  Print: 'Print Head',
};

/**
 * OperationAssigner
 * -----------------
 * Assign operations and tool heads to imported vector paths.
 * Selection changes are logged to logs/central.log and a summary is displayed.
 */
export default function OperationAssigner({ paths = [], onChange }) {
  const [assignments, setAssignments] = useState(() =>
    paths.map(() => ({ operation: 'Cut', head: DEFAULT_MAP.Cut }))
  );

  // reset assignments when path count changes
  useEffect(() => {
    setAssignments(paths.map(() => ({ operation: 'Cut', head: DEFAULT_MAP.Cut })));
  }, [paths.length]);

  // notify parent on change
  useEffect(() => {
    if (onChange) onChange(assignments);
  }, [assignments, onChange]);

  const setOperation = (idx, op) => {
    setAssignments((prev) => {
      const next = [...prev];
      next[idx] = { operation: op, head: DEFAULT_MAP[op] };
      log(`Path ${idx} operation -> ${op}`);
      return next;
    });
  };

  const setHead = (idx, head) => {
    setAssignments((prev) => {
      const next = [...prev];
      next[idx] = { ...next[idx], head };
      log(`Path ${idx} head -> ${head}`);
      return next;
    });
  };

  const summary = assignments.reduce(
    (acc, a) => {
      acc.heads[a.head] = (acc.heads[a.head] || 0) + 1;
      acc.ops[a.operation] = (acc.ops[a.operation] || 0) + 1;
      return acc;
    },
    { heads: {}, ops: {} }
  );

  return (
    <div className="p-4 space-y-4 max-w-md mx-auto">
      {paths.map((p, idx) => (
        <div key={idx} className="border rounded p-2 space-y-1">
          <div className="font-medium">Path {idx + 1}</div>
          <div className="flex space-x-2">
            <select
              value={assignments[idx]?.operation}
              onChange={(e) => setOperation(idx, e.target.value)}
              className="border rounded p-1"
              data-testid={`op-${idx}`}
            >
              {OPERATIONS.map((o) => (
                <option key={o} value={o}>
                  {o}
                </option>
              ))}
            </select>
            <select
              value={assignments[idx]?.head}
              onChange={(e) => setHead(idx, e.target.value)}
              className="border rounded p-1"
              data-testid={`head-${idx}`}
            >
              {TOOL_HEADS.map((h) => (
                <option key={h} value={h}>
                  {h}
                </option>
              ))}
            </select>
          </div>
        </div>
      ))}
      <div className="border-t pt-2">
        <h3 className="font-medium">Summary</h3>
        <div className="flex space-x-8 text-sm" data-testid="summary">
          <ul>
            {OPERATIONS.map((o) => (
              <li key={o}>
                {o}: {summary.ops[o] || 0}
              </li>
            ))}
          </ul>
          <ul>
            {TOOL_HEADS.map((h) => (
              <li key={h}>
                {h}: {summary.heads[h] || 0}
              </li>
            ))}
          </ul>
        </div>
      </div>
    </div>
  );
}
