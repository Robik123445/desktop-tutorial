import React from 'react';
import log from './logger';

/**
 * Display planned robotic arm zones and transitions.
 */
export default function ZoneSummary({ zones = [] }) {
  if (!zones.length) return <div>No zones planned.</div>;
  log(`showing ${zones.length} zones`);
  return (
    <div className="p-4 space-y-2">
      <h4 className="font-medium">Zone Plan</h4>
      <ul className="space-y-1" data-testid="zone-list">
        {zones.map((z, i) => (
          <li key={i} className="border rounded p-2">
            <div>Base: X{z.base[0]} Y{z.base[1]}</div>
            <div>{z.toolpath.length} pts</div>
            {z.move_cmds.map((c, ci) => (
              <div key={ci} className="text-xs text-gray-500">{c}</div>
            ))}
          </li>
        ))}
      </ul>
    </div>
  );
}
