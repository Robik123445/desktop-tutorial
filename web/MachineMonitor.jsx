import React, { useEffect, useState } from 'react';
import log from './src/log.js';

/**
 * MachineMonitor
 * ---------------
 * Display dummy machine status that updates every second.
 * Real implementation would poll a controller API.
 */
export default function MachineMonitor() {
  const [status, setStatus] = useState({ x: 0, y: 0, z: 0, feed: 0, state: 'Idle' });

  useEffect(() => {
    const timer = setInterval(() => {
      setStatus((s) => {
        const next = { ...s, x: s.x + 1, y: s.y + 1, feed: 1000, state: 'Running' };
        log(`monitor update ${JSON.stringify(next)}`);
        return next;
      });
    }, 1000);
    return () => clearInterval(timer);
  }, []);

  return (
    <div className="space-y-1 text-sm">
      <div>X: {status.x.toFixed(1)}</div>
      <div>Y: {status.y.toFixed(1)}</div>
      <div>Z: {status.z.toFixed(1)}</div>
      <div>Feed: {status.feed} mm/min</div>
      <div>Status: {status.state}</div>
    </div>
  );
}
