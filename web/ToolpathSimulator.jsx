import React, { useRef, useState, useEffect, useMemo } from 'react';
import log from './logger';

/**
 * Simulate toolpath motion over time on a canvas.
 * Accepts raw G-code string and displays tool position with play/pause/step controls.
 */
export default function ToolpathSimulator({ gcode = '' }) {
  // Parse G-code lines to coordinate points
  const points = useMemo(() => {
    const pts = [];
    let x = 0, y = 0, z = 0;
    gcode.split(/\r?\n/).forEach((line) => {
      if (!/^G0|^G1/.test(line)) return;
      const mx = line.match(/X([-0-9.]+)/i);
      const my = line.match(/Y([-0-9.]+)/i);
      const mz = line.match(/Z([-0-9.]+)/i);
      if (mx) x = parseFloat(mx[1]);
      if (my) y = parseFloat(my[1]);
      if (mz) z = parseFloat(mz[1]);
      pts.push({ x, y, z });
    });
    return pts;
  }, [gcode]);

  const canvasRef = useRef(null);
  const [index, setIndex] = useState(0);
  const [running, setRunning] = useState(false);
  const timerRef = useRef(null);

  // Draw toolpath and current position
  const draw = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.strokeStyle = '#ccc';
    ctx.beginPath();
    points.forEach((p, i) => {
      const sx = p.x + canvas.width / 2;
      const sy = canvas.height / 2 - p.y;
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.stroke();
    const cur = points[index] || points[points.length - 1];
    if (!cur) return;
    const cx = cur.x + canvas.width / 2;
    const cy = canvas.height / 2 - cur.y;
    ctx.fillStyle = 'red';
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();
  };

  useEffect(draw, [index, points]);

  // Animation timer
  useEffect(() => {
    if (running) {
      timerRef.current = setInterval(() => {
        setIndex((i) => {
          const next = Math.min(i + 1, points.length - 1);
          if (next === points.length - 1) setRunning(false);
          return next;
        });
      }, 300);
    }
    return () => clearInterval(timerRef.current);
  }, [running, points.length]);

  const play = () => {
    if (!running) {
      log('simulation start');
      setRunning(true);
    }
  };
  const pause = () => {
    if (running) {
      log('simulation pause');
      setRunning(false);
    }
  };
  const step = () => {
    setIndex((i) => Math.min(i + 1, points.length - 1));
    log('simulation step');
  };

  const cur = points[index] || { x: 0, y: 0, z: 0 };

  return (
    <div className="space-y-2 p-4 max-w-md mx-auto">
      <canvas
        ref={canvasRef}
        width={300}
        height={300}
        className="border w-full"
        data-testid="sim-canvas"
      />
      <div className="flex space-x-2">
        <button onClick={play} className="bg-green-600 text-white px-3 py-1 rounded hover:bg-green-700">Play</button>
        <button onClick={pause} className="bg-yellow-600 text-white px-3 py-1 rounded hover:bg-yellow-700">Pause</button>
        <button onClick={step} className="bg-blue-600 text-white px-3 py-1 rounded hover:bg-blue-700">Step</button>
      </div>
      <div className="text-sm" data-testid="coords">
        X:{cur.x} Y:{cur.y} Z:{cur.z}
      </div>
    </div>
  );
}
