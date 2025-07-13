import React, { useMemo, useEffect, useRef, useState } from 'react';
import log from './logger';

/**
 * Analyze G-code against a heightmap and highlight problem areas.
 * Collisions appear in red while segments above clearance are blue.
 */
export default function SurfaceAnalyzer({ gcode = '', heightmap = { points: [] }, clearance = 0.1 }) {
  // Parse G-code lines into coordinate points
  const points = useMemo(() => {
    const pts = [];
    let x = 0, y = 0, z = 0;
    gcode.split(/\r?\n/).forEach(line => {
      line = line.trim();
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

  // Build lookup table for heightmap Z values
  const hmap = useMemo(() => {
    const m = new Map();
    (heightmap.points || []).forEach(p => {
      m.set(`${p.x},${p.y}`, parseFloat(p.z));
    });
    return m;
  }, [heightmap]);

  const canvasRef = useRef(null);
  const [issues, setIssues] = useState([]);

  // Draw overlay with color-coded markers
  const draw = (cols, high, ok) => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const cx = canvas.width / 2;
    const cy = canvas.height / 2;

    ctx.strokeStyle = '#666';
    ctx.beginPath();
    points.forEach((p, i) => {
      const sx = p.x + cx;
      const sy = cy - p.y;
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.stroke();

    function drawDots(arr, color) {
      ctx.fillStyle = color;
      arr.forEach(p => {
        const sx = p.x + cx;
        const sy = cy - p.y;
        ctx.beginPath();
        ctx.arc(sx, sy, 3, 0, Math.PI * 2);
        ctx.fill();
      });
    }

    drawDots(ok, 'green');
    drawDots(high, 'blue');
    drawDots(cols, 'red');
  };

  // Analyze when inputs change
  useEffect(() => {
    const cols = [];
    const high = [];
    const ok = [];
    const issuesList = [];

    points.forEach(p => {
      const surf = hmap.get(`${p.x},${p.y}`) ?? 0;
      const diff = p.z - surf;
      if (diff < 0) {
        cols.push(p);
        issuesList.push({ type: 'COLLISION', x: p.x, y: p.y, diff });
      } else if (diff > clearance) {
        high.push(p);
        issuesList.push({ type: 'NO_CONTACT', x: p.x, y: p.y, diff });
      } else {
        ok.push(p);
      }
    });

    setIssues(issuesList);
    draw(cols, high, ok);
    log(`analysis complete: ${cols.length} collisions, ${high.length} high points`);
  }, [points, hmap, clearance]);

  return (
    <div className="space-y-2 p-4 max-w-md mx-auto">
      <canvas ref={canvasRef} width={300} height={300} className="border w-full" />
      <div className="text-sm font-medium">Analysis Results:</div>
      <ul className="text-sm list-disc pl-4" data-testid="issues">
        {issues.length === 0 && <li>No issues found</li>}
        {issues.map((i, idx) => (
          <li key={idx}>{`${i.type} at X${i.x} Y${i.y} Î${i.diff.toFixed(2)}`}</li>
        ))}
      </ul>
    </div>
  );
}
