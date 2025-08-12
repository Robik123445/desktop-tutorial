import React, { useMemo, useState, useEffect, useRef } from 'react';
import log from './log.js';

/**
 * Simple G-code preview component.
 *
 * Parses G0/G1 moves and renders them as SVG lines. Users can play or
 * pause the animation and the current move number is displayed.
 */
export default function ToolpathPreview({ gcode = '' }) {
  // Parse lines to coordinate list
  const points = useMemo(() => {
    const coords = [];
    let x = 0,
      y = 0;
    const re = /^(?:G0|G1)\s+.*?(?:X([-+]?\d*\.?\d+))?.*?(?:Y([-+]?\d*\.?\d+))?/i;
    gcode.split(/\r?\n/).forEach((line) => {
      const m = line.match(re);
      if (m) {
        if (m[1] !== undefined) x = parseFloat(m[1]);
        if (m[2] !== undefined) y = parseFloat(m[2]);
        coords.push({ x, y });
      }
    });
    return coords;
  }, [gcode]);

  // Bounding box for scaling
  const bounds = useMemo(() => {
    if (points.length === 0) return { minX: 0, maxX: 0, minY: 0, maxY: 0 };
    let minX = points[0].x,
      maxX = points[0].x,
      minY = points[0].y,
      maxY = points[0].y;
    for (const p of points) {
      if (p.x < minX) minX = p.x;
      if (p.x > maxX) maxX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.y > maxY) maxY = p.y;
    }
    return { minX, maxX, minY, maxY };
  }, [points]);

  const width = 200;
  const height = 200;
  const scale = useMemo(() => {
    const dx = bounds.maxX - bounds.minX || 1;
    const dy = bounds.maxY - bounds.minY || 1;
    return Math.min(width / dx, height / dy);
  }, [bounds]);

  // Animation state
  const [index, setIndex] = useState(0);
  const [playing, setPlaying] = useState(false);
  const timer = useRef(null);

  useEffect(() => {
    if (!playing) return;
    timer.current = setInterval(() => {
      setIndex((i) => {
        const next = i + 1;
        if (next >= points.length) {
          clearInterval(timer.current);
          return i;
        }
        return next;
      });
    }, 300);
    return () => clearInterval(timer.current);
  }, [playing, points.length]);

  const toggle = () => {
    setPlaying((p) => {
      const next = !p;
      log(next ? 'play' : 'pause');
      return next;
    });
  };

  const polyPoints = points
    .map((p) => [
      ((p.x - bounds.minX) * scale).toFixed(2),
      (height - (p.y - bounds.minY) * scale).toFixed(2),
    ].join(','))
    .join(' ');

  return (
    <div className="space-y-2">
      <svg
        data-testid="preview-svg"
        width={width}
        height={height}
        className="border border-gray-300 bg-white"
      >
        <polyline points={polyPoints} stroke="orange" strokeWidth="1" fill="none" />
        {points[index] && (
          <circle
            cx={((points[index].x - bounds.minX) * scale).toFixed(2)}
            cy={(height - (points[index].y - bounds.minY) * scale).toFixed(2)}
            r="3"
            fill="red"
          />
        )}
      </svg>
      <div className="flex items-center space-x-2">
        <button onClick={toggle} className="px-2 py-1 bg-blue-600 text-white rounded">
          {playing ? 'Pause' : 'Play'}
        </button>
        <span>
          Move {Math.min(index + 1, points.length)} / {points.length}
        </span>
      </div>
    </div>
  );
}
