import React, { useMemo, useState, useEffect, useRef } from 'react';
import log from '../log';

/**
 * Simple G-code previewer.
 *
 * Parses G0/G1 moves and renders them as lines in an SVG. The preview
 * can step through moves using Play/Pause buttons and logs actions to log.txt.
 */
export default function ToolpathPreview({ gcode = '' }) {
  // --- Parse G-code into XY coordinate list ---------------------------------
  const points = useMemo(() => {
    const coords = [];
    let x = 0, y = 0;
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

  // --- Calculate bounding box for scaling ----------------------------------
  const bounds = useMemo(() => {
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    points.forEach(({ x, y }) => {
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y);
    });
    if (!isFinite(minX)) {
      minX = minY = 0;
      maxX = maxY = 1;
    }
    return { minX, minY, maxX, maxY };
  }, [points]);

  const scale = 380 / Math.max(bounds.maxX - bounds.minX || 1, bounds.maxY - bounds.minY || 1);
  const offX = -bounds.minX;
  const offY = -bounds.minY;

  const [playing, setPlaying] = useState(false);
  const [index, setIndex] = useState(0);
  const timerRef = useRef(null);

  // Reset animation when gcode changes
  useEffect(() => setIndex(0), [gcode]);

  // Handle play/pause state
  useEffect(() => {
    if (playing) {
      log('play');
      timerRef.current = setInterval(() => {
        setIndex((i) => Math.min(i + 1, points.length - 1));
      }, 300);
    } else {
      if (timerRef.current) clearInterval(timerRef.current);
      log('pause');
    }
    return () => timerRef.current && clearInterval(timerRef.current);
  }, [playing, points.length]);

  return (
    <div className="p-4">
      <div className="flex items-center gap-2 mb-2">
        <button
          onClick={() => setPlaying((p) => !p)}
          className="px-3 py-1 rounded bg-blue-600 text-white"
        >
          {playing ? 'Pause' : 'Play'}
        </button>
        <span className="text-sm">
          Move {index + 1} / {points.length}
        </span>
      </div>
      <svg width={400} height={400} className="border">
        {points.slice(0, index + 1).map((p, i) => {
          const next = points[i + 1];
          if (!next) return null;
          return (
            <line
              key={i}
              x1={(p.x + offX) * scale}
              y1={(p.y + offY) * scale}
              x2={(next.x + offX) * scale}
              y2={(next.y + offY) * scale}
              stroke="lime"
            />
          );
        })}
      </svg>
    </div>
  );
}
