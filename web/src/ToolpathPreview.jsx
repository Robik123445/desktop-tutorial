import React, { useMemo, useState, useEffect, useRef } from 'react';
import log from './log.js';

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
