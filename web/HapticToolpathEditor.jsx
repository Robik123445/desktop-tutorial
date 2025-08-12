import React, { useState, useRef } from 'react';
import log from './src/log.js';
import ToolpathSimulator from './ToolpathSimulator';

/**
 * HapticToolpathEditor
 * --------------------
 * Freehand drawing editor with layer and history support.
 * Paths are stored as arrays of points [{x,y,z,pressure}] and rendered
 * live while drawing. Actions are logged to logs/central.log.
 */
export default function HapticToolpathEditor({ width = 800, height = 600 }) {
  const [layers, setLayers] = useState([]); // {id,name,visible,paths:[]}
  const [currentLayer, setCurrentLayer] = useState(null);
  const [drawing, setDrawing] = useState(false);
  const [history, setHistory] = useState([]);
  const [future, setFuture] = useState([]);
  const svgRef = useRef(null);
  const [simPath, setSimPath] = useState(null);

  // push current state to history
  const pushHistory = (newLayers) => {
    setHistory((h) => [...h, JSON.parse(JSON.stringify(layers))]);
    setFuture([]);
    setLayers(newLayers);
  };

  const startLayer = () => {
    const layer = {
      id: Date.now(),
      name: `Layer ${layers.length + 1}`,
      visible: true,
      paths: [],
    };
    pushHistory([...layers, layer]);
    setCurrentLayer(layer.id);
    log('new layer created');
  };

  const addPoint = (x, y, p) => {
    setLayers((ls) =>
      ls.map((l) => {
        if (l.id !== currentLayer) return l;
        const paths = [...l.paths];
        if (!paths.length || !drawing) paths.push([]);
        const curr = paths[paths.length - 1];
        curr.push({ x, y, p });
        return { ...l, paths };
      })
    );
  };

  const handlePointerDown = (e) => {
    if (currentLayer == null) startLayer();
    setDrawing(true);
    const rect = svgRef.current.getBoundingClientRect();
    addPoint(e.clientX - rect.left, e.clientY - rect.top, e.pressure || 0);
  };

  const handlePointerMove = (e) => {
    if (!drawing) return;
    const rect = svgRef.current.getBoundingClientRect();
    addPoint(e.clientX - rect.left, e.clientY - rect.top, e.pressure || 0);
  };

  const stopDrawing = () => {
    if (drawing) {
      log('path drawn');
      setDrawing(false);
      pushHistory([...layers]);
    }
  };

  const undo = () => {
    setHistory((h) => {
      if (!h.length) return h;
      const prev = h[h.length - 1];
      setFuture((f) => [JSON.parse(JSON.stringify(layers)), ...f]);
      setLayers(prev);
      return h.slice(0, -1);
    });
    log('undo');
  };

  const redo = () => {
    setFuture((f) => {
      if (!f.length) return f;
      const next = f[0];
      setHistory((h) => [...h, JSON.parse(JSON.stringify(layers))]);
      setLayers(next);
      return f.slice(1);
    });
    log('redo');
  };

  const exportGcode = () => {
    const path = layers
      .filter((l) => l.visible)
      .flatMap((l) => l.paths)
      .flat();
    setSimPath(path);
    log('export G-code');
  };

  return (
    <div className="flex">
      <div className="w-64 p-2 space-y-2 border-r">
        <button
          className="bg-blue-600 text-white px-2 py-1 rounded"
          onClick={startLayer}
        >
          New Layer
        </button>
        <button className="bg-gray-300 px-2 py-1 rounded" onClick={undo}>
          Undo
        </button>
        <button className="bg-gray-300 px-2 py-1 rounded" onClick={redo}>
          Redo
        </button>
        <button
          className="bg-green-600 text-white px-2 py-1 rounded"
          onClick={exportGcode}
        >
          Export as G-code
        </button>
        <div>
          {layers.map((l) => (
            <div key={l.id} className="flex items-center space-x-1">
              <input
                type="checkbox"
                checked={l.visible}
                onChange={() =>
                  setLayers((ls) =>
                    ls.map((o) => (o.id === l.id ? { ...o, visible: !o.visible } : o))
                  )
                }
              />
              <span
                className={
                  'cursor-pointer ' + (currentLayer === l.id ? 'font-bold' : '')
                }
                onClick={() => setCurrentLayer(l.id)}
              >
                {l.name}
              </span>
            </div>
          ))}
        </div>
      </div>
      <div className="flex-1 relative" onPointerUp={stopDrawing}>
        <svg
          ref={svgRef}
          width={width}
          height={height}
          className="bg-gray-100 touch-none"
          onPointerDown={handlePointerDown}
          onPointerMove={handlePointerMove}
        >
          {layers.map((l) =>
            l.visible ? (
              <g key={l.id}>
                {l.paths.map((p, i) => (
                  <polyline
                    key={i}
                    points={p.map((pt) => `${pt.x},${pt.y}`).join(' ')}
                    fill="none"
                    stroke="red"
                    strokeWidth={2}
                  />
                ))}
              </g>
            ) : null
          )}
        </svg>
        {simPath && (
          <div className="absolute inset-0 bg-white bg-opacity-90 flex">
            <div className="m-auto bg-white p-4 shadow">
              <ToolpathSimulator gcode={simPath.map((pt) => `G1 X${pt.x} Y${pt.y}`)} />
              <button
                className="bg-gray-300 px-2 py-1 mt-2 rounded"
                onClick={() => setSimPath(null)}
              >
                Close
              </button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
