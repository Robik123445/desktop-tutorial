import React, { useState, useRef } from 'react';
import FileUploader from './FileUploader';
import ToolpathTransform from './ToolpathTransform';
import log from './logger';

/**
 * Advanced Vector Editor
 * ----------------------
 * Supports basic node and path editing with a layer system. This is a light
 * frontâend mock so vector operations are simplified, but the structure allows
 * future extension. All actions are appended to `logs/central.log`.
 */
export default function VectorEditor() {
  const [file, setFile] = useState(null);
  const [layers, setLayers] = useState([]); // {id,name,visible,locked,paths:[]}
  const [selectedLayer, setSelectedLayer] = useState(null);
  const [selectedPath, setSelectedPath] = useState(null);
  const [selectedNode, setSelectedNode] = useState(null); // {pathId,index}
  const [history, setHistory] = useState([]);
  const [future, setFuture] = useState([]);
  const svgRef = useRef(null);
  const [threshold, setThreshold] = useState(128);
  const [mode, setMode] = useState('outline'); // outline or centerline

  const pushHistory = (newLayers) => {
    setHistory((h) => [...h, JSON.parse(JSON.stringify(layers))]);
    setFuture([]);
    setLayers(newLayers);
  };

  // convert points to SVG path string
  const makeD = (pts, closed) => {
    if (!pts.length) return '';
    const start = `M${pts[0].x} ${pts[0].y}`;
    const rest = pts.slice(1).map((p) => `L${p.x} ${p.y}`).join(' ');
    return closed ? `${start} ${rest} Z` : `${start} ${rest}`;
  };

  // simple rectangle vectorization as placeholder
  const loadOpenCV = () =>
    new Promise((resolve) => {
      if (window.cv) return resolve();
      const script = document.createElement('script');
      script.src =
        'https://docs.opencv.org/4.x/opencv.js';
      script.onload = resolve;
      document.body.appendChild(script);
    });

  const vectorize = async () => {
    if (!file) return;
    await loadOpenCV();
    const img = new Image();
    img.onload = () => {
      const canvas = document.createElement('canvas');
      canvas.width = img.width;
      canvas.height = img.height;
      const ctx = canvas.getContext('2d');
      ctx.drawImage(img, 0, 0);
      const src = cv.imread(canvas);
      cv.cvtColor(src, src, cv.COLOR_RGBA2GRAY, 0);
      cv.threshold(src, src, threshold, 255, cv.THRESH_BINARY);
      const contours = new cv.MatVector();
      const hierarchy = new cv.Mat();
      if (mode === 'outline') {
        cv.findContours(
          src,
          contours,
          hierarchy,
          cv.RETR_EXTERNAL,
          cv.CHAIN_APPROX_SIMPLE
        );
      } else {
        cv.Canny(src, src, 50, 150, 3, false);
        cv.findContours(src, contours, hierarchy, cv.RETR_LIST, cv.CHAIN_APPROX_NONE);
      }
      const newPaths = [];
      for (let i = 0; i < contours.size(); i++) {
        const cnt = contours.get(i);
        const pts = [];
        for (let j = 0; j < cnt.data32S.length / 2; j++) {
          pts.push({ x: cnt.data32S[j * 2], y: cnt.data32S[j * 2 + 1] });
        }
        cnt.delete();
        newPaths.push({
          id: Date.now() + i,
          points: pts,
          closed: true,
          tx: 0,
          ty: 0,
          rot: 0,
          scale: 1,
          visible: true,
        });
      }
      src.delete();
      contours.delete();
      hierarchy.delete();
      const layer = {
        id: Date.now(),
        name: file.name,
        visible: true,
        locked: false,
        paths: newPaths,
      };
      pushHistory([...layers, layer]);
      setSelectedLayer(layer.id);
      if (newPaths.length) setSelectedPath(newPaths[0].id);
      log(`vectorized ${file.name} (${mode})`);
    };
    img.src = URL.createObjectURL(file);
  };

  const findLayer = (layerId) => layers.find((l) => l.id === layerId);
  const findPath = (layer, pathId) => layer.paths.find((p) => p.id === pathId);

  const updatePath = (layerId, pathId, updater) => {
    const newLayers = layers.map((l) => {
      if (l.id !== layerId) return l;
      const newPaths = l.paths.map((p) =>
        p.id === pathId ? updater({ ...p }) : p
      );
      return { ...l, paths: newPaths };
    });
    pushHistory(newLayers);
  };

  const updateSelectedTransform = (t) => {
    if (!selectedLayer || !selectedPath) return;
    updatePath(selectedLayer, selectedPath, (p) => ({
      ...p,
      tx: t.x,
      ty: t.y,
      rot: t.rotation,
      scale: t.scale,
    }));
    log(`transform path ${selectedPath}`);
  };

  const duplicatePath = () => {
    if (!selectedLayer || !selectedPath) return;
    const layer = findLayer(selectedLayer);
    const path = findPath(layer, selectedPath);
    const copy = { ...path, id: Date.now() };
    const newLayers = layers.map((l) =>
      l.id === selectedLayer ? { ...l, paths: [...l.paths, copy] } : l
    );
    pushHistory(newLayers);
    log(`duplicate path ${selectedPath}`);
  };

  const mirrorPath = () => {
    if (!selectedLayer || !selectedPath) return;
    updatePath(selectedLayer, selectedPath, (p) => {
      const pts = p.points.map((pt) => ({ x: -pt.x, y: pt.y }));
      return { ...p, points: pts };
    });
    log(`mirror path ${selectedPath}`);
  };

  const addNodeAt = (x, y) => {
    if (!selectedLayer || !selectedPath) return;
    updatePath(selectedLayer, selectedPath, (p) => {
      const pts = [...p.points, { x, y }];
      return { ...p, points: pts };
    });
    log(`add node to ${selectedPath}`);
  };

  const deleteNode = () => {
    if (!selectedNode) return;
    const { pathId, index } = selectedNode;
    updatePath(selectedLayer, pathId, (p) => {
      const pts = p.points.filter((_, i) => i !== index);
      return { ...p, points: pts };
    });
    setSelectedNode(null);
    log(`delete node ${index} from ${pathId}`);
  };

  const moveNode = (index, dx, dy) => {
    updatePath(selectedLayer, selectedPath, (p) => {
      const pts = p.points.map((pt, i) =>
        i === index ? { x: pt.x + dx, y: pt.y + dy } : pt
      );
      return { ...p, points: pts };
    });
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

  const deletePath = () => {
    if (!selectedLayer || !selectedPath) return;
    const newLayers = layers.map((l) =>
      l.id === selectedLayer
        ? { ...l, paths: l.paths.filter((p) => p.id !== selectedPath) }
        : l
    );
    pushHistory(newLayers);
    setSelectedPath(null);
    log(`delete path ${selectedPath}`);
  };

  // derived visible paths
  const visiblePaths = layers.flatMap((layer) =>
    layer.visible
      ? layer.paths
          .filter((p) => p.visible)
          .map((p) => ({ ...p, layerId: layer.id }))
      : []
  );

  // handle simple node drag
  const handleMouseDown = (pathId, index, e) => {
    e.stopPropagation();
    setSelectedNode({ pathId, index });
  };

  const handleMouseMove = (e) => {
    if (!selectedNode) return;
    const svg = svgRef.current.getBoundingClientRect();
    const x = e.clientX - svg.left;
    const y = e.clientY - svg.top;
    const path = findPath(findLayer(selectedLayer), selectedPath);
    const pt = path.points[selectedNode.index];
    moveNode(selectedNode.index, x - (pt.x + path.tx), y - (pt.y + path.ty));
  };

  const handleMouseUp = () => {
    if (selectedNode) log(`move node ${selectedNode.index}`);
    setSelectedNode(null);
  };

  return (
    <div className="flex" onMouseMove={handleMouseMove} onMouseUp={handleMouseUp}>
      <div className="w-72 p-4 space-y-4 border-r overflow-y-auto h-screen">
        {!file && <FileUploader onContinue={(f) => setFile(f)} />}
        {file && (
          <>
            <div className="space-y-2">
              <label className="block text-sm">Threshold: {threshold}
                <input
                  type="range"
                  min="0"
                  max="255"
                  value={threshold}
                  onChange={(e) => setThreshold(parseInt(e.target.value, 10))}
                  className="w-full"
                />
              </label>
              <div className="space-x-2 text-sm">
                <label>
                  <input
                    type="radio"
                    value="outline"
                    checked={mode === 'outline'}
                    onChange={(e) => setMode(e.target.value)}
                  />{' '}
                  Outline
                </label>
                <label>
                  <input
                    type="radio"
                    value="centerline"
                    checked={mode === 'centerline'}
                    onChange={(e) => setMode(e.target.value)}
                  />{' '}
                  Centerline
                </label>
              </div>
              <button
                className="bg-blue-600 text-white px-3 py-1 rounded"
                onClick={vectorize}
              >
                Vectorize
              </button>
              <div className="space-x-1">
                <button
                  className="bg-gray-300 px-2 py-1 rounded"
                  onClick={undo}
                >
                  Undo
                </button>
                <button
                  className="bg-gray-300 px-2 py-1 rounded"
                  onClick={redo}
                >
                  Redo
                </button>
              </div>
            </div>
            {selectedPath && (
              <>
                <ToolpathTransform onChange={updateSelectedTransform} />
                <div className="space-x-1">
                  <button
                    className="bg-gray-300 px-2 py-1 rounded"
                    onClick={duplicatePath}
                  >
                    Duplicate
                  </button>
                  <button
                    className="bg-gray-300 px-2 py-1 rounded"
                    onClick={mirrorPath}
                  >
                    Mirror H
                  </button>
                  <button
                    className="bg-gray-300 px-2 py-1 rounded"
                    onClick={deletePath}
                  >
                    Delete Path
                  </button>
                  <button
                    className="bg-gray-300 px-2 py-1 rounded"
                    onClick={deleteNode}
                  >
                    Delete Node
                  </button>
                </div>
              </>
            )}
            <div className="pt-4">
              <h4 className="font-semibold">Layers</h4>
              {layers.map((l) => (
                <div key={l.id} className="flex items-center space-x-1">
                  <input
                    type="checkbox"
                    checked={l.visible}
                    onChange={() =>
                      pushHistory(
                        layers.map((o) =>
                          o.id === l.id ? { ...o, visible: !o.visible } : o
                        )
                      )
                    }
                  />
                  <input
                    type="checkbox"
                    checked={l.locked}
                    onChange={() =>
                      pushHistory(
                        layers.map((o) =>
                          o.id === l.id ? { ...o, locked: !o.locked } : o
                        )
                      )
                    }
                  />
                  <span
                    onClick={() => setSelectedLayer(l.id)}
                    className={
                      'cursor-pointer ' + (selectedLayer === l.id ? 'font-bold' : '')
                    }
                  >
                    {l.name}
                  </span>
                </div>
              ))}
            </div>
          </>
        )}
        <p>Paths: {visiblePaths.length}</p>
      </div>
      <div className="flex-1 relative bg-gray-100">
        {file && (
          <svg
            ref={svgRef}
            className="w-full h-full"
            viewBox="0 0 500 500"
            onClick={(e) => {
              if (!selectedPath) return;
              const rect = svgRef.current.getBoundingClientRect();
              const x = e.clientX - rect.left;
              const y = e.clientY - rect.top;
              addNodeAt(x, y);
            }}
          >
            {visiblePaths.map((p) => (
              <g
                key={p.id}
                transform={`translate(${p.tx} ${p.ty}) scale(${p.scale}) rotate(${p.rot})`}
                onClick={(e) => {
                  e.stopPropagation();
                  setSelectedLayer(p.layerId);
                  setSelectedPath(p.id);
                  setSelectedNode(null);
                }}
              >
                <path d={makeD(p.points, p.closed)} stroke="red" fill="none" />
                {selectedPath === p.id &&
                  p.points.map((pt, idx) => (
                    <circle
                      key={idx}
                      cx={pt.x}
                      cy={pt.y}
                      r={4}
                      fill={
                        selectedNode &&
                        selectedNode.pathId === p.id &&
                        selectedNode.index === idx
                          ? 'blue'
                          : 'white'
                      }
                      stroke="blue"
                      onMouseDown={(e) => handleMouseDown(p.id, idx, e)}
                    />
                  ))}
              </g>
            ))}
          </svg>
        )}
      </div>
    </div>
  );
}
