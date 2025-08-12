import React, { useState, useCallback } from 'react';
import log from './src/log.js';
import ToolpathTransform from './ToolpathTransform';
import OperationAssigner from './OperationAssigner';
import ToolpathOptimizer from './ToolpathOptimizer';
import ToolpathSimulator from './ToolpathSimulator';
import SurfaceAnalyzer from './SurfaceAnalyzer';
import GcodeStreamer from './GcodeStreamer';
import MachineMonitor from './MachineMonitor';
import PluginManager from './PluginManager';
import DiagnosticsPanel from './DiagnosticsPanel';
import WorkspaceScanner from './WorkspaceScanner';
import RobotArmPanel from './RobotArmPanel';
import * as THREE from 'three';

/**
 * CuraStyleUI
 * ------------
 * Main layout for the CAM slicer inspired by Cura.
 * Sidebar sections are collapsible; the right panel shows
 * parameters for the currently selected section.
 */

// Sidebar sections with simple emoji icons
const SECTIONS = [
  { name: 'Import & Model', icon: 'ð¥' },
  { name: 'Tools & Operations', icon: 'ð ï¸' },
  { name: 'Hybrid Operations', icon: 'ð' },
  { name: 'Toolpath Optimization', icon: 'âï¸' },
  { name: 'Surface & Sensors', icon: 'ð' },
  { name: 'Simulation', icon: 'ðï¸' },
  { name: 'AI Analyze & Optimize', icon: 'ð¤' },
  { name: 'Export & Streaming', icon: 'ð¤' },
  { name: 'Monitoring', icon: 'ð¡' },
  { name: 'Experimental features', icon: 'ð§ª' },
  { name: 'Experimental AI', icon: 'ð°ï¸' },
  { name: 'Experimental â Robot Arm', icon: 'ð¤' },
  { name: 'Diagnostics', icon: 'ð©º' },
];

// Placeholder options per section
const OPTIONS = {
  'Import & Model': ['Upload', 'Vectorize'],
  'Tools & Operations': ['Assign Ops'],
  'Hybrid Operations': ['Preview Heads'],
  'Toolpath Optimization': [
    'Optimize sequence',
    'Smooth corners',
    'Remove air moves',
    'Pressure advance',
    'Input shaping',
  ],
  'Surface & Sensors': ['Probe', 'Heightmap'],
  Simulation: ['Preview simulation'],
  'AI Analyze & Optimize': ['Run AI Analysis', 'Auto Optimize'],
  'Export & Streaming': [
    'Debris scan & avoid',
    'Export GRBL',
    'Export Marlin',
    'Export Klipper',
    'Custom Post',
    'Stream',
    'Dry-run',
  ],
  Monitoring: ['Live status'],
  'Experimental features': ['Adaptive Toolpath'],
  'Experimental AI': ['SurfaceAnalyzer', 'ToolpathOptimizer', 'AI Feedback'],
  'Experimental â Robot Arm': ['Robot Arm Mode'],
  Diagnostics: ['Report'],
};

export default function CuraStyleUI() {
  const [open, setOpen] = useState({});
  const [active, setActive] = useState(null);
  const [tool, setTool] = useState('Router');
  const [file, setFile] = useState(null);
  const [transform, setTransform] = useState({ rotation: 0, scale: 1, x: 0, y: 0 });
  const [paths, setPaths] = useState([]); // simplified vector paths
  const [assignments, setAssignments] = useState([]);
  const [params, setParams] = useState([]); // feedrate/depth per path
  const [adaptiveMode, setAdaptiveMode] = useState(false);

  const toggle = (name) => {
    setOpen((prev) => ({ ...prev, [name]: !prev[name] }));
    setActive(name);
    log(`section selected: ${name}`);
  };

  const handleDragOver = useCallback((e) => {
    e.preventDefault();
  }, []);

  const handleDrop = useCallback((e) => {
    e.preventDefault();
    if (!e.dataTransfer.files.length) return;
    const f = e.dataTransfer.files[0];
    setFile(f);
    // mock up three simple paths when a file is imported
    const dummy = [0, 1, 2].map((i) => [{ x: i * 10, y: 0 }]);
    setPaths(dummy);
    setAssignments(dummy.map(() => ({ operation: 'Cut', head: 'Spindle' })));
    setParams(dummy.map(() => ({ feedrate: 1000, depth: 1 })));
    log(`canvas import: ${f.name}`);
  }, []);

  const updateTransform = (t) => setTransform(t);

  const dummyGcode = 'G1 X0 Y0\nG1 X10 Y0\nG1 X10 Y10';
  const dummyLineCount = dummyGcode.split(/\r?\n/).length;

  return (
    <div className="min-h-screen bg-gray-900 text-gray-100 flex flex-col">
      <header className="bg-gray-800 p-2 flex items-center justify-between">
        <div className="font-semibold">Profile</div>
        <div className="flex items-center space-x-2">
          <select
            value={tool}
            onChange={(e) => {
              setTool(e.target.value);
              log(`tool head -> ${e.target.value}`);
            }}
            className="bg-gray-700 rounded px-2 py-1"
          >
            <option>Laser</option>
            <option>Spindle</option>
            <option>Print Head</option>
          </select>
          <button
            className="bg-blue-600 hover:bg-blue-700 px-3 py-1 rounded"
            onClick={() => log('import click')}
          >
            Import
          </button>
          <button
            className="bg-green-600 hover:bg-green-700 px-3 py-1 rounded"
            onClick={() => log('export click')}
          >
            Export
          </button>
          <a
            href="https://example.github.io/cam_slicer/"
            target="_blank"
            rel="noreferrer"
            className="underline text-blue-400"
          >
            Docs
          </a>
        </div>
      </header>
      <div className="flex flex-1 overflow-hidden">
        <aside className="w-64 bg-gray-800 overflow-y-auto p-2">
          {SECTIONS.map(({ name, icon }) => (
            <div key={name} className="mb-2">
              <button
                className="w-full flex items-center text-left bg-gray-700 hover:bg-gray-600 rounded px-2 py-1"
                onClick={() => toggle(name)}
                data-testid={`section-${name}`}
              >
                <span className="mr-2" aria-hidden="true">{icon}</span>
                {name}
              </button>
              {open[name] && (
                <div className="pl-4 py-2 text-sm text-gray-300" data-testid="section-content">
                  {OPTIONS[name].map((o) => (
                    <button
                      key={o}
                      className="block w-full text-left hover:underline mt-1"
                      onClick={() => log(`${o} clicked`)}
                    >
                      {o}
                    </button>
                  ))}
                </div>
              )}
            </div>
          ))}
        </aside>
        <main className="flex-1 flex">
          <div
            className="flex-1 m-2 bg-gray-800 rounded flex items-center justify-center relative"
            data-testid="canvas"
            onDragOver={handleDragOver}
            onDrop={handleDrop}
          >
            {file && <Preview file={file} transform={transform} />}
            {assignments.length > 0 && <OperationPreview assignments={assignments} />}
          </div>
          <div className="w-64 m-2 bg-gray-800 rounded p-2" data-testid="right-panel">
            {file ? (
              <div>
                <h2 className="font-semibold">Import Details</h2>
                <p className="break-all">{file.name}</p>
                <p>{(file.size / 1024).toFixed(1)} kB</p>
                <ToolpathTransform onChange={updateTransform} />
              </div>
            ) : active === 'Tools & Operations' ? (
              <div>
                <OperationAssigner paths={paths} onChange={setAssignments} />
                {paths.map((p, idx) => (
                  <div key={idx} className="mt-2 text-sm">
                    <h4 className="font-semibold">Path {idx + 1} Params</h4>
                    <label className="block">Feedrate
                      <input
                        type="number"
                        value={params[idx]?.feedrate || 0}
                        onChange={(e) => {
                          const val = Number(e.target.value);
                          setParams((ps) => ps.map((p, i) => i === idx ? { ...p, feedrate: val } : p));
                        }}
                        className="w-full text-black rounded p-1"
                      />
                    </label>
                    <label className="block">Depth
                      <input
                        type="number"
                        value={params[idx]?.depth || 0}
                        onChange={(e) => {
                          const val = Number(e.target.value);
                          setParams((ps) => ps.map((p, i) => i === idx ? { ...p, depth: val } : p));
                        }}
                        className="w-full text-black rounded p-1"
                      />
                  </label>
                </div>
              ))}
              <label className="block text-sm mt-3">
                <input
                  type="checkbox"
                  className="mr-1"
                  checked={adaptiveMode}
                  onChange={(e) => {
                    setAdaptiveMode(e.target.checked);
                    log(`adaptive mode -> ${e.target.checked}`);
                  }}
                />
                Adaptive Toolpath (experimental)
              </label>
            </div>
            ) : active === 'Hybrid Operations' ? (
              <div>
                <OperationAssigner paths={paths} onChange={setAssignments} />
              </div>
            ) : active === 'Toolpath Optimization' ? (
              <div className="text-sm">Use optimizers from the AI Analyze & Optimize panel.</div>
            ) : active === 'Simulation' ? (
              <div>
                <ToolpathSimulator gcode={dummyGcode} />
                <div className="text-sm mt-2">
                  <h4 className="font-semibold">Simulation Stats</h4>
                  <p>Lines: {dummyLineCount}</p>
                </div>
              </div>
            ) : active === 'Export & Streaming' ? (
              <div className="space-y-2">
                <WorkspaceScanner onScan={() => log('scan done')} />
                <div className="space-x-2">
                  <button onClick={() => log('export grbl')} className="bg-blue-600 px-2 py-1 rounded">GRBL</button>
                  <button onClick={() => log('export marlin')} className="bg-blue-600 px-2 py-1 rounded">Marlin</button>
                  <button onClick={() => log('export klipper')} className="bg-blue-600 px-2 py-1 rounded">Klipper</button>
                  <button onClick={() => log('export custom')} className="bg-blue-600 px-2 py-1 rounded">Custom</button>
                </div>
                <GcodeStreamer gcodeLines={dummyGcode.split(/\r?\n/)} fileName="demo.gcode" />
                <label className="block text-sm mt-2">
                  <input type="checkbox" className="mr-1" onChange={() => log('dry run toggled')} />Dry run only
                </label>
              </div>
            ) : active === 'AI Analyze & Optimize' ? (
              <div className="space-y-4">
                <PluginManager plugins={[{ name: 'reverse_path' }, { name: 'adaptive_path' }]} />
                <ToolpathOptimizer gcode={dummyGcode} />
              </div>
            ) : active === 'Experimental features' ? (
              <div>
                <label className="block text-sm" title="Toggle experimental adaptive toolpath mode">
                  <input
                    type="checkbox"
                    className="mr-1"
                    checked={adaptiveMode}
                    onChange={(e) => {
                      setAdaptiveMode(e.target.checked);
                      log(`adaptive mode -> ${e.target.checked}`);
                    }}
                  />
                  Adaptive Toolpath (experimental)
                </label>
              </div>
            ) : active === 'Experimental AI' ? (
              <div className="space-y-4">
                <SurfaceAnalyzer gcode={dummyGcode} heightmap={{ points: [] }} />
                <ToolpathOptimizer gcode={dummyGcode} />
                <p className="text-xs">AI Feedback coming soon...</p>
              </div>
            ) : active === 'Experimental â Robot Arm' ? (
              <RobotArmPanel />
            ) : active === 'Diagnostics' ? (
              <DiagnosticsPanel />
            ) : active === 'Monitoring' ? (
              <MachineMonitor />
            ) : active ? (
              <h2 className="font-semibold mb-2">{active} Settings</h2>
            ) : (
              <p>Select a menu section</p>
            )}
          </div>
        </main>
      </div>
    </div>
  );
}

function Preview({ file, transform }) {
  const ext = file.name.split('.').pop().toLowerCase();
  const style = {
    transform: `translate(${transform.x}px, ${transform.y}px) rotate(${transform.rotation}deg) scale(${transform.scale})`,
  };
  if (['png', 'jpg', 'jpeg', 'svg'].includes(ext)) {
    return (
      <img
        src={URL.createObjectURL(file)}
        alt="preview"
        style={style}
        className="max-h-full max-w-full"
      />
    );
  }
  if (['dxf', 'dwg'].includes(ext)) {
    return <p style={style}>2D CAD Drawing</p>;
  }
  if (['obj', 'stl'].includes(ext)) {
    return <Basic3DPreview style={style} />;
  }
  return <p>Unsupported file</p>;
}

function Basic3DPreview({ style }) {
  const ref = React.useRef(null);
  React.useEffect(() => {
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(200, 200);
    ref.current.appendChild(renderer.domElement);
    const geo = new THREE.BoxGeometry(1, 1, 1);
    const mat = new THREE.MeshNormalMaterial();
    const cube = new THREE.Mesh(geo, mat);
    scene.add(cube);
    camera.position.z = 3;
    let anim;
    const animate = () => {
      cube.rotation.x += 0.01;
      cube.rotation.y += 0.01;
      renderer.render(scene, camera);
      anim = requestAnimationFrame(animate);
    };
    animate();
    return () => {
      cancelAnimationFrame(anim);
      ref.current.removeChild(renderer.domElement);
    };
  }, []);
  return <div ref={ref} style={style} className="w-40 h-40" />;
}

function OperationPreview({ assignments }) {
  const colors = { Laser: 'red', Spindle: 'blue', 'Print Head': 'green' };
  return (
    <div className="absolute top-2 left-2 space-y-1 text-xs">
      {assignments.map((a, idx) => (
        <div key={idx} className="flex items-center">
          <span
            className="inline-block w-3 h-3 mr-1"
            style={{ background: colors[a.head] || 'gray' }}
          />
          <span>
            Path {idx + 1}: {a.head}
          </span>
        </div>
      ))}
    </div>
  );
}
