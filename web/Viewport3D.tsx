import React, { useEffect, useCallback, useRef, useState } from 'react';
import { Canvas, useThree } from '@react-three/fiber';
import { OrbitControls, TransformControls, Html } from '@react-three/drei';
import * as THREE from 'three';
import log from "./src/log.js";
import { saveAs } from 'file-saver';
import jsPDF from 'jspdf';
import ToolpathRenderer from "./ToolpathRenderer";
import ZoneVisualizer, { ZonePlan } from "./ZoneVisualizer";
import {
  loadModel,
  loadToolpaths,
  ModelData,
  ToolpathData,
  ToolpathType,
  ToolHead,
  HEAD_COLORS,
  suggestToolpathOrder,
  measureDistance,
  measureAngle,
} from "./viewportUtils";
import { analyzeScene, predictLoadPoints, AIDiagnostic, LoadPoint } from './viewportDiagnostics';

function AIDiagnosticMarker({ diag, onFocus }: { diag: AIDiagnostic; onFocus?: (p: THREE.Vector3) => void }) {
  const [hover, setHover] = useState(false);
  const color = diag.severity === 'error' ? 'red' : diag.severity === 'warning' ? 'orange' : 'yellow';
  return (
    <mesh
      position={diag.position}
      onPointerOver={() => setHover(true)}
      onPointerOut={() => setHover(false)}
      onClick={() => {
        log(`ai diagnostic clicked: ${diag.message}`);
        onFocus && onFocus(diag.position);
      }}
    >
      <sphereGeometry args={[1.5, 8, 8]} />
      <meshStandardMaterial color={color} />
      {hover && (
        <Html distanceFactor={10} style={{ pointerEvents: 'none' }}>
          <div className="bg-black text-white text-xs px-1 rounded">
            {diag.message}
          </div>
        </Html>
      )}
    </mesh>
  );
}

function LoadMarker({ load }: { load: LoadPoint }) {
  const color = load.load > 0.7 ? 'red' : load.load > 0.4 ? 'orange' : 'green';
  return (
    <mesh position={load.position}>
      <sphereGeometry args={[1, 8, 8]} />
      <meshStandardMaterial color={color} opacity={0.6} transparent />
    </mesh>
  );
}

interface SceneProps {
  models: ModelData[];
  toolpaths: ToolpathData[];
  visible: Record<ToolpathType, boolean>;
  animating: boolean;
  activeLayer: number;
  activeSegment: number;
  viewMode: 'layer' | 'segment';
  diagnostics: boolean;
  aiDiagnostics: AIDiagnostic[];
  showAIDiagnostics: boolean;
  controlsRef: React.RefObject<any>;
  selected: number | null;
  mode: 'translate' | 'rotate';
  simIndex: number;
  startIndices: number[];
  measurementPoints: THREE.Vector3[];
  measurementMode: 'none' | 'distance' | 'angle';
  measurementValue: number | null;
  notes: { position: THREE.Vector3; text: string }[];
  scanData?: { debris?: [number, number, number, number][]; heightmap?: { points: { x: number; y: number; z: number }[] } };
  zones: ZonePlan[];
  zoneIndex: number;
  loads: LoadPoint[];
  showAnalytics: boolean;
  onSelect: (index: number | null) => void;
  onFocus: (pos: THREE.Vector3) => void;
  onCanvasClick: (p: THREE.Vector3) => void;
}

function Scene({ models, toolpaths, visible, animating, activeLayer, activeSegment, viewMode, diagnostics, aiDiagnostics, showAIDiagnostics, controlsRef, selected, mode, simIndex, startIndices, measurementPoints, measurementMode, measurementValue, notes, scanData, zones, zoneIndex, loads, showAnalytics, onSelect, onFocus, onCanvasClick }: SceneProps) {
  const { camera } = useThree();
  const measureGeom = React.useMemo(() => {
    const geom = new THREE.BufferGeometry();
    if (measurementPoints.length > 1) geom.setFromPoints(measurementPoints);
    return geom;
  }, [measurementPoints]);

  useEffect(() => {
    if (!models.length && !toolpaths.length) return;
    const box = new THREE.Box3();
    models.forEach((m) => box.union(m.bbox));
    toolpaths.forEach((t) => box.union(t.bbox));
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z);
    const fov = (camera as THREE.PerspectiveCamera).fov * Math.PI / 180;
    let dist = maxDim / (2 * Math.tan(fov / 2));
    dist *= 1.2;
    camera.position.set(center.x + dist, center.y + dist, center.z + dist);
    if (controlsRef.current) {
      controlsRef.current.target.copy(center);
      controlsRef.current.update();
    }
  }, [models, toolpaths, camera, controlsRef]);


  return (
    <>
      <ambientLight intensity={0.5} />
      <directionalLight position={[100, 100, 100]} intensity={0.6} />
      <gridHelper args={[400, 40, '#4B5563', '#374151']} />
      <axesHelper args={[200]} />
      <mesh
        rotation={[-Math.PI / 2, 0, 0]}
        position={[0, 0, 0]}
        onPointerDown={(e) => {
          e.stopPropagation();
          onCanvasClick(e.point.clone());
        }}
      >
        <planeGeometry args={[400, 400]} />
        <meshBasicMaterial visible={false} />
      </mesh>
      {models.map((m, idx) => (
        <primitive
          key={m.name + m.size.toArray().join()}
          object={m.object}
          onPointerDown={(e) => {
            e.stopPropagation();
            onSelect(idx);
          }}
        />
      ))}
      <ToolpathRenderer
        toolpaths={toolpaths}
        visible={visible}
        headsVisible={headsVisible}
        animating={animating}
        activeLayer={activeLayer}
        activeSegment={activeSegment}
        mode={viewMode}
        diagnostics={diagnostics}
        simIndex={simIndex}
        startIndices={startIndices}
        materialPreview={materialPreview}
        onFocus={(pos) => onFocus(pos)}
      />
      <ZoneVisualizer zones={zones} animating={zoneAnimating} simIndex={zoneIndex} onUpdate={setZones} />
      {measurementMode !== 'none' && measurementPoints.length > 1 && (
        <line>
          <primitive attach="geometry" object={measureGeom} />
          <lineBasicMaterial attach="material" color="cyan" />
        </line>
      )}
      {measurementValue !== null && (
        <Html position={measurementPoints[Math.floor(measurementPoints.length/2)]}>
          <div className="bg-black text-cyan-300 text-xs px-1 rounded">
            {measurementMode === 'distance'
              ? `${measurementValue.toFixed(2)} mm`
              : `${measurementValue.toFixed(1)}Â°`}
          </div>
        </Html>
      )}
      {notes.map((n, i) => (
        <Html key={`note-${i}`} position={n.position}>
          <div className="bg-yellow-200 text-black text-xs px-1 rounded">
            {n.text}
          </div>
        </Html>
      ))}
      {showAIDiagnostics &&
        aiDiagnostics.map((d, i) => (
          <AIDiagnosticMarker key={`aid-${i}`} diag={d} onFocus={onFocus} />
        ))}
      {showAnalytics &&
        loads.map((l, i) => <LoadMarker key={`load-${i}`} load={l} />)}
      {scanData?.debris?.map((b, i) => {
        const [x1, y1, x2, y2] = b;
        const cx = (x1 + x2) / 2;
        const cy = (y1 + y2) / 2;
        const w = Math.abs(x2 - x1);
        const h = Math.abs(y2 - y1);
        return (
          <mesh key={`deb-${i}`} position={[cx, 0.1, cy]} rotation-x={-Math.PI / 2}>
            <planeGeometry args={[w, h]} />
            <meshBasicMaterial color="red" opacity={0.5} transparent />
          </mesh>
        );
      })}
      {scanData?.heightmap?.points && (
        <group>
          {scanData.heightmap.points.map((p, i) => (
            <mesh key={`hm-${i}`} position={[p.x, p.z, p.y]}>
              <sphereGeometry args={[0.5, 4, 4]} />
              <meshStandardMaterial color="green" />
            </mesh>
          ))}
        </group>
      )}
      {selected !== null && models[selected] && (
        <TransformControls
          object={models[selected].object}
          mode={mode}
          onMouseDown={() => {
            if (controlsRef.current) controlsRef.current.enabled = false;
          }}
          onMouseUp={() => {
            if (controlsRef.current) controlsRef.current.enabled = true;
          }}
        />
      )}
      <OrbitControls ref={controlsRef} />
    </>
  );
}

/**
 * Viewport3D renders a 3D workspace with drag-and-drop model import.
 */
export default function Viewport3D() {
  const [models, setModels] = useState<ModelData[]>([]);
  const [toolpaths, setToolpaths] = useState<ToolpathData[]>([]);
  const [visible, setVisible] = useState<Record<ToolpathType, boolean>>({
    milling: true,
    drilling: true,
    engraving: true,
    laser_cut: true,
    laser_mark: true,
  });
  const [animating, setAnimating] = useState(false);
  const [selected, setSelected] = useState<number | null>(null);
  const [mode, setMode] = useState<'translate' | 'rotate'>('translate');
  const [activeLayer, setActiveLayer] = useState(0);
  const [activeSegment, setActiveSegment] = useState(0);
  const [viewMode, setViewMode] = useState<'layer' | 'segment'>('layer');
  const [diagnostics, setDiagnostics] = useState(true);
  const [showAIDiag, setShowAIDiag] = useState(true);
  const [aiDiagnostics, setAIDiagnostics] = useState<AIDiagnostic[]>([]);
  const [headsVisible, setHeadsVisible] = useState<Record<ToolHead, boolean>>({
    spindle: true,
    laser: true,
    picker: true,
  });
  const [simIndex, setSimIndex] = useState(0);
  const [simRunning, setSimRunning] = useState(false);
  const [simPoints, setSimPoints] = useState<THREE.Vector3[]>([]);
  const [startIndices, setStartIndices] = useState<number[]>([]);
  const [diagMap, setDiagMap] = useState<Record<number, string>>({});
  const [materialPreview, setMaterialPreview] = useState(false);
  const [projectName, setProjectName] = useState('');
  const [notes, setNotes] = useState('');
  const [measurementMode, setMeasurementMode] = useState<'none' | 'distance' | 'angle'>('none');
  const [measurementPoints, setMeasurementPoints] = useState<THREE.Vector3[]>([]);
  const [measurementValue, setMeasurementValue] = useState<number | null>(null);
  const [notesList, setNotesList] = useState<{ position: THREE.Vector3; text: string }[]>([]);
  const [scanData, setScanData] = useState<{ debris?: [number, number, number, number][]; heightmap?: { points: {x:number;y:number;z:number}[] } }>();
  const [zones, setZones] = useState<ZonePlan[]>([]);
  const [zoneIndex, setZoneIndex] = useState(0);
  const [zoneAnimating, setZoneAnimating] = useState(false);
  const [loadPoints, setLoadPoints] = useState<LoadPoint[]>([]);
  const [showAnalytics, setShowAnalytics] = useState(false);
  const controlsRef = useRef<any>(null);
  const noteMode = useRef(false);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    log('Viewport3D mounted');
  }, []);

  useEffect(() => {
    const diags = analyzeScene(models, toolpaths);
    setAIDiagnostics(diags);
  }, [models, toolpaths]);

  useEffect(() => {
    const map: Record<number, string> = {};
    let index = 0;
    toolpaths.forEach(tp => {
      tp.points.forEach((_, i) => {
        const d = tp.diagnostics.find(di => di.index === i);
        if (d) map[index] = d.message;
        index += 1;
      });
    });
    setDiagMap(map);
  }, [toolpaths]);

  // compute simulation points whenever toolpaths change
  useEffect(() => {
    const pts: THREE.Vector3[] = [];
    const starts: number[] = [];
    let idx = 0;
    toolpaths.forEach((tp, i) => {
      starts[i] = idx;
      tp.points.forEach(p => {
        pts.push(p);
        idx += 1;
      });
    });
    setSimPoints(pts);
    setStartIndices(starts);
    setSimIndex(0);
    setLoadPoints(predictLoadPoints(toolpaths));
  }, [toolpaths]);

  useEffect(() => {
    if (!simRunning || !simPoints.length) return;
    const timer = setInterval(() => {
      setSimIndex(i => {
        const next = Math.min(i + 1, simPoints.length - 1);
        if (diagMap[next]) log(`simulation issue: ${diagMap[next]}`);
        if (next === simPoints.length - 1) setSimRunning(false);
        return next;
      });
    }, 200);
    return () => clearInterval(timer);
  }, [simRunning, simPoints, diagMap]);

  useEffect(() => {
    if (!zoneAnimating || zones.length === 0) return;
    const timer = setInterval(() => {
      setZoneIndex(i => {
        const next = Math.min(i + 1, zones.length - 1);
        if (next === zones.length - 1) setZoneAnimating(false);
        return next;
      });
    }, 1000);
    return () => clearInterval(timer);
  }, [zoneAnimating, zones]);

  const handleDrop = useCallback(async (e: React.DragEvent<HTMLDivElement>) => {
    e.preventDefault();
    if (!e.dataTransfer.files.length) return;
    const file = e.dataTransfer.files[0];
    log(`file dropped: ${file.name}`);
    const ext = file.name.split('.').pop()?.toLowerCase();
    if (ext === 'gcode' || ext === 'nc') {
      const segments = await loadToolpaths(file);
      if (segments) {
        setToolpaths((prev) => {
          const updated = [...prev, ...segments];
          setActiveLayer(0);
          setActiveSegment(0);
          return updated;
        });
      }
    } else if (ext === 'json') {
      const text = await file.text();
      try {
        const parsed = JSON.parse(text);
        if (Array.isArray(parsed) && parsed[0]?.base) {
          setZones(parsed as ZonePlan[]);
          setZoneIndex(0);
          log(`zone plan loaded: ${parsed.length} zones`);
        } else {
          log(`Unknown JSON format: ${file.name}`, 'ERROR');
        }
      } catch (err) {
        log('Failed to parse JSON', 'ERROR', err);
      }
    } else {
      const data = await loadModel(file);
      if (data) {
        setModels((prev) => {
          const updated = [...prev, data];
          setSelected(updated.length - 1);
          return updated;
        });
      } else {
        log(`Unsupported file: ${file.name}`, 'ERROR');
      }
    }
  }, []);

  const exportSnapshot = () => {
    if (!canvasRef.current) return;
    canvasRef.current.toBlob((blob) => {
      if (blob) {
        saveAs(blob, `${projectName || 'viewport'}_${Date.now()}.png`);
        log('export snapshot');
      }
    });
  };

  const exportReport = () => {
    if (!canvasRef.current) return;
    const img = canvasRef.current.toDataURL('image/png');
    const pdf = new jsPDF();
    pdf.text(`Project: ${projectName || 'Unnamed'}`, 10, 10);
    pdf.text(`Time: ${new Date().toLocaleString()}`, 10, 20);
    if (notes.trim()) {
      pdf.text(`Notes: ${notes}`, 10, 30);
    }
    pdf.addImage(img, 'PNG', 10, 40, 180, 100);
    pdf.text('Detected issues:', 10, 145);
    aiDiagnostics.forEach((d, i) => {
      pdf.text(`- ${d.message}`, 10, 155 + i * 10);
    });
    pdf.save(`${projectName || 'viewport_report'}.pdf`);
    log('export report');
  };

  const handleCanvasClick = (p: THREE.Vector3) => {
    if (measurementMode !== 'none') {
      setMeasurementPoints((pts) => {
        const next = [...pts, p];
        if (
          (measurementMode === 'distance' && next.length === 2) ||
          (measurementMode === 'angle' && next.length === 3)
        ) {
          if (measurementMode === 'distance') {
            setMeasurementValue(measureDistance(next[0], next[1]));
          } else {
            setMeasurementValue(measureAngle(next[0], next[1], next[2]));
          }
        }
        return next;
      });
    } else if (noteMode.current) {
      const text = prompt('Note text?');
      if (text) {
        setNotesList((n) => [...n, { position: p, text }]);
        log(`note added: ${text}`);
      }
      noteMode.current = false;
    }
  };

  const startSim = () => {
    if (!simRunning) {
      log('simulation play');
      setSimRunning(true);
    }
  };
  const pauseSim = () => {
    if (simRunning) {
      log('simulation pause');
      setSimRunning(false);
    }
  };
  const stepSim = (delta: number) => {
    setSimIndex((i) => {
      const next = Math.min(Math.max(i + delta, 0), simPoints.length - 1);
      log('simulation step');
      return next;
    });
  };

  return (
    <div
      className="w-full h-full bg-gray-800 relative"
      onDragOver={(e) => e.preventDefault()}
      onDrop={handleDrop}
      data-testid="viewport"
    >
      <Canvas
        className="w-full h-full"
        camera={{ position: [250, 250, 250], fov: 60 }}
        onCreated={({ gl }) => (canvasRef.current = gl.domElement)}
      >
        <Scene
          models={models}
          toolpaths={toolpaths}
          visible={visible}
          animating={animating}
          activeLayer={activeLayer}
          activeSegment={activeSegment}
          viewMode={viewMode}
          diagnostics={diagnostics}
          aiDiagnostics={aiDiagnostics}
          showAIDiagnostics={showAIDiag}
        simIndex={simIndex}
        startIndices={startIndices}
        loads={loadPoints}
        showAnalytics={showAnalytics}
        measurementPoints={measurementPoints}
        measurementMode={measurementMode}
        measurementValue={measurementValue}
        notes={notesList}
        scanData={scanData}
        zones={zones}
        zoneIndex={zoneIndex}
        controlsRef={controlsRef}
        selected={selected}
        mode={mode}
        onSelect={(idx) => setSelected(idx)}
        onFocus={(pos) => {
          if (controlsRef.current) {
            controlsRef.current.target.copy(pos);
            controlsRef.current.update();
          }
          log(`focus on ${pos.x.toFixed(1)},${pos.y.toFixed(1)},${pos.z.toFixed(1)}`);
        }}
        onCanvasClick={handleCanvasClick}
      />
      </Canvas>
      {models.length > 0 && (
        <div className="absolute top-0 right-0 bg-black bg-opacity-50 text-white text-xs p-2 space-y-1">
          {models.map((m, idx) => (
            <div key={idx} className="mb-2">
              <div>{m.name}</div>
              <div>
                {m.size.x.toFixed(1)} Ã {m.size.y.toFixed(1)} Ã {m.size.z.toFixed(1)} mm
              </div>
              <button
                className="bg-gray-700 px-2 py-1 mt-1 text-white rounded"
                onClick={() => setSelected(idx)}
              >
                {selected === idx ? 'Selected' : 'Select'}
              </button>
            </div>
          ))}
          <div className="mt-2 flex space-x-2">
            <button
              className="bg-gray-700 px-2 py-1 text-white rounded"
              onClick={() => setMode('translate')}
            >
              Move
            </button>
            <button
              className="bg-gray-700 px-2 py-1 text-white rounded"
              onClick={() => setMode('rotate')}
            >
              Rotate
            </button>
            <button
              className="bg-gray-700 px-2 py-1 text-white rounded"
              onClick={() => setSelected(null)}
            >
              Deselect
            </button>
          </div>
          {toolpaths.length > 0 && (
            <div className="mt-2 space-y-1">
              {Object.keys(visible).map((t) => (
                <label key={t} className="block">
                  <input
                    type="checkbox"
                    checked={visible[t as ToolpathType]}
                    onChange={() =>
                      setVisible((v) => ({ ...v, [t]: !v[t as ToolpathType] }))
                    }
                    className="mr-1"
                  />
                  {t}
                </label>
              ))}
              <button
                className="bg-gray-700 px-2 py-1 mt-1 text-white rounded w-full"
                onClick={() => setAnimating((a) => !a)}
              >
                {animating ? 'Pause' : 'Play'}
              </button>
              <div className="flex items-center space-x-2 mt-1">
                <select
                  value={viewMode}
                  onChange={(e) => setViewMode(e.target.value as 'layer' | 'segment')}
                  className="text-xs bg-gray-700 text-white px-1 rounded"
                >
                  <option value="layer">Layer</option>
                  <option value="segment">Segment</option>
                </select>
                {viewMode === 'layer' ? (
                  <>
                    <input
                      type="range"
                      min="0"
                      max={Math.max(0, ...toolpaths.map((t) => t.layers.length - 1))}
                      value={activeLayer}
                      onChange={(e) => {
                        const val = Number(e.target.value);
                        setActiveLayer(val);
                        log(`layer ${val}`);
                      }}
                      className="flex-1"
                    />
                    <span data-testid="layer-label" className="text-xs text-white w-6 text-center">
                      {activeLayer}
                    </span>
                  </>
                ) : (
                  <>
                    <input
                      type="range"
                      min="0"
                      max={Math.max(0, toolpaths.length - 1)}
                      value={activeSegment}
                      onChange={(e) => {
                        const val = Number(e.target.value);
                        setActiveSegment(val);
                        log(`segment ${val}`);
                      }}
                      className="flex-1"
                    />
                    <span className="text-xs text-white w-6 text-center">{activeSegment}</span>
                  </>
                )}
              </div>
              <label className="flex items-center space-x-1 text-xs">
                <input type="checkbox" checked={diagnostics} onChange={(e)=>setDiagnostics(e.target.checked)} />
                <span>Toolpath diag</span>
              </label>
              <label className="flex items-center space-x-1 text-xs">
                <input type="checkbox" checked={showAIDiag} onChange={(e)=>setShowAIDiag(e.target.checked)} />
                <span>AI Diagnostics</span>
              </label>
              <div className="mt-1">
                {Object.keys(headsVisible).map((h) => (
                  <label key={h} className="flex items-center space-x-1 text-xs mr-2">
                    <input
                      type="checkbox"
                      checked={headsVisible[h as ToolHead]}
                      onChange={() => setHeadsVisible(v => ({...v, [h]: !v[h as ToolHead]}))}
                    />
                    <span style={{color: `#${HEAD_COLORS[h as ToolHead].toString(16)}`}}>{h}</span>
                  </label>
                ))}
              </div>
              <button
                className="bg-gray-700 px-2 py-1 text-white rounded w-full mt-1"
                onClick={() => {
                  setToolpaths(tp => suggestToolpathOrder(tp));
                  log('optimize order');
                }}
              >
                Optimize Order
              </button>
              {simPoints.length > 0 && (
                <div className="space-y-1 mt-1">
                  <div className="flex space-x-1">
                    <button className="bg-green-600 text-white px-2 rounded" onClick={startSim}>Play</button>
                    <button className="bg-yellow-600 text-white px-2 rounded" onClick={pauseSim}>Pause</button>
                    <button className="bg-blue-600 text-white px-2 rounded" onClick={() => stepSim(-1)}>&lt;</button>
                    <button className="bg-blue-600 text-white px-2 rounded" onClick={() => stepSim(1)}>&gt;</button>
                  </div>
                  <div className="w-full bg-gray-600 h-2 rounded">
                    <div
                      className="bg-green-500 h-2 rounded"
                      style={{ width: `${(simIndex / (simPoints.length - 1)) * 100}%` }}
                    />
                  </div>
                </div>
              )}
              {zones.length > 0 && (
                <div className="space-y-1 mt-1">
                  <div className="flex space-x-1">
                    <button className="bg-green-600 text-white px-2 rounded" onClick={() => { setZoneAnimating(true); log('zone play'); }}>Play Zones</button>
                    <button className="bg-yellow-600 text-white px-2 rounded" onClick={() => { setZoneAnimating(false); log('zone pause'); }}>Pause</button>
                    <button className="bg-blue-600 text-white px-2 rounded" onClick={() => setZoneIndex(i=>Math.max(i-1,0))}>&lt;</button>
                    <button className="bg-blue-600 text-white px-2 rounded" onClick={() => setZoneIndex(i=>Math.min(i+1, zones.length-1))}>&gt;</button>
                  </div>
                  <div className="w-full bg-gray-600 h-2 rounded">
                    <div className="bg-purple-500 h-2 rounded" style={{width:`${(zoneIndex/(zones.length-1))*100}%`}} />
                  </div>
                </div>
              )}
              <label className="flex items-center space-x-1 text-xs">
                <input type="checkbox" checked={materialPreview} onChange={(e)=>setMaterialPreview(e.target.checked)} />
                <span>Material preview</span>
              </label>
              <label className="flex items-center space-x-1 text-xs">
                <input type="checkbox" checked={showAnalytics} onChange={(e)=>setShowAnalytics(e.target.checked)} />
                <span>Load overlay</span>
              </label>
              <div className="flex space-x-1 mt-1">
                <button className="bg-gray-700 px-1 rounded" onClick={() => {setMeasurementMode('distance'); setMeasurementPoints([]); setMeasurementValue(null); log('measure distance');}}>Dist</button>
                <button className="bg-gray-700 px-1 rounded" onClick={() => {setMeasurementMode('angle'); setMeasurementPoints([]); setMeasurementValue(null); log('measure angle');}}>Angle</button>
                <button className="bg-gray-700 px-1 rounded" onClick={() => {setMeasurementMode('none'); setMeasurementPoints([]); setMeasurementValue(null);}}>Clear</button>
                <button className="bg-gray-700 px-1 rounded" onClick={() => {noteMode.current = true; log('add note');}}>Note</button>
              </div>
            </div>
          )}
          {showAIDiag && aiDiagnostics.length > 0 && (
            <div className="mt-2">
              <div className="font-bold text-xs">AI Diagnostics</div>
              <ul className="space-y-1">
                {aiDiagnostics.map((d, i) => (
                  <li key={i}>
                    <button
                      className="underline"
                      onClick={() => {
                        if (controlsRef.current) {
                          controlsRef.current.target.copy(d.position);
                          controlsRef.current.update();
                        }
                      }}
                    >
                      {d.message}
                    </button>
                  </li>
                ))}
              </ul>
            </div>
          )}
          <div className="mt-2 space-y-1">
            <input
              type="text"
              placeholder="Project Name"
              value={projectName}
              onChange={(e) => setProjectName(e.target.value)}
              className="w-full text-black p-1 text-xs"
            />
            <textarea
              placeholder="Notes"
              value={notes}
              onChange={(e) => setNotes(e.target.value)}
              rows={2}
              className="w-full text-black p-1 text-xs"
            />
            <button
              className="bg-gray-700 px-2 py-1 text-white rounded w-full"
              onClick={exportSnapshot}
            >
              Snapshot
            </button>
            <button
              className="bg-gray-700 px-2 py-1 text-white rounded w-full"
              onClick={exportReport}
            >
              Export Report
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
