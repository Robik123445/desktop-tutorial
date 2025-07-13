import * as THREE from 'three';
// @ts-ignore
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
// @ts-ignore
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
// @ts-ignore
import { SVGLoader } from 'three/examples/jsm/loaders/SVGLoader.js';
// @ts-ignore
import { DXFLoader } from 'three/examples/jsm/loaders/DXFLoader.js';

export interface ModelData {
  object: THREE.Object3D;
  bbox: THREE.Box3;
  name: string;
  size: THREE.Vector3;
}

export type ToolpathType =
  | 'milling'
  | 'drilling'
  | 'engraving'
  | 'laser_cut'
  | 'laser_mark';

export type ToolHead = 'spindle' | 'laser' | 'picker';

export const HEAD_COLORS: Record<ToolHead, number> = {
  spindle: 0x0088ff,
  laser: 0xff2222,
  picker: 0x22ff22,
};

export interface Diagnostic {
  layer: number;
  index: number;
  position: THREE.Vector3;
  message: string;
}

export interface ToolpathData {
  object: THREE.Object3D;
  points: THREE.Vector3[];
  rotations: THREE.Vector3[];
  type: ToolpathType;
  head: ToolHead;
  bbox: THREE.Box3;
  name: string;
  layers: THREE.Vector3[][];
  rotationLayers?: THREE.Vector3[][];
  diagnostics: Diagnostic[];
}

function parseGcode(text: string): THREE.Vector3[] {
  const lines = text.split(/\r?\n/);
  let x = 0,
    y = 0,
    z = 0,
    a = 0,
    b = 0,
    c = 0;
  const pts: THREE.Vector3[] = [];
  for (const line of lines) {
    if (/G0|G1/.test(line)) {
      const xMatch = line.match(/X(-?\d*\.?\d*)/);
      const yMatch = line.match(/Y(-?\d*\.?\d*)/);
      const zMatch = line.match(/Z(-?\d*\.?\d*)/);
      const aMatch = line.match(/A(-?\d*\.?\d*)/i);
      const bMatch = line.match(/B(-?\d*\.?\d*)/i);
      const cMatch = line.match(/C(-?\d*\.?\d*)/i);
      if (xMatch) x = parseFloat(xMatch[1]);
      if (yMatch) y = parseFloat(yMatch[1]);
      if (zMatch) z = parseFloat(zMatch[1]);
      if (aMatch) a = parseFloat(aMatch[1]);
      if (bMatch) b = parseFloat(bMatch[1]);
      if (cMatch) c = parseFloat(cMatch[1]);
      pts.push(new THREE.Vector3(x, y, z));
    }
  }
  return pts;
}

/** Parse G-code into categorized toolpath segments. */
function parseGcodeToolpaths(text: string): ToolpathData[] {
  const lines = text.split(/\r?\n/);
  let x = 0,
    y = 0,
    z = 0,
    a = 0,
    b = 0,
    c = 0;
  let current: ToolpathType = 'milling';
  const map: Record<ToolpathType, THREE.Vector3[]> = {
    milling: [],
    drilling: [],
    engraving: [],
    laser_cut: [],
    laser_mark: [],
  };
  const rotMap: Record<ToolpathType, THREE.Vector3[]> = {
    milling: [],
    drilling: [],
    engraving: [],
    laser_cut: [],
    laser_mark: [],
  };

  const opMap: Record<string, ToolpathType> = {
    cut: 'milling',
    drill: 'drilling',
    engrave: 'engraving',
    laser: 'laser_cut',
    mark: 'laser_mark',
  };

  const headMap: Record<ToolpathType, ToolHead> = {
    milling: 'spindle',
    drilling: 'spindle',
    engraving: 'spindle',
    laser_cut: 'laser',
    laser_mark: 'laser',
  };

  for (const line of lines) {
    const op = line.match(/;\s*OP[:=](\w+)/i);
    if (op) {
      const t = opMap[op[1].toLowerCase()];
      if (t) current = t;
    }
    if (/G0|G1/.test(line)) {
      const xMatch = line.match(/X(-?\d*\.?\d*)/i);
      const yMatch = line.match(/Y(-?\d*\.?\d*)/i);
      const zMatch = line.match(/Z(-?\d*\.?\d*)/i);
      const aMatch = line.match(/A(-?\d*\.?\d*)/i);
      const bMatch = line.match(/B(-?\d*\.?\d*)/i);
      const cMatch = line.match(/C(-?\d*\.?\d*)/i);
      if (xMatch) x = parseFloat(xMatch[1]);
      if (yMatch) y = parseFloat(yMatch[1]);
      if (zMatch) z = parseFloat(zMatch[1]);
      if (aMatch) a = parseFloat(aMatch[1]);
      if (bMatch) b = parseFloat(bMatch[1]);
      if (cMatch) c = parseFloat(cMatch[1]);
      map[current].push(new THREE.Vector3(x, y, z));
      rotMap[current].push(new THREE.Vector3(a, b, c));
    }
  }

  const colors: Record<ToolpathType, number> = {
    milling: 0x00ff00,
    drilling: 0xff00ff,
    engraving: 0xffff00,
    laser_cut: 0xff0000,
    laser_mark: 0xff8800,
  };

  const segments: ToolpathData[] = [];
  for (const type of Object.keys(map) as ToolpathType[]) {
    const pts = map[type];
    if (!pts.length) continue;
    const layers: THREE.Vector3[][] = [];
    let current: THREE.Vector3[] = [];
    let lastZ = pts[0].z;
    pts.forEach((p) => {
      if (p.z > lastZ + 0.01) {
        layers.push(current);
        current = [];
      }
      current.push(p);
      lastZ = p.z;
    });
    if (current.length) layers.push(current);
    const rotLayers: THREE.Vector3[][] = [];
    let curR: THREE.Vector3[] = [];
    let idx = 0;
    rotMap[type].forEach((r, i) => {
      if (i > 0 && pts[i].z > pts[i - 1].z + 0.01) {
        rotLayers.push(curR);
        curR = [];
      }
      curR.push(r);
      idx = i;
    });
    if (curR.length) rotLayers.push(curR);

    const diagnostics: Diagnostic[] = [];
    layers.forEach((layerPts, layerIdx) => {
      for (let i = 1; i < layerPts.length - 1; i++) {
        const prev = layerPts[i - 1];
        const curr = layerPts[i];
        const next = layerPts[i + 1];
        const v1 = new THREE.Vector3().subVectors(curr, prev);
        const v2 = new THREE.Vector3().subVectors(next, curr);
        const angle = (v1.angleTo(v2) * 180) / Math.PI;
        if (angle < 135) {
          diagnostics.push({
            layer: layerIdx,
            index: i,
            position: curr.clone(),
            message: 'sharp corner',
          });
        }
        if (curr.z < -1) {
          diagnostics.push({
            layer: layerIdx,
            index: i,
            position: curr.clone(),
            message: 'possible collision',
          });
        }
      }
    });

    const geom = new THREE.BufferGeometry().setFromPoints(pts);
    const line = new THREE.Line(
      geom,
      new THREE.LineBasicMaterial({ color: colors[type] })
    );
    const bbox = new THREE.Box3().setFromPoints(pts);
    const center = bbox.getCenter(new THREE.Vector3());
    line.position.sub(center);
    const shifted = pts.map((p) => p.clone().sub(center));
    const shiftedRotations = rotMap[type].map((r) => r.clone());
    const shiftedLayers = layers.map((l) => l.map((p) => p.clone().sub(center)));
    const shiftedRotLayers = rotLayers.map((l) => l.map((r) => r.clone()));
    const shiftedDiags = diagnostics.map((d) => ({
      ...d,
      position: d.position.clone().sub(center),
    }));
    segments.push({
      object: line,
      points: shifted,
      type,
      head: headMap[type],
      bbox,
      name: type,
      layers: shiftedLayers,
      rotations: shiftedRotations,
      rotationLayers: shiftedRotLayers,
      diagnostics: shiftedDiags,
    });
  }

  return segments;
}

export async function loadModel(file: File): Promise<ModelData | null> {
  const ext = file.name.split('.').pop()?.toLowerCase();
  return new Promise((resolve) => {
    const reader = new FileReader();
    reader.onerror = () => resolve(null);

    if (ext === 'stl') {
      reader.onload = () => {
        const loader = new STLLoader();
        const geom = loader.parse(reader.result as ArrayBuffer);
        geom.computeBoundingBox();
        const mesh = new THREE.Mesh(
          geom,
          new THREE.MeshStandardMaterial({ color: 0x999999 })
        );
        const bbox = geom.boundingBox ?? new THREE.Box3();
        const size = bbox.getSize(new THREE.Vector3());
        mesh.position.sub(bbox.getCenter(new THREE.Vector3()));
        resolve({ object: mesh, bbox, name: file.name, size });
      };
      reader.readAsArrayBuffer(file);
    } else if (ext === 'obj') {
      reader.onload = () => {
        const loader = new OBJLoader();
        const obj = loader.parse(reader.result as string);
        const bbox = new THREE.Box3().setFromObject(obj);
        const size = bbox.getSize(new THREE.Vector3());
        obj.position.sub(bbox.getCenter(new THREE.Vector3()));
        resolve({ object: obj, bbox, name: file.name, size });
      };
      reader.readAsText(file);
    } else if (ext === 'svg') {
      reader.onload = () => {
        const loader = new SVGLoader();
        const data = loader.parse(reader.result as string);
        const group = new THREE.Group();
        data.paths.forEach((p) => {
          const shapes = p.toShapes(true);
          shapes.forEach((s) => {
            const geom = new THREE.ShapeGeometry(s);
            const mat = new THREE.MeshBasicMaterial({
              color: p.color,
              side: THREE.DoubleSide,
            });
            const mesh = new THREE.Mesh(geom, mat);
            group.add(mesh);
          });
        });
        const bbox = new THREE.Box3().setFromObject(group);
        const size = bbox.getSize(new THREE.Vector3());
        group.position.sub(bbox.getCenter(new THREE.Vector3()));
        resolve({ object: group, bbox, name: file.name, size });
      };
      reader.readAsText(file);
    } else if (ext === 'dxf') {
      reader.onload = () => {
        // DXFLoader is not typed
        // @ts-ignore
        const loader = new DXFLoader();
        // @ts-ignore
        const group = loader.parse(reader.result as string);
        const bbox = new THREE.Box3().setFromObject(group);
        const size = bbox.getSize(new THREE.Vector3());
        group.position.sub(bbox.getCenter(new THREE.Vector3()));
        resolve({ object: group, bbox, name: file.name, size });
      };
      reader.readAsText(file);
    } else {
      resolve(null);
    }
  });
}

export async function loadToolpaths(file: File): Promise<ToolpathData[] | null> {
  const ext = file.name.split('.').pop()?.toLowerCase();
  return new Promise((resolve) => {
    if (ext !== 'gcode' && ext !== 'nc') {
      resolve(null);
      return;
    }
    const reader = new FileReader();
    reader.onerror = () => resolve(null);
    reader.onload = () => {
      const segments = parseGcodeToolpaths(reader.result as string);
      segments.forEach((s) => (s.name = file.name));
      resolve(segments);
    };
    reader.readAsText(file);
  });
}

/** Reorder toolpaths using a simple nearest-neighbour approach. */
export function suggestToolpathOrder(paths: ToolpathData[]): ToolpathData[] {
  if (!paths.length) return [];
  const remaining = [...paths];
  const ordered: ToolpathData[] = [remaining.shift() as ToolpathData];
  while (remaining.length) {
    const last = ordered[ordered.length - 1];
    const lastEnd = last.points[last.points.length - 1];
    let nearestIdx = 0;
    let nearestDist = lastEnd.distanceTo(remaining[0].points[0]);
    for (let i = 1; i < remaining.length; i++) {
      const dist = lastEnd.distanceTo(remaining[i].points[0]);
      if (dist < nearestDist) {
        nearestDist = dist;
        nearestIdx = i;
      }
    }
    ordered.push(remaining.splice(nearestIdx, 1)[0]);
  }
  return ordered;
}

/** Measure straight line distance between two points in mm. */
export function measureDistance(a: THREE.Vector3, b: THREE.Vector3): number {
  return a.distanceTo(b);
}

/**
 * Measure angle ABC (in degrees) with B as vertex.
 */
export function measureAngle(
  a: THREE.Vector3,
  b: THREE.Vector3,
  c: THREE.Vector3
): number {
  const v1 = new THREE.Vector3().subVectors(a, b).normalize();
  const v2 = new THREE.Vector3().subVectors(c, b).normalize();
  return (v1.angleTo(v2) * 180) / Math.PI;
}

