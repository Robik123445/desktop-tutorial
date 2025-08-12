import React, { useState, useEffect, useRef } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
// @ts-ignore
import { STLExporter } from 'three/examples/jsm/exporters/STLExporter.js';
// @ts-ignore
import { BufferGeometryUtils } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { saveAs } from 'file-saver';
import { ToolpathData } from './viewportUtils';
import log from './src/log.js';

interface Voxel {
  pos: THREE.Vector3;
  visible: boolean;
}

interface Props {
  toolpaths: ToolpathData[];
  stock: { x: number; y: number; z: number };
  toolRadius?: number;
  resolution?: number;
}

/**
 * Cut stock simulation using a simple voxel grid.
 * Removed voxels are hidden as the tool moves along the path.
 */
export default function CutStockSimulator({
  toolpaths,
  stock,
  toolRadius = 2,
  resolution = 20,
}: Props) {
  const [voxels, setVoxels] = useState<Voxel[]>([]);
  const [index, setIndex] = useState(0);
  const [running, setRunning] = useState(false);
  const timerRef = useRef<NodeJS.Timeout | null>(null);

  const stepX = stock.x / resolution;
  const stepY = stock.y / resolution;
  const stepZ = stock.z / Math.max(1, Math.floor(resolution / 2));

  useEffect(() => {
    const vox: Voxel[] = [];
    for (let i = 0; i < resolution; i++) {
      for (let j = 0; j < resolution; j++) {
        for (let k = 0; k < Math.floor(resolution / 2); k++) {
          vox.push({
            pos: new THREE.Vector3(
              -stock.x / 2 + stepX * i + stepX / 2,
              -stock.y / 2 + stepY * j + stepY / 2,
              -stock.z / 2 + stepZ * k + stepZ / 2,
            ),
            visible: true,
          });
        }
      }
    }
    setVoxels(vox);
  }, [stock.x, stock.y, stock.z, resolution, stepX, stepY, stepZ]);

  const points = toolpaths.flatMap((tp) => tp.points);

  function cutAtPoint(p: THREE.Vector3) {
    setVoxels((vs) =>
      vs.map((v) => {
        if (!v.visible) return v;
        if (v.pos.distanceTo(p) <= toolRadius) {
          return { ...v, visible: false };
        }
        return v;
      }),
    );
  }

  useEffect(() => {
    if (running) {
      timerRef.current = setInterval(() => {
        setIndex((i) => {
          const next = Math.min(i + 1, points.length - 1);
          const pt = points[next];
          if (pt) cutAtPoint(pt);
          if (next === points.length - 1) {
            setRunning(false);
            log('cut stock sim finished');
          }
          return next;
        });
      }, 200);
    }
    return () => {
      if (timerRef.current) clearInterval(timerRef.current);
    };
  }, [running, points]);

  const play = () => {
    if (!running) {
      log('cut stock sim start');
      setRunning(true);
    }
  };
  const pause = () => {
    if (running) {
      log('cut stock sim pause');
      setRunning(false);
    }
  };

  const exportStl = () => {
    const boxGeom = new THREE.BoxGeometry(stepX, stepY, stepZ);
    const geoms: THREE.BufferGeometry[] = [];
    voxels.forEach((v) => {
      if (v.visible) {
        const g = boxGeom.clone();
        g.translate(v.pos.x, v.pos.y, v.pos.z);
        geoms.push(g);
      }
    });
    if (geoms.length === 0) return;
    const merged = BufferGeometryUtils.mergeBufferGeometries(geoms);
    const mesh = new THREE.Mesh(merged);
    const exporter = new STLExporter();
    const stl = exporter.parse(mesh) as string;
    const blob = new Blob([stl], { type: 'model/stl' });
    saveAs(blob, 'cut_stock.stl');
    log('cut stock STL exported');
  };

  return (
    <div className="p-4 space-y-2">
      <Canvas style={{ height: 300 }}>
        <ambientLight intensity={0.5} />
        <directionalLight position={[5, 5, 5]} />
        <group>
          {voxels.map((v, i) => (
            <mesh key={i} position={v.pos} visible={v.visible}>
              <boxGeometry args={[stepX, stepY, stepZ]} />
              <meshStandardMaterial
                color={v.visible ? '#bbbbbb' : '#444444'}
                transparent
                opacity={0.8}
              />
            </mesh>
          ))}
        </group>
        <OrbitControls />
      </Canvas>
      <div className="flex space-x-2">
        <button onClick={play} className="bg-green-600 text-white px-3 py-1 rounded">
          Play
        </button>
        <button onClick={pause} className="bg-yellow-600 text-white px-3 py-1 rounded">
          Pause
        </button>
        <button onClick={exportStl} className="bg-blue-600 text-white px-3 py-1 rounded">
          Export STL
        </button>
      </div>
    </div>
  );
}
