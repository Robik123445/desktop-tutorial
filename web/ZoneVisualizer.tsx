import React, { useEffect, useState, useRef } from 'react';
import { Html, TransformControls } from '@react-three/drei';
import * as THREE from 'three';
import log from './logger';

export interface ZonePlan {
  base: [number, number];
  toolpath: [number, number, number][];
  move_cmds: string[];
  warnings?: string[];
}

interface ZoneVisualizerProps {
  zones: ZonePlan[];
  visible?: boolean;
  animating?: boolean;
  onUpdate?: (zones: ZonePlan[]) => void;
  simIndex?: number;
}

/**
 * Visualize base positions and moves for planned zones.
 */
export default function ZoneVisualizer({
  zones,
  visible = true,
  animating = false,
  onUpdate,
  simIndex = -1,
}: ZoneVisualizerProps) {
  const [selected, setSelected] = useState<number | null>(null);
  const headRef = useRef<THREE.Mesh>(null);
  const pathPts = React.useMemo(() => {
    const pts: THREE.Vector3[] = [];
    zones.forEach(z => {
      pts.push(new THREE.Vector3(z.base[0], 0, z.base[1]));
    });
    return pts;
  }, [zones]);

  // Animate base moves along the zone order
  useEffect(() => {
    if (!animating || !pathPts.length || !headRef.current) return;
    let i = 0;
    headRef.current.position.copy(pathPts[0]);
    const timer = setInterval(() => {
      i += 1;
      if (i >= pathPts.length) {
        clearInterval(timer);
        return;
      }
      headRef.current!.position.copy(pathPts[i]);
    }, 500);
    return () => clearInterval(timer);
  }, [animating, pathPts]);

  // external simulation index for step preview
  useEffect(() => {
    if (simIndex < 0 || !headRef.current || !pathPts.length) return;
    const idx = Math.min(simIndex, pathPts.length - 1);
    headRef.current.position.copy(pathPts[idx]);
  }, [simIndex, pathPts]);

  const handleTransform = (i: number, pos: THREE.Vector3) => {
    const next = zones.map((z, idx) =>
      idx === i ? { ...z, base: [pos.x, pos.z] } : z
    );
    onUpdate && onUpdate(next);
  };

  if (!visible || !zones.length) return null;
  return (
    <group>
      {zones.map((z, i) => (
        <group key={i} position={[z.base[0], 0, z.base[1]]}>
          <mesh
            onClick={(e) => {
              e.stopPropagation();
              setSelected(i);
              log(`zone ${i} selected`);
            }}
          >
            <boxGeometry args={[5, 2, 5]} />
            <meshStandardMaterial color={selected === i ? 'orange' : 'gray'} />
            {z.warnings && z.warnings.length > 0 && (
              <Html position={[0, 2, 0]}>
                <div className="bg-red-600 text-white text-xs px-1 rounded">
                  {z.warnings.join(', ')}
                </div>
              </Html>
            )}
          </mesh>
          {selected === i && (
            <TransformControls
              position={[0, 0, 0]}
              onMouseUp={(e) => {
                const obj = (e.target as any).object as THREE.Object3D;
                const p = obj.getWorldPosition(new THREE.Vector3());
                handleTransform(i, p);
                setSelected(null);
              }}
            />
          )}
        </group>
      ))}
      {zones.length > 1 && (
        <line>
          <primitive
            object={new THREE.BufferGeometry().setFromPoints(pathPts)}
            attach="geometry"
          />
          <lineBasicMaterial attach="material" color="yellow" />
        </line>
      )}
      <mesh ref={headRef} visible={animating || simIndex >= 0}>
        <sphereGeometry args={[2, 16, 16]} />
        <meshStandardMaterial color="white" />
      </mesh>
    </group>
  );
}

