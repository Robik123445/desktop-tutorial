import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { Html } from '@react-three/drei';
import { ToolpathData, ToolpathType, Diagnostic, HEAD_COLORS, ToolHead } from './viewportUtils';
import log from './logger';
const isMobile = typeof window !== 'undefined' && window.innerWidth < 768;

interface ToolpathRendererProps {
  toolpaths: ToolpathData[];
  visible: Record<ToolpathType, boolean>;
  headsVisible: Record<ToolHead, boolean>;
  animating: boolean;
  activeLayer: number;
  activeSegment: number;
  mode: 'layer' | 'segment';
  diagnostics: boolean;
  materialPreview?: boolean;
  onFocus?: (pos: THREE.Vector3) => void;
  simIndex?: number;
  startIndices?: number[];
}

function DiagnosticMarker({ diag, onFocus }: { diag: Diagnostic; onFocus?: (p: THREE.Vector3) => void }) {
  const [hover, setHover] = useState(false);
  return (
    <mesh
      position={diag.position}
      onPointerOver={() => setHover(true)}
      onPointerOut={() => setHover(false)}
      onClick={() => {
        log(`diagnostic clicked: ${diag.message}`);
        onFocus && onFocus(diag.position);
      }}
    >
      <sphereGeometry args={[1, 8, 8]} />
      <meshStandardMaterial color="red" />
      {hover && (
        <Html distanceFactor={10} style={{ pointerEvents: 'none' }}>
          <div className="bg-red-600 text-white text-xs px-1 rounded">
            {diag.message}
          </div>
        </Html>
      )}
    </mesh>
  );
}

function HeadIcon({ head }: { head: ToolHead }) {
  switch (head) {
    case 'laser':
      return (
        <mesh>
          <coneGeometry args={[2, 4, 8]} />
          <meshStandardMaterial color={HEAD_COLORS.laser} />
        </mesh>
      );
    case 'picker':
      return (
        <mesh>
          <boxGeometry args={[2, 2, 2]} />
          <meshStandardMaterial color={HEAD_COLORS.picker} />
        </mesh>
      );
    default:
      return (
        <mesh>
          <cylinderGeometry args={[1, 1, 4, 8]} />
          <meshStandardMaterial color={HEAD_COLORS.spindle} />
        </mesh>
      );
  }
}

/**
 * Render toolpaths with optional head animation.
 * Modular component so new toolheads can be added easily.
 */
function ToolpathRenderer({
  toolpaths,
  visible,
  headsVisible,
  animating,
  activeLayer,
  activeSegment,
  mode,
  diagnostics,
  materialPreview = false,
  onFocus,
  simIndex = -1,
  startIndices = [],
}: ToolpathRendererProps) {
  const headRef = useRef<THREE.Mesh>(null);

  // Animate a small sphere along the currently visible toolpath points
  useEffect(() => {
    if (!animating) return;
    const pathPts = toolpaths
      .filter((t, i) => {
        if (!visible[t.type] || !headsVisible[t.head]) return false;
        if (mode === 'segment') return i === activeSegment;
        return true;
      })
      .flatMap((t) => {
        const pts = mode === 'layer' ? t.layers[activeLayer] || [] : t.points;
        const rots = mode === 'layer' ? t.rotationLayers?.[activeLayer] || [] : t.rotations;
        return pts.map((p, idx) => ({ p, head: t.head, rot: rots[idx] || new THREE.Vector3() }));
      });
    if (!pathPts.length) return;
    let i = 0;
    const stepTime = isMobile ? 60 : 30;
    const timer = setInterval(() => {
      const { p, head, rot } = pathPts[i];
      if (headRef.current) {
        headRef.current.position.copy(p);
        const mat = headRef.current.material as THREE.Material;
        (mat as any).color = new THREE.Color(HEAD_COLORS[head]);
        headRef.current.rotation.set(
          THREE.MathUtils.degToRad(rot.x),
          THREE.MathUtils.degToRad(rot.y),
          THREE.MathUtils.degToRad(rot.z)
        );
      }
      i += 1;
      if (i >= pathPts.length) clearInterval(timer);
    }, stepTime);
    return () => clearInterval(timer);
  }, [animating, toolpaths, visible, activeLayer, activeSegment, mode]);

  // update head position on external simulation index
  useEffect(() => {
    if (simIndex < 0) return;
    const allPoints = toolpaths.flatMap(tp => tp.points);
    const allRots = toolpaths.flatMap(tp => tp.rotations);
    const idx = Math.min(simIndex, allPoints.length - 1);
    const pt = allPoints[idx];
    const rot = allRots[idx] || new THREE.Vector3();
    if (pt && headRef.current) {
      headRef.current.position.copy(pt);
      headRef.current.rotation.set(
        THREE.MathUtils.degToRad(rot.x),
        THREE.MathUtils.degToRad(rot.y),
        THREE.MathUtils.degToRad(rot.z)
      );
    }
  }, [simIndex, toolpaths]);

  return (
    <>
      {toolpaths.map((tp, idx) => {
        if (!visible[tp.type] || !headsVisible[tp.head]) return null;
        const color = HEAD_COLORS[tp.head];
        const start = startIndices[idx] || 0;
        const done = Math.max(0, Math.min(simIndex - start + 1, tp.points.length));
        return tp.layers.map((layerPts, li) => {
          const layerStartIdx = tp.points.indexOf(layerPts[0]);
          const layerEndIdx = layerStartIdx + layerPts.length;
          const processed = Math.max(0, Math.min(done - layerStartIdx, layerPts.length));
          const processedPts = layerPts.slice(0, processed);
          const remainingPts = layerPts.slice(Math.max(processed - 1, 0));
          const elems: JSX.Element[] = [];
          if (processedPts.length > 1) {
            const geom = new THREE.BufferGeometry().setFromPoints(processedPts);
            elems.push(
              <line key={`${idx}-${li}-done`} geometry={geom} material={new THREE.LineBasicMaterial({ color })} />
            );
          }
          if (remainingPts.length > 1) {
            const geomR = new THREE.BufferGeometry().setFromPoints(remainingPts);
            elems.push(
              <line
                key={`${idx}-${li}-rem`}
                geometry={geomR}
                material={new THREE.LineBasicMaterial({ color, opacity: 0.2, transparent: true })}
              />
            );
          }
          return elems;
        });
      })}
      {toolpaths.map((tp, idx) =>
        visible[tp.type] && headsVisible[tp.head] ? (
          <group key={`icon-${idx}`} position={tp.points[0]}>
            <HeadIcon head={tp.head} />
          </group>
        ) : null
      )}
      {diagnostics &&
        toolpaths.map((tp, ti) =>
          tp.diagnostics
            .filter((d) =>
              mode === 'segment'
                ? ti === activeSegment && visible[tp.type] && headsVisible[tp.head]
                : d.layer === activeLayer && visible[tp.type] && headsVisible[tp.head]
            )
            .map((d, di) => (
              <DiagnosticMarker
                key={`diag-${ti}-${di}`}
                diag={d}
                onFocus={onFocus}
              />
            ))
        )}
      {animating && (
        <mesh ref={headRef} data-testid="head">
          <sphereGeometry args={[2, 16, 16]} />
          <meshStandardMaterial color="white" />
        </mesh>
      )}
    </>
  );
}

export default React.memo(ToolpathRenderer);