import React from 'react';
import * as THREE from 'three';

interface Props {
  forbidden: [number, number, number, number][];
  airMove?: [number, number, number, number][];
  visible?: boolean;
}

/**
 * Render GeoFence zones as colored rectangles in the viewport.
 */
export default function GeoFenceOverlay({
  forbidden,
  airMove = [],
  visible = true,
}: Props) {
  if (!visible) return null;

  const toMesh = (b: [number, number, number, number], color: string, key: string) => {
    const [x1, y1, x2, y2] = b;
    const w = Math.abs(x2 - x1);
    const h = Math.abs(y2 - y1);
    const cx = (x1 + x2) / 2;
    const cy = (y1 + y2) / 2;
    return (
      <mesh key={key} position={[cx, 0, cy]}>
        <boxGeometry args={[w, 0.1, h]} />
        <meshBasicMaterial color={color} opacity={0.3} transparent />
      </mesh>
    );
  };

  return (
    <group>
      {forbidden.map((b, i) => toMesh(b, 'red', `f${i}`))}
      {airMove.map((b, i) => toMesh(b, 'blue', `a${i}`))}
    </group>
  );
}
