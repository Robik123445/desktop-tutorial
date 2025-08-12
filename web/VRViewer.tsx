import React, { useEffect, useRef, useState } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
import { VRButton } from 'three/examples/jsm/webxr/VRButton.js';
import { ARButton } from 'three/examples/jsm/webxr/ARButton.js';
import log from './src/log.js';

interface Point { x: number; y: number; z: number; }

function XRSetup({ mode }: { mode: 'vr' | 'ar' }) {
  const { gl } = useThree();
  useEffect(() => {
    gl.xr.enabled = true;
    const button = mode === 'vr' ? VRButton.createButton(gl) : ARButton.createButton(gl);
    document.body.appendChild(button);
    log(`XR mode enabled: ${mode}`);
    return () => {
      gl.xr.enabled = false;
      button.remove();
      log('XR mode disabled');
    };
  }, [gl, mode]);
  return null;
}

function MovingHead({ points, running }: { points: Point[]; running: boolean }) {
  const mesh = useRef<THREE.Mesh>(null!);
  const index = useRef(0);
  useFrame((_, delta) => {
    if (!running || points.length === 0) return;
    const cur = points[index.current];
    mesh.current.position.set(cur.x, cur.y, cur.z);
    index.current = Math.min(index.current + Math.ceil(delta * 10), points.length - 1);
  });
  return (
    <mesh ref={mesh}>
      <sphereGeometry args={[1, 16, 16]} />
      <meshStandardMaterial color="red" />
    </mesh>
  );
}

export default function VRViewer({
  points = [],
  mode = 'vr',
}: {
  points?: Point[];
  mode?: 'vr' | 'ar';
}) {
  const [running, setRunning] = useState(false);
  const start = () => {
    log('VR simulation start');
    setRunning(true);
  };
  const stop = () => {
    log('VR simulation stop');
    setRunning(false);
  };

  const lineGeom = new THREE.BufferGeometry();
  if (points.length) {
    lineGeom.setAttribute('position', new THREE.Float32BufferAttribute(points.flatMap(p => [p.x, p.y, p.z]), 3));
  }

  return (
    <div className="w-full h-96 relative" data-testid="vr-viewer">
      <Canvas>
        <XRSetup mode={mode} />
        <ambientLight intensity={0.5} />
        <directionalLight position={[5, 5, 5]} />
        {points.length > 1 && (
          <line>
            <primitive object={lineGeom} attach="geometry" />
            <lineBasicMaterial color="orange" />
          </line>
        )}
        <MovingHead points={points} running={running} />
        <OrbitControls />
      </Canvas>
      <div className="absolute bottom-2 left-2 space-x-2">
        <button onClick={start} className="bg-green-600 text-white px-2 py-1 rounded">Play</button>
        <button onClick={stop} className="bg-red-600 text-white px-2 py-1 rounded">Stop</button>
      </div>
    </div>
  );
}
