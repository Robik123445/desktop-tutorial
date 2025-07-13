import React, { useRef, useState, useEffect } from 'react';
import * as THREE from 'three';
import log from './logger';

/**
 * Upload and visualize a heightmap from JSON.
 * JSON format: { "points": [{"x":0,"y":0,"z":0}, ...] }
 */
export default function HeightmapViewer() {
  const containerRef = useRef(null);
  const [heightmap, setHeightmap] = useState(null);
  const [hoverInfo, setHoverInfo] = useState(null);

  // Handle file selection
  const handleFile = (e) => {
    const file = e.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const data = JSON.parse(reader.result);
        if (!data.points) throw new Error('Invalid heightmap');
        setHeightmap(data.points);
        log(`Heightmap loaded: ${file.name}`);
      } catch (err) {
        console.error('Failed to parse heightmap', err);
      }
    };
    reader.readAsText(file);
  };

  // Render three.js surface when heightmap changes
  useEffect(() => {
    if (!heightmap || !containerRef.current) return;

    const width = containerRef.current.clientWidth;
    const height = 300;

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
    camera.position.set(0, -Math.max(...heightmap.map(p => p.y)) - 5, 10);
    camera.lookAt(0, 0, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    containerRef.current.innerHTML = '';
    containerRef.current.appendChild(renderer.domElement);

    const xs = [...new Set(heightmap.map(p => p.x))].sort((a,b)=>a-b);
    const ys = [...new Set(heightmap.map(p => p.y))].sort((a,b)=>a-b);
    const xCount = xs.length;
    const yCount = ys.length;

    const geometry = new THREE.PlaneGeometry(
      xs[xCount-1]-xs[0],
      ys[yCount-1]-ys[0],
      xCount-1,
      yCount-1
    );

    const zValues = [];
    for (let j=0; j<yCount; j++) {
      for (let i=0; i<xCount; i++) {
        const point = heightmap.find(p => p.x===xs[i] && p.y===ys[j]);
        zValues.push(point ? point.z : 0);
      }
    }
    const minZ = Math.min(...zValues);
    const maxZ = Math.max(...zValues);
    geometry.rotateX(-Math.PI/2);
    geometry.translate(-(xs[xCount-1]+xs[0])/2, 0, (ys[yCount-1]+ys[0])/2);

    const positions = geometry.attributes.position;
    for (let i=0;i<positions.count;i++) {
      positions.setY(i, zValues[i]);
    }
    positions.needsUpdate = true;

    const colors = [];
    for (const z of zValues) {
      const t = (z - minZ) / (maxZ - minZ || 1);
      const color = new THREE.Color();
      color.setHSL((1 - t) * 0.33, 1, 0.5); // green to red
      colors.push(color.r, color.g, color.b);
    }
    geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));

    const material = new THREE.MeshStandardMaterial({ vertexColors: true, side: THREE.DoubleSide });
    const mesh = new THREE.Mesh(geometry, material);
    scene.add(mesh);

    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(0, 10, 10);
    scene.add(light);

    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();
    function onMove(event) {
      const rect = renderer.domElement.getBoundingClientRect();
      mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
      mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(mouse, camera);
      const intersects = raycaster.intersectObject(mesh);
      if (intersects.length > 0) {
        const idx = intersects[0].face.a;
        const x = xs[idx % xCount];
        const y = ys[Math.floor(idx / xCount)];
        const z = zValues[idx];
        setHoverInfo({ x, y, z });
      } else {
        setHoverInfo(null);
      }
    }
    renderer.domElement.addEventListener('mousemove', onMove);

    function animate() {
      requestAnimationFrame(animate);
      renderer.render(scene, camera);
    }
    animate();

    return () => {
      renderer.domElement.removeEventListener('mousemove', onMove);
      geometry.dispose();
      material.dispose();
      renderer.dispose();
    };
  }, [heightmap]);

  return (
    <div className="space-y-4 p-4" data-testid="viewer">
      <input type="file" accept=".json" onChange={handleFile} className="border p-2" aria-label="load heightmap" />
      <div ref={containerRef} className="w-full" data-testid="canvas" />
      {hoverInfo && (
        <div className="text-sm bg-white p-1 border inline-block">
          {`X:${hoverInfo.x} Y:${hoverInfo.y} Z:${hoverInfo.z}`}
        </div>
      )}
    </div>
  );
}
