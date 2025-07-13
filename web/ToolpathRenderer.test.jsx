import React from 'react';
import { render } from '@testing-library/react';
import ToolpathRenderer from './ToolpathRenderer';
import * as THREE from 'three';

test('shows head mesh when animating', () => {
  const line = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial());
  const tp = { object: line, points: [new THREE.Vector3(), new THREE.Vector3(1,0,0)], type: 'milling', head: 'spindle', bbox: new THREE.Box3(), name: 'path', layers:[[new THREE.Vector3(), new THREE.Vector3(1,0,0)]], diagnostics:[] };
  const { container } = render(
    <ToolpathRenderer
      toolpaths={[tp]}
      visible={{ milling: true, drilling: true, engraving: true, laser_cut: true, laser_mark: true }}
      headsVisible={{ spindle: true, laser: true, picker: true }}
      animating={true}
      activeLayer={0}
      activeSegment={0}
      mode="layer"
      diagnostics={false}
    />
  );
  expect(container.querySelector('mesh')).not.toBeNull();
});

test('clicking diagnostic calls onFocus', () => {
  const line = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial());
  const tp = {
    object: line,
    points: [new THREE.Vector3(), new THREE.Vector3(1, 0, 0)],
    type: 'milling',
    head: 'spindle',
    bbox: new THREE.Box3(),
    name: 'path',
    layers: [[new THREE.Vector3(), new THREE.Vector3(1, 0, 0)]],
    diagnostics: [
      {
        layer: 0,
        index: 1,
        position: new THREE.Vector3(0.5, 0, 0),
        message: 'sharp corner',
      },
    ],
  };
  const onFocus = jest.fn();
  const { container } = render(
    <ToolpathRenderer
      toolpaths={[tp]}
      visible={{ milling: true, drilling: true, engraving: true, laser_cut: true, laser_mark: true }}
      headsVisible={{ spindle: true, laser: true, picker: true }}
      animating={false}
      activeLayer={0}
      activeSegment={0}
      mode="layer"
      diagnostics={true}
      onFocus={onFocus}
    />
  );
  const mesh = container.querySelector('mesh');
  if (mesh) {
    mesh.dispatchEvent(new MouseEvent('click', { bubbles: true }));
  }
  expect(onFocus).toHaveBeenCalled();
});
