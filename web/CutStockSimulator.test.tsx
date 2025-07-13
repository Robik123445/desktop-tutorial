import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import CutStockSimulator from './CutStockSimulator';
import * as THREE from 'three';
jest.mock('./logger', () => jest.fn());
import log from './logger';
jest.mock('file-saver', () => ({ saveAs: jest.fn() }));
import { saveAs } from 'file-saver';

test('logs start and export events', () => {
  const tp = {
    object: new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial()),
    points: [new THREE.Vector3(), new THREE.Vector3(1,0,0)],
    type: 'milling',
    head: 'spindle',
    bbox: new THREE.Box3(),
    name: 'p',
    layers: [[new THREE.Vector3(), new THREE.Vector3(1,0,0)]],
    diagnostics: []
  };
  render(<CutStockSimulator toolpaths={[tp]} stock={{x:10,y:10,z:5}} toolRadius={1} />);
  fireEvent.click(screen.getByText('Play'));
  expect(log).toHaveBeenCalledWith('cut stock sim start');
  fireEvent.click(screen.getByText('Export STL'));
  expect(saveAs).toHaveBeenCalled();
  expect(log).toHaveBeenCalledWith('cut stock STL exported');
});
