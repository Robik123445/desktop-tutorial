import React from 'react';
import { render, fireEvent, screen, act } from '@testing-library/react';
import Viewport3D from './Viewport3D';
jest.mock('./logger', () => jest.fn());
import log from './logger';
jest.useFakeTimers();
jest.mock('file-saver', () => ({ saveAs: jest.fn() }));
import { saveAs } from 'file-saver';
jest.mock('jspdf', () => {
  return jest.fn().mockImplementation(() => ({
    text: jest.fn(),
    addImage: jest.fn(),
    save: jest.fn(),
  }));
});
import jsPDF from 'jspdf';
jest.mock('./viewportUtils', () => {
  const THREE = require('three');
  return {
    loadModel: jest.fn(() =>
      Promise.resolve({
        object: {},
        bbox: new THREE.Box3(),
        name: 'model.stl',
        size: new THREE.Vector3(1, 1, 1),
      })
    ),
    loadToolpaths: jest.fn(() =>
      Promise.resolve([
        {
          object: {},
          points: [new THREE.Vector3(), new THREE.Vector3(1, 0, 0)],
          type: 'milling',
          bbox: new THREE.Box3(),
          name: 'path.gcode',
          layers: [[new THREE.Vector3(), new THREE.Vector3(1, 0, 0)]],
          diagnostics: [],
        },
      ])
    ),
  };
});
import { loadModel, loadToolpaths } from './viewportUtils';
jest.mock('./viewportDiagnostics', () => {
  const THREE = require('three');
  return {
    analyzeScene: jest.fn(() => [
      { position: new THREE.Vector3(0,0,0), message: 'test warn', severity: 'warning' }
    ])
  };
});
jest.mock('@react-three/drei', () => ({
  OrbitControls: ({ children }: any) => <div>{children}</div>,
  TransformControls: () => <div data-testid="transform-controls" />,
}));

test('logs mount', () => {
  render(<Viewport3D />);
  expect(log).toHaveBeenCalledWith('Viewport3D mounted');
});

test('imports model on drop', async () => {
  render(<Viewport3D />);
  const file = new File(['dummy'], 'model.stl', { type: 'model/stl' });
  await act(async () => {
    fireEvent.drop(screen.getByTestId('viewport'), { dataTransfer: { files: [file] } });
  });
  expect(loadModel).toHaveBeenCalledWith(file);
  expect(log).toHaveBeenCalledWith('file dropped: model.stl');
  expect(await screen.findByText('model.stl')).toBeInTheDocument();
});

test('selects model and shows transform controls', async () => {
  render(<Viewport3D />);
  const file = new File(['dummy'], 'model.stl', { type: 'model/stl' });
  await act(async () => {
    fireEvent.drop(screen.getByTestId('viewport'), { dataTransfer: { files: [file] } });
  });
  fireEvent.click(await screen.findByText('Select'));
  expect(screen.getByTestId('transform-controls')).toBeInTheDocument();
});

test('imports gcode toolpaths', async () => {
  render(<Viewport3D />);
  const file = new File(['G1 X0 Y0'], 'path.gcode', { type: 'text/plain' });
  await act(async () => {
    fireEvent.drop(screen.getByTestId('viewport'), {
      dataTransfer: { files: [file] },
    });
  });
  expect(loadToolpaths).toHaveBeenCalledWith(file);
});

test('layer slider changes active layer', async () => {
  render(<Viewport3D />);
  const file = new File(['G1 X0 Y0'], 'path.gcode', { type: 'text/plain' });
  await act(async () => {
    fireEvent.drop(screen.getByTestId('viewport'), {
      dataTransfer: { files: [file] },
    });
  });
  const slider = screen.getByRole('slider');
  fireEvent.change(slider, { target: { value: '0' } });
  expect(slider).toHaveValue('0');
  expect(screen.getByTestId('layer-label')).toHaveTextContent('0');
});

test('segment slider selects toolpath', async () => {
  render(<Viewport3D />);
  const file = new File(['G1 X0 Y0'], 'path.gcode', { type: 'text/plain' });
  await act(async () => {
    fireEvent.drop(screen.getByTestId('viewport'), {
      dataTransfer: { files: [file] },
    });
  });
  fireEvent.change(screen.getByDisplayValue('Layer'), {
    target: { value: 'segment' },
  });
  const slider = screen.getByRole('slider');
  fireEvent.change(slider, { target: { value: '0' } });
  expect(slider).toHaveValue('0');
});

test('shows AI diagnostics summary', async () => {
  render(<Viewport3D />);
  const file = new File(['G1 X0 Y-500'], 'path.gcode', { type: 'text/plain' });
  await act(async () => {
    fireEvent.drop(screen.getByTestId('viewport'), {
      dataTransfer: { files: [file] },
    });
  });
  expect(await screen.findByText('AI Diagnostics')).toBeInTheDocument();
});

test('export snapshot logs action', async () => {
  render(<Viewport3D />);
  fireEvent.click(screen.getByText('Snapshot'));
  expect(log).toHaveBeenCalledWith('export snapshot');
  expect(saveAs).toHaveBeenCalled();
});

test('optimize order button logs action', async () => {
  render(<Viewport3D />);
  fireEvent.click(screen.getByText('Optimize Order'));
  expect(log).toHaveBeenCalledWith('optimize order');
});

test('simulation controls log play', () => {
  render(<Viewport3D />);
  fireEvent.click(screen.getByText('Play'));
  act(() => {
    jest.advanceTimersByTime(250);
  });
  expect(log).toHaveBeenCalledWith('simulation play');
});

test('measurement buttons log actions', () => {
  render(<Viewport3D />);
  fireEvent.click(screen.getByText('Dist'));
  expect(log).toHaveBeenCalledWith('measure distance');
  fireEvent.click(screen.getByText('Angle'));
  expect(log).toHaveBeenCalledWith('measure angle');
});
