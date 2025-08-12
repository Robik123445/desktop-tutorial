import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import CuraSlicerUI from './CuraSlicerUI';
jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

test('selecting section expands content and updates panel', () => {
  render(<CuraSlicerUI />);
  const btn = screen.getByTestId('section-Import & Model');
  fireEvent.click(btn);
  expect(screen.getByTestId('section-content')).toBeInTheDocument();
  expect(screen.getByTestId('right-panel')).toHaveTextContent('Import & Model Settings');
  expect(log).toHaveBeenCalledWith('section selected: Import & Model');
});

test('dropping file on canvas shows preview and details', () => {
  const file = new File(['dummy'], 'shape.svg', { type: 'image/svg+xml' });
  render(<CuraSlicerUI />);
  const canvas = screen.getByTestId('canvas');
  fireEvent.drop(canvas, { dataTransfer: { files: [file] } });
  expect(log).toHaveBeenCalledWith('canvas import: shape.svg');
  expect(screen.getByAltText('preview')).toBeInTheDocument();
  expect(screen.getByTestId('right-panel')).toHaveTextContent('shape.svg');
});

test('assign operations via tools menu', () => {
  const file = new File(['dummy'], 'part.svg', { type: 'image/svg+xml' });
  render(<CuraSlicerUI />);
  fireEvent.drop(screen.getByTestId('canvas'), { dataTransfer: { files: [file] } });
  fireEvent.click(screen.getByTestId('section-Tools & Operations'));
  expect(screen.getByTestId('right-panel')).toHaveTextContent('Path 1');
  fireEvent.change(screen.getByTestId('op-0'), { target: { value: 'Engrave' } });
  expect(screen.getByTestId('summary')).toHaveTextContent('Laser: 1');
});

test('open optimization and simulation panels', () => {
  render(<CuraSlicerUI />);
  fireEvent.click(screen.getByTestId('section-Toolpath Optimization'));
  expect(screen.getByTestId('right-panel')).toHaveTextContent('Use optimizers');
  fireEvent.click(screen.getByTestId('section-Simulation'));
  expect(screen.getByTestId('right-panel')).toHaveTextContent('Simulation Stats');
});

test('export and monitoring sections display components', () => {
  render(<CuraSlicerUI />);
  fireEvent.click(screen.getByTestId('section-Export & Streaming'));
  expect(screen.getByText('GRBL')).toBeInTheDocument();
  fireEvent.click(screen.getByTestId('section-Monitoring'));
  expect(screen.getByText(/Status/)).toBeInTheDocument();
});

test('plugin toggles and diagnostics actions', () => {
  render(<CuraSlicerUI />);
  fireEvent.click(screen.getByTestId('section-AI Analyze & Optimize'));
  fireEvent.click(screen.getByTestId('plug-0'));
  expect(log).toHaveBeenCalledWith('plugin reverse_path -> off');
  fireEvent.click(screen.getByTestId('ai-analyze'));
  fireEvent.click(screen.getByTestId('ai-opt'));
  fireEvent.click(screen.getByTestId('section-Diagnostics'));
  fireEvent.click(screen.getByTestId('run-tests'));
  fireEvent.click(screen.getByTestId('gen-report'));
  expect(log).toHaveBeenCalledWith('diagnostics run tests');
  expect(log).toHaveBeenCalledWith('diagnostics report');
});

test('experimental features toggle and debris scan', () => {
  render(<CuraSlicerUI />);
  fireEvent.click(screen.getByTestId('section-Experimental features'));
  const toggle = screen.getByLabelText(/Adaptive Toolpath/);
  fireEvent.click(toggle);
  expect(log).toHaveBeenCalledWith('adaptive mode -> true');
  fireEvent.click(screen.getByTestId('section-Export & Streaming'));
  fireEvent.click(screen.getByText('Debris scan & avoid'));
  expect(log).toHaveBeenCalledWith('debris scan');
});

test('experimental AI panel shows analyzer and optimizer', () => {
  render(<CuraSlicerUI />);
  fireEvent.click(screen.getByTestId('section-Experimental AI'));
  expect(screen.getByTestId('right-panel')).toHaveTextContent('AI Feedback');
});

test('robot arm panel enables mode and displays diagnostics', () => {
  render(<CuraSlicerUI />);
  fireEvent.click(screen.getByTestId('section-Experimental â Robot Arm'));
  const toggle = screen.getByTestId('robot-toggle');
  fireEvent.click(toggle);
  expect(log).toHaveBeenCalledWith('robot arm mode -> true');
  expect(screen.getByTestId('profile-select')).toBeInTheDocument();
});
