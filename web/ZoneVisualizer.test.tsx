import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import ZoneVisualizer, { ZonePlan } from './ZoneVisualizer';
jest.mock('./logger', () => jest.fn());
import log from './logger';

const zones: ZonePlan[] = [
  { base: [0, 0], toolpath: [[0,0,0]], move_cmds: ['MOVE_BASE X0 Y0'] },
  { base: [100, 0], toolpath: [[100,0,0]], move_cmds: ['MOVE_BASE X100 Y0'], warnings: ['warn'] },
];

test('renders base positions', () => {
  const { container } = render(<ZoneVisualizer zones={zones} />);
  expect(container.querySelectorAll('mesh').length).toBeGreaterThan(0);
});

test('clicking zone logs selection', () => {
  const { container } = render(<ZoneVisualizer zones={zones} />);
  const mesh = container.querySelector('mesh');
  if (mesh) {
    fireEvent.click(mesh);
    expect(log).toHaveBeenCalledWith('zone 0 selected');
  }
});
