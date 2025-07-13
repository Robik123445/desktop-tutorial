import React from 'react';
import { render, screen } from '@testing-library/react';
import ZoneSummary from './ZoneSummary';
jest.mock('./logger', () => jest.fn());

const zones = [
  { base: [0, 0], toolpath: [[0,0,0]], move_cmds: ['MOVE_BASE X0 Y0'] },
  { base: [100, 0], toolpath: [[100,0,0]], move_cmds: ['G1 Z50', 'MOVE_BASE X100 Y0'] },
];

test('renders list of zones', () => {
  render(<ZoneSummary zones={zones} />);
  expect(screen.getByTestId('zone-list').children.length).toBe(2);
});
