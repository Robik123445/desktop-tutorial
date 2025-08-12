import React from 'react';
import { render, screen } from '@testing-library/react';
import SurfaceAnalyzer from './SurfaceAnalyzer';
jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

test('detects collision and no contact', () => {
  const gcode = 'G1 X0 Y0 Z0\nG1 X1 Y0 Z-1\nG1 X2 Y0 Z0.3';
  const heightmap = { points: [ {x:0,y:0,z:0}, {x:1,y:0,z:0}, {x:2,y:0,z:0} ] };
  render(<SurfaceAnalyzer gcode={gcode} heightmap={heightmap} clearance={0.1} />);
  const list = screen.getByTestId('issues');
  expect(list.textContent).toMatch(/COLLISION/);
  expect(list.textContent).toMatch(/NO_CONTACT/);
  expect(log).toHaveBeenCalledWith('analysis complete: 1 collisions, 1 high points');
});
