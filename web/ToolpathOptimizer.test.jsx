import React from 'react';
import { render, screen, fireEvent, act } from '@testing-library/react';
import ToolpathOptimizer from './ToolpathOptimizer';
jest.useFakeTimers();
jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

test('optimizes toolpath and logs actions', () => {
  const code = 'G1 X0 Y0\nG1 X1 Y0';
  render(<ToolpathOptimizer gcode={code} />);
  fireEvent.click(screen.getByTestId('opt-button'));
  act(() => { jest.advanceTimersByTime(310); });
  expect(screen.getByTestId('after').textContent).toContain(';OPT');
  expect(log).toHaveBeenCalledWith(expect.stringContaining('optimize request'));
  expect(log).toHaveBeenCalledWith('optimization complete');
});
