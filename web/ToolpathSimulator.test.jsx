import React from 'react';
import { render, screen, fireEvent, act } from '@testing-library/react';
import ToolpathSimulator from './ToolpathSimulator';
jest.useFakeTimers();
jest.mock('./logger', () => jest.fn());
import log from './logger';

test('plays through points and logs', () => {
  const gcode = 'G1 X0 Y0\nG1 X1 Y0\nG1 X1 Y1';
  render(<ToolpathSimulator gcode={gcode} />);
  fireEvent.click(screen.getByText('Play'));
  act(() => {
    jest.advanceTimersByTime(320);
  });
  expect(log).toHaveBeenCalledWith('simulation start');
  expect(screen.getByTestId('coords').textContent).toContain('X1');
});
