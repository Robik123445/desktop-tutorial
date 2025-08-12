import React from 'react';
import { render, screen } from '@testing-library/react';
import MachineMonitor from './MachineMonitor';

jest.useFakeTimers();
jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

test('monitor updates status periodically', () => {
  render(<MachineMonitor />);
  jest.advanceTimersByTime(1000);
  expect(log).toHaveBeenCalled();
  expect(screen.getByText(/X:/)).toBeInTheDocument();
});
