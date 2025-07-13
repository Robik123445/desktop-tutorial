import React from 'react';
import { render, screen } from '@testing-library/react';
import MachineMonitor from './MachineMonitor';

jest.useFakeTimers();
jest.mock('./logger', () => jest.fn());
import log from './logger';

test('monitor updates status periodically', () => {
  render(<MachineMonitor />);
  jest.advanceTimersByTime(1000);
  expect(log).toHaveBeenCalled();
  expect(screen.getByText(/X:/)).toBeInTheDocument();
});
