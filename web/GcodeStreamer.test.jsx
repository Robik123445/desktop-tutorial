import React from 'react';
import { render, screen, fireEvent, act } from '@testing-library/react';
import GcodeStreamer from './GcodeStreamer';
jest.useFakeTimers();
jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

test('streams lines in blocks of five', () => {
  const lines = Array.from({ length: 12 }, (_, i) => `G1 X${i}`);
  render(<GcodeStreamer gcodeLines={lines} fileName="test.gcode" />);
  fireEvent.click(screen.getByText('Start Streaming'));
  act(() => {
    jest.advanceTimersByTime(210); // first batch
  });
  const list = screen.getByTestId('last-lines');
  expect(list.textContent).toContain('G1 X4'); // last line in first block
  expect(log).toHaveBeenCalledWith('Start streaming test.gcode');
});

