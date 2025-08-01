import { render, screen, fireEvent } from '@testing-library/react';
import ToolpathPreview from './src/ToolpathPreview';

jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

it('parses moves and animates play', () => {
  jest.useFakeTimers();
  const code = 'G1 X0 Y0\nG1 X1 Y0\nG1 X1 Y1';
  render(<ToolpathPreview gcode={code} />);
  expect(screen.getByText('Move 1 / 3')).toBeInTheDocument();
  fireEvent.click(screen.getByText('Play'));
  jest.advanceTimersByTime(350);
  expect(screen.getByText('Move 2 / 3')).toBeInTheDocument();
  expect(log).toHaveBeenCalledWith('play');
});
