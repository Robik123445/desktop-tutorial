import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import HeightmapViewer from './HeightmapViewer';
jest.mock('./logger', () => jest.fn());
import log from './logger';

test('loads heightmap and logs file name', () => {
  const data = { points: [{ x: 0, y: 0, z: 0 }] };
  const file = new File([JSON.stringify(data)], 'map.json', { type: 'application/json' });
  render(<HeightmapViewer />);
  const input = screen.getByLabelText(/load heightmap/i);
  fireEvent.change(input, { target: { files: [file] } });
  expect(log).toHaveBeenCalledWith('Heightmap loaded: map.json');
  expect(screen.getByTestId('canvas')).toBeInTheDocument();
});
