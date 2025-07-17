import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import HeightmapEditor from './src/HeightmapEditor';

global.prompt = jest.fn(() => '5');

beforeEach(() => {
  global.fetch = jest.fn(() => Promise.resolve());
});

test('updates cell value and logs change', () => {
  render(<HeightmapEditor heightmapData={[[0, 1], [2, 3]]} />);

  const cell = screen.getByTestId('cell-0-0');
  fireEvent.click(cell);

  expect(global.prompt).toHaveBeenCalled();
  expect(cell.dataset.value).toBe('5');
  expect(global.fetch).toHaveBeenCalledWith('/log', expect.any(Object));
});
