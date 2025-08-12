import React from 'react';
import { render, screen, fireEvent, act } from '@testing-library/react';
import LayerPreview from './LayerPreview';
jest.useFakeTimers();
jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

test('changes layer and plays preview', () => {
  const layers = [
    [[[0,0,0],[10,0,0]]],
    [[[0,0,0],[0,10,-1]]]
  ];
  render(<LayerPreview layers={layers} />);
  // change via slider
  const slider = screen.getByRole('slider');
  fireEvent.change(slider, { target: { value: '1' } });
  expect(slider.value).toBe('1');
  expect(screen.getAllByTestId('line')).toHaveLength(1);
  // play preview
  fireEvent.click(screen.getByText('Play preview'));
  act(() => {
    jest.advanceTimersByTime(600);
  });
  expect(log).toHaveBeenCalledWith('Play preview');
});
