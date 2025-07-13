import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import HapticToolpathEditor from './HapticToolpathEditor';
jest.mock('./logger', () => jest.fn());
import log from './logger';

test('draws path and logs export', () => {
  const { container, getByText } = render(<HapticToolpathEditor width={200} height={200} />);
  const svg = container.querySelector('svg');
  fireEvent.pointerDown(svg, { clientX: 10, clientY: 10 });
  fireEvent.pointerMove(svg, { clientX: 50, clientY: 50 });
  fireEvent.pointerUp(svg);
  expect(container.querySelectorAll('polyline').length).toBe(1);
  fireEvent.click(getByText('Export as G-code'));
  expect(log).toHaveBeenCalledWith('export G-code');
});
