import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import OperationAssigner from './OperationAssigner';
jest.mock('./logger', () => jest.fn());
import log from './logger';

test('assigns operations and heads with summary', () => {
  render(<OperationAssigner paths={[[], []]} />);
  expect(screen.getByTestId('summary')).toHaveTextContent('Spindle: 2');

  fireEvent.change(screen.getByTestId('op-0'), { target: { value: 'Engrave' } });
  expect(screen.getByTestId('head-0').value).toBe('Laser');
  fireEvent.change(screen.getByTestId('head-1'), { target: { value: 'Print Head' } });

  const summary = screen.getByTestId('summary');
  expect(summary).toHaveTextContent('Laser: 1');
  expect(summary).toHaveTextContent('Print Head: 1');
  expect(log).toHaveBeenCalledWith('Path 0 operation -> Engrave');
  expect(log).toHaveBeenCalledWith('Path 1 head -> Print Head');
});
