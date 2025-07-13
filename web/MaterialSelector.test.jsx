import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import MaterialSelector from './MaterialSelector';
jest.mock('./logger', () => jest.fn());
import log from './logger';

test('shows feed and speed when selecting material and custom input', () => {
  render(<MaterialSelector />);
  const select = screen.getByTestId('material-select');
  fireEvent.change(select, { target: { value: 'MDF' } });
  expect(screen.getByTestId('speed')).toHaveTextContent('16000');
  expect(screen.getByTestId('feed')).toHaveTextContent('900');
  expect(log).toHaveBeenCalledWith('Material selected: MDF');

  fireEvent.change(select, { target: { value: 'Custom' } });
  expect(screen.getByTestId('custom-input')).toBeInTheDocument();
  expect(log).toHaveBeenCalledWith('Material selected: Custom');
});
