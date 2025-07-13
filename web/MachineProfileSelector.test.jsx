import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import MachineProfileSelector from './MachineProfileSelector';

jest.mock('./logger', () => jest.fn());
import log from './logger';

beforeEach(() => {
  localStorage.clear();
});

test('selects profile and edits custom values', () => {
  render(<MachineProfileSelector />);
  const select = screen.getByTestId('profile-select');
  expect(select.value).toBe('GRBL Small');

  fireEvent.change(select, { target: { value: 'Smoothieware' } });
  expect(screen.getByTestId('value-Feedrate')).toHaveTextContent('3000');
  expect(log).toHaveBeenCalledWith('Machine profile selected: Smoothieware');

  fireEvent.change(select, { target: { value: 'Custom' } });
  const inputX = screen.getByTestId('input-X limit');
  fireEvent.change(inputX, { target: { value: '400' } });
  expect(localStorage.getItem('machineProfile')).toContain('400');
});
