import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import ProjectSaver from './ProjectSaver';
jest.mock('./logger', () => jest.fn());
import log from './logger';

const lines = ['G1 X0 Y0', 'G1 X1 Y0'];

test('shows notes textarea when checked and logs save', () => {
  render(<ProjectSaver gcodeLines={lines} />);
  fireEvent.change(screen.getByTestId('project-name'), { target: { value: 'Demo' } });
  fireEvent.click(screen.getByLabelText('Add notes'));
  fireEvent.change(screen.getByTestId('notes-area'), { target: { value: 'test' } });
  fireEvent.click(screen.getByText('Save Project'));
  expect(log).toHaveBeenCalledWith('Save project: Demo');
  expect(log).toHaveBeenCalledWith('Notes: test');
  expect(screen.getByTestId('toast')).toBeInTheDocument();
});

