import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import SetupWizard from './SetupWizard';
jest.mock('./logger', () => jest.fn());
import log from './logger';

test('wizard steps and export', () => {
  const file = new File(['<svg></svg>'], 'design.svg', { type: 'image/svg+xml' });
  render(<SetupWizard gcodeLines={['G1 X0 Y0']} />);
  fireEvent.change(screen.getByTestId('design-input'), { target: { files: [file] } });
  fireEvent.click(screen.getByTestId('next-btn')); // to step 2
  fireEvent.click(screen.getByTestId('next-btn')); // to step 3
  fireEvent.click(screen.getByTestId('next-btn')); // to step 4
  fireEvent.click(screen.getByTestId('next-btn')); // to step 5
  expect(screen.getByTestId('summary')).toBeInTheDocument();
  fireEvent.click(screen.getByTestId('export-btn'));
  expect(log).toHaveBeenCalledWith('wizard export gcode');
});
