import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import RobotDiagnosticsDashboard from './RobotDiagnosticsDashboard';

jest.mock('./logger', () => ({
  getLogContent: jest.fn(() => '2024 INFO Robot run started\n2024 WARNING something\n2024 ERROR fail\n2024 INFO feedback: ok'),
}));

import { getLogContent } from './logger';

test('displays parsed log stats', () => {
  render(<RobotDiagnosticsDashboard />);
  expect(screen.getByText(/Runs:/).textContent).toContain('1');
  expect(screen.getByText(/Warnings:/).textContent).toContain('1');
  expect(screen.getByText(/Errors:/).textContent).toContain('1');
  expect(screen.getByText(/Feedback:/).textContent).toContain('1');
  fireEvent.click(screen.getByText('Refresh'));
  expect(getLogContent).toHaveBeenCalledTimes(2);
});
