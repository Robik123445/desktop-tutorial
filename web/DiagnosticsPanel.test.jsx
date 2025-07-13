import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import DiagnosticsPanel from './DiagnosticsPanel';
jest.mock('./logger', () => jest.fn());
import log from './logger';

test('diagnostics actions log events', () => {
  render(<DiagnosticsPanel />);
  fireEvent.click(screen.getByTestId('run-tests'));
  fireEvent.click(screen.getByTestId('gen-report'));
  expect(log).toHaveBeenCalledWith('diagnostics run tests');
  expect(log).toHaveBeenCalledWith('diagnostics report');
});
