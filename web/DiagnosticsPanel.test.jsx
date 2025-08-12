import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import DiagnosticsPanel from './DiagnosticsPanel';
jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

test('diagnostics actions log events', () => {
  render(<DiagnosticsPanel />);
  fireEvent.click(screen.getByTestId('run-tests'));
  fireEvent.click(screen.getByTestId('gen-report'));
  expect(log).toHaveBeenCalledWith('diagnostics run tests');
  expect(log).toHaveBeenCalledWith('diagnostics report');
});
