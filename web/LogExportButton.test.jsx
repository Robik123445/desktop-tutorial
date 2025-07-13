import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import LogExportButton from './LogExportButton';

jest.mock('./logger', () => {
  const log = jest.fn();
  log.getLogContent = jest.fn(() => 'log');
  return log;
});

jest.mock('file-saver', () => ({ saveAs: jest.fn() }));
import { saveAs } from 'file-saver';
import log from './logger';

test('exports logs when clicked', () => {
  render(<LogExportButton />);
  fireEvent.click(screen.getByText('Export Logs'));
  expect(log.getLogContent).toHaveBeenCalled();
  expect(saveAs).toHaveBeenCalled();
  expect(log).toHaveBeenCalledWith('User exported logs');
});
