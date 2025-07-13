import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import BugReportForm from './BugReportForm';
jest.mock('./logger', () => jest.fn());
import log from './logger';

test('logs bug report', () => {
  render(<BugReportForm />);
  fireEvent.change(screen.getByTestId('bug-input'), { target: { value: 'bug' } });
  fireEvent.click(screen.getByTestId('bug-send'));
  expect(log).toHaveBeenCalledWith('bug report: bug');
});
