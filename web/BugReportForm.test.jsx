import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import BugReportForm from './BugReportForm';
jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

test('logs bug report', () => {
  render(<BugReportForm />);
  fireEvent.change(screen.getByTestId('bug-input'), { target: { value: 'bug' } });
  fireEvent.click(screen.getByTestId('bug-send'));
  expect(log).toHaveBeenCalledWith('bug report: bug');
});
