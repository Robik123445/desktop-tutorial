import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import ResumeJobButton from './ResumeJobButton';
jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

beforeEach(() => {
  global.fetch = jest.fn(() =>
    Promise.resolve({ json: () => Promise.resolve({ line: 10 }) })
  );
});

test('shows resume info and logs on click', async () => {
  const onResume = jest.fn();
  render(<ResumeJobButton onResume={onResume} />);
  await waitFor(() => screen.getByText(/Last line/));
  fireEvent.click(screen.getByText('Resume interrupted job'));
  expect(onResume).toHaveBeenCalledWith({ line: 10 });
  expect(log).toHaveBeenCalledWith('resume job clicked');
});
