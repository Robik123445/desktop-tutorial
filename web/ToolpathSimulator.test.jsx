import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import ToolpathSimulator from './ToolpathSimulator';

global.fetch = jest.fn(() =>
  Promise.resolve({ json: () => Promise.resolve({ points: [[0,0],[1,1]], plot: 'pngb64' }) })
);

test('calls API and displays results', async () => {
  render(<ToolpathSimulator />);
  fireEvent.change(screen.getByPlaceholderText(/Paste G-code/i), {
    target: { value: 'G0 X0 Y0\nG1 X1 Y1' },
  });
  fireEvent.click(screen.getByText('Simulate'));
  expect(global.fetch).toHaveBeenCalledWith('/simulate', expect.any(Object));
  await screen.findByTestId('points');
  expect(screen.getByTestId('plot')).toBeInTheDocument();
});
