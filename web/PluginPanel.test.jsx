import { render, screen, fireEvent } from '@testing-library/react';
import PluginPanel from './src/PluginPanel';

beforeEach(() => {
  global.fetch = jest.fn((url) => {
    if (url === '/plugins') {
      return Promise.resolve({
        json: () => Promise.resolve([{ name: 'opt', description: 'Optimizer' }])
      });
    }
    return Promise.resolve({
      json: () => Promise.resolve({ ok: true })
    });
  });
});

test('runs selected plugin', async () => {
  render(<PluginPanel />);
  const btn = await screen.findByText('Run');
  fireEvent.click(btn);
  expect(global.fetch).toHaveBeenCalledWith('/plugins');
  expect(global.fetch).toHaveBeenCalledWith('/log', expect.any(Object));
  expect(global.fetch).toHaveBeenCalledWith('/plugins/run', expect.any(Object));
});
