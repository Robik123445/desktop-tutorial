import { render, screen, fireEvent } from '@testing-library/react';
import HeightmapProbe from './src/HeightmapProbe';

beforeEach(() => {
  global.fetch = jest.fn(() =>
    Promise.resolve({
      json: () =>
        Promise.resolve({
          points: [
            [0, 0, 1],
            [0, 1, 2],
            [1, 0, 3],
            [1, 1, 4],
          ],
        }),
    })
  );
});

test('starts probing and displays map', async () => {
  render(<HeightmapProbe />);
  fireEvent.click(screen.getByText('Start Probing'));
  expect(global.fetch).toHaveBeenCalledWith('/probe_heightmap', expect.any(Object));
});
