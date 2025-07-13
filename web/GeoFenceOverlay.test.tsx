import { render } from '@testing-library/react';
import GeoFenceOverlay from './GeoFenceOverlay';

test('renders forbidden zone box', () => {
  const { container } = render(<GeoFenceOverlay forbidden={[[0,0,10,10]]} />);
  expect(container.querySelector('canvas')).toBeInTheDocument;
});
