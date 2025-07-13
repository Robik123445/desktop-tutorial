import { render, screen, fireEvent } from '@testing-library/react';
import RobotTrajectoryOptimizer from './RobotTrajectoryOptimizer';

global.fetch = jest.fn(() => Promise.resolve({ json: () => Promise.resolve({ points: [[0,0,0]], warnings: ['warn'] }) }));

test('run optimization shows suggestions', async () => {
  render(<RobotTrajectoryOptimizer toolpath={[[0,0,0],[1,0,0]]} profile={{}} />);
  fireEvent.click(screen.getByText('Optimize Trajectory'));
  await screen.findByText('warn');
});
