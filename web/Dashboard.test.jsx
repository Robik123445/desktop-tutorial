import React from 'react';
import { render, screen } from '@testing-library/react';
import Dashboard from './Dashboard';

test('renders heading, button and recent projects', () => {
  const projects = [
    { name: 'a', date: '1', type: 'SVG' },
    { name: 'b', date: '2', type: 'DXF' },
    { name: 'c', date: '3', type: 'GCODE' },
    { name: 'd', date: '4', type: 'STL' },
  ];
  render(<Dashboard projects={projects} />);
  expect(screen.getByText('CAM Slicer')).toBeInTheDocument();
  expect(screen.getByText('New Project')).toBeInTheDocument();
  expect(screen.getAllByRole('listitem')).toHaveLength(3);
});
