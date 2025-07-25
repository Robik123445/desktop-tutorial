import React from 'react';
import { render, screen } from '@testing-library/react';
import Dashboard from './src/Dashboard';

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

test('uses mockProjects when none provided', () => {
  render(<Dashboard />);
  expect(screen.getByText('Demo Job')).toBeInTheDocument();
});

test('shows message when list empty', () => {
  render(<Dashboard projects={[]} />);
  expect(screen.getByText('Žiadne nedávne projekty.')).toBeInTheDocument();
});
