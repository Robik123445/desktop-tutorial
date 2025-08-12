import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import ToolList from './ToolList';
jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

beforeEach(() => {
  localStorage.clear();
});

test('adds and removes tools while logging', () => {
  render(<ToolList />);
  fireEvent.change(screen.getByTestId('name-input'), { target: { value: 'EndMill' } });
  fireEvent.change(screen.getByTestId('diameter-input'), { target: { value: '3' } });
  fireEvent.change(screen.getByTestId('rpm-input'), { target: { value: '20000' } });
  fireEvent.click(screen.getByTestId('add-btn'));

  const name = screen.getByTestId('name-0');
  expect(name).toHaveTextContent('EndMill');
  expect(log).toHaveBeenCalledWith('Tool added: EndMill');

  fireEvent.click(screen.getByTestId('remove-0'));
  expect(screen.queryByTestId('name-0')).toBeNull();
  expect(log).toHaveBeenCalledWith('Tool removed at index 0');
});
