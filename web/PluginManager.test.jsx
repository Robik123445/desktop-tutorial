import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import PluginManager from './PluginManager';
jest.mock('./logger', () => jest.fn());
import log from './logger';

test('toggle plugins and trigger AI actions', () => {
  render(<PluginManager plugins={[{ name: 'reverse_path' }, { name: 'adaptive_path' }]} />);
  fireEvent.click(screen.getByTestId('plug-0'));
  expect(log).toHaveBeenCalledWith('plugin reverse_path -> off');
  fireEvent.click(screen.getByTestId('ai-analyze'));
  fireEvent.click(screen.getByTestId('ai-opt'));
  expect(log).toHaveBeenCalledWith('AI analysis requested');
  expect(log).toHaveBeenCalledWith('AI auto optimize');

  fireEvent.change(screen.getByPlaceholderText('plugin name'), { target: { value: 'newp' } });
  fireEvent.click(screen.getByText('Install'));
  expect(log).toHaveBeenCalledWith('install plugin newp');
  fireEvent.click(screen.getAllByTitle('Remove')[0]);
  expect(log).toHaveBeenCalledWith('remove plugin reverse_path');
});
