import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import CuttingStrategySelector from './CuttingStrategySelector';
jest.mock('./logger', () => jest.fn());
import log from './logger';

test('selects strategy and logs choice', () => {
  render(<CuttingStrategySelector />);
  const radio = screen.getByTestId('radio-conventional');
  fireEvent.click(radio);
  expect(radio).toBeChecked();
  expect(log).toHaveBeenCalledWith('Cutting strategy: conventional');
});
