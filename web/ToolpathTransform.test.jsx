import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import ToolpathTransform from './ToolpathTransform';

// basic test verifying reset behaviour

test('reset button clears all transforms', () => {
  render(<ToolpathTransform />);
  const rot = screen.getByLabelText(/rotation/i);
  fireEvent.change(rot, { target: { value: '45' } });
  fireEvent.click(screen.getByText(/reset transform/i));
  expect(rot.value).toBe('0');
});
