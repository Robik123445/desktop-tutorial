import { render, screen, fireEvent } from '@testing-library/react';
import Sidebar from './src/Sidebar';

it('navigates when menu item clicked', () => {
  const cb = jest.fn();
  render(<Sidebar onNavigate={cb} />);
  fireEvent.click(screen.getByText('AI Analyzer'));
  expect(cb).toHaveBeenCalledWith('ai');
  const btn = screen.getByText('AI Analyzer').closest('button');
  expect(btn.className).toMatch(/bg-gray-800/);
});
