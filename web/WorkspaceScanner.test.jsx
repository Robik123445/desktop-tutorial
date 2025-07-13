import { render, fireEvent, screen } from '@testing-library/react';
import WorkspaceScanner from './WorkspaceScanner';
import log from './logger';

jest.mock('./logger');

test('starts scan with selected mode', () => {
  render(<WorkspaceScanner onScan={() => {}} />);
  fireEvent.change(screen.getByRole('combobox'), { target: { value: 'camera' } });
  fireEvent.click(screen.getByRole('button'));
  expect(log).toHaveBeenCalledWith('workspace scan start: camera');
});
