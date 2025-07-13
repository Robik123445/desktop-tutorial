import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import WorkflowShare from './WorkflowShare';
jest.mock('./logger', () => jest.fn());
import log from './logger';

const mockFile = new File([JSON.stringify({test:1})], 'wf.json', {type:'application/json'});

test('export and import workflow', () => {
  const onImport = jest.fn();
  render(<WorkflowShare workflow={{a:1}} onImport={onImport} />);
  fireEvent.click(screen.getByText('Share Workflow'));
  expect(log).toHaveBeenCalledWith('export workflow');
  const input = screen.getByTestId('wf-input');
  fireEvent.change(input, { target: { files: [mockFile] } });
  expect(log).toHaveBeenCalledWith('import workflow');
  expect(onImport).toHaveBeenCalled();
});

