import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import FileUploader from './FileUploader';
jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

test('shows preview, logs actions and button after selecting file', () => {
  const file = new File(['dummy'], 'test.dxf', { type: 'application/dxf' });
  render(<FileUploader />);
  const input = screen.getByLabelText(/drag/i);
  fireEvent.change(input, { target: { files: [file] } });
  expect(log).toHaveBeenCalledWith('File selected via input: test.dxf');
  expect(screen.getByTestId('file-name')).toHaveTextContent('test.dxf');
  expect(screen.getByText('2D CAD Drawing')).toBeInTheDocument();
  expect(screen.getByText('Continue to Toolpath Editor')).toBeInTheDocument();
  fireEvent.click(screen.getByText('Continue to Toolpath Editor'));
  expect(log).toHaveBeenCalledWith('Continue to editor: test.dxf');
});

test('supports OBJ files as well', () => {
  const file = new File(['dummy'], 'mesh.obj', { type: 'model/obj' });
  render(<FileUploader />);
  const input = screen.getByLabelText(/drag/i);
  fireEvent.change(input, { target: { files: [file] } });
  expect(log).toHaveBeenCalledWith('File selected via input: mesh.obj');
  expect(screen.getByText('3D Model (OBJ)')).toBeInTheDocument();
});

test('handles image preview', () => {
  const file = new File(['dummy'], 'pic.png', { type: 'image/png' });
  render(<FileUploader />);
  const input = screen.getByLabelText(/drag/i);
  fireEvent.change(input, { target: { files: [file] } });
  expect(log).toHaveBeenCalledWith('File selected via input: pic.png');
  expect(screen.getByAltText('Image preview')).toBeInTheDocument();
});
