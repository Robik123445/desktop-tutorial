import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import VRViewer from './VRViewer';
jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

jest.mock('three/examples/jsm/webxr/VRButton.js', () => ({
  VRButton: { createButton: () => document.createElement('div') }
}));
jest.mock('three/examples/jsm/webxr/ARButton.js', () => ({
  ARButton: { createButton: () => document.createElement('div') }
}));

// Simple test ensuring the component renders and logs

test('starts and stops VR simulation', () => {
  const { getByText, getByTestId } = render(<VRViewer points={[{x:0,y:0,z:0},{x:1,y:0,z:0}]} mode="vr" />);
  expect(getByTestId('vr-viewer')).toBeInTheDocument();
  fireEvent.click(getByText('Play'));
  expect(log).toHaveBeenCalledWith('VR simulation start');
  fireEvent.click(getByText('Stop'));
  expect(log).toHaveBeenCalledWith('VR simulation stop');
});
