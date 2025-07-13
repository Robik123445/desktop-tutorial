import React from 'react';
import { render, screen, fireEvent, act } from '@testing-library/react';
import WebcamDetector from './WebcamDetector';
jest.useFakeTimers();

beforeAll(() => {
  global.navigator.mediaDevices = {
    getUserMedia: jest.fn(() => Promise.resolve({
      getTracks: () => [{ stop: jest.fn() }],
    })),
  };
});

jest.mock('./logger', () => jest.fn());
import log from './logger';

test('shows warning banner when person detected', async () => {
  render(<WebcamDetector testDetection="hand" />);
  fireEvent.click(screen.getByText('Start Object Detection'));
  await act(async () => {
    jest.runAllTimers();
  });
  expect(screen.getByTestId('warning')).toHaveTextContent('HAND DETECTED');
  expect(log).toHaveBeenCalledWith('Start detection clicked');
  expect(log).toHaveBeenCalledWith('Webcam started');
  expect(log).toHaveBeenCalledWith('Warning: hand detected');
});
