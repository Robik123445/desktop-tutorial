import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import FeedbackForm from './FeedbackForm';
jest.mock('./src/log.js', () => jest.fn());
import log from './src/log.js';

test('logs feedback on submit', () => {
  render(<FeedbackForm />);
  fireEvent.change(screen.getByTestId('feedback-input'), { target: { value: 'hi' } });
  fireEvent.click(screen.getByTestId('feedback-send'));
  expect(log).toHaveBeenCalledWith('feedback: hi');
});
