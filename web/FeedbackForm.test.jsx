import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import FeedbackForm from './FeedbackForm';
jest.mock('./logger', () => jest.fn());
import log from './logger';

test('logs feedback on submit', () => {
  render(<FeedbackForm />);
  fireEvent.change(screen.getByTestId('feedback-input'), { target: { value: 'hi' } });
  fireEvent.click(screen.getByTestId('feedback-send'));
  expect(log).toHaveBeenCalledWith('feedback: hi');
});
