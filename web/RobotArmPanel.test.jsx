import React from 'react';
import { render, screen, fireEvent, act } from '@testing-library/react';
import RobotArmPanel from './RobotArmPanel';
jest.mock('./logger', () => jest.fn());
import log from './logger';

beforeEach(() => {
  jest.useFakeTimers();
  localStorage.clear();
});

afterEach(() => {
  jest.useRealTimers();
});

test('toggle robot mode and update profile', () => {
  render(<RobotArmPanel />);
  const toggle = screen.getByTestId('robot-toggle');
  fireEvent.click(toggle);
  expect(log).toHaveBeenCalledWith('robot arm mode -> true');
  expect(localStorage.getItem('robotArmEnabled')).toBe('true');

  const select = screen.getByTestId('profile-select');
  expect(select.value).toBe('0');
  fireEvent.change(select, { target: { value: '0' } });
});

test('diagnostics show warnings', () => {
  render(<RobotArmPanel />);
  fireEvent.click(screen.getByTestId('robot-toggle'));
  act(() => {
    jest.advanceTimersByTime(1100);
  });
  const j0 = screen.getByTestId('joint-0');
  expect(j0.textContent).toMatch(/J1:/);
});

test('dry run updates status and logs', () => {
  render(<RobotArmPanel />);
  fireEvent.click(screen.getByTestId('robot-toggle'));
  const dry = screen.getByTestId('dry-toggle');
  fireEvent.click(dry);
  act(() => {
    jest.advanceTimersByTime(1100);
  });
  expect(screen.getByText(/Status:/).textContent).toMatch(/Running/);
  expect(log).toHaveBeenCalledWith(expect.stringContaining('warnings'), 'WARNING');
});

test('save profile stores data', () => {
  render(<RobotArmPanel />);
  fireEvent.change(screen.getByTestId('len-0'), { target: { value: '150' } });
  fireEvent.click(screen.getByTestId('save-btn'));
  expect(localStorage.getItem('robotArmProfile')).toMatch(/150/);
});

test('stream button logs action', async () => {
  render(<RobotArmPanel />);
  fireEvent.click(screen.getByTestId('stream-btn'));
  expect(log).toHaveBeenCalledWith('robotic stream started');
});

test('bug report form logs message', () => {
  render(<RobotArmPanel />);
  fireEvent.change(screen.getByTestId('bug-input'), { target: { value: 'oops' } });
  fireEvent.click(screen.getByTestId('bug-send'));
  expect(log).toHaveBeenCalledWith('bug report: oops');
});
