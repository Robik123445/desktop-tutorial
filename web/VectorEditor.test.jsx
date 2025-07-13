import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import VectorEditor from './VectorEditor';
jest.mock('./logger', () => jest.fn());

beforeAll(() => {
  global.cv = {
    imread: jest.fn(() => ({ delete: jest.fn() })),
    cvtColor: jest.fn(),
    threshold: jest.fn(),
    Canny: jest.fn(),
    findContours: jest.fn(),
    MatVector: function () {
      return {
        vec: [
          {
            data32S: Int32Array.from([0, 0, 10, 0, 10, 10, 0, 10]),
            delete: jest.fn(),
          },
        ],
        size() {
          return this.vec.length;
        },
        get(i) {
          return this.vec[i];
        },
        delete: jest.fn(),
      };
    },
    Mat: function () {
      return { delete: jest.fn() };
    },
    COLOR_RGBA2GRAY: 0,
    THRESH_BINARY: 0,
    RETR_EXTERNAL: 0,
    CHAIN_APPROX_SIMPLE: 0,
    RETR_LIST: 0,
    CHAIN_APPROX_NONE: 0,
  };
});

// test advanced features: duplicate and undo/redo
test('duplicate path and undo works', () => {
  const file = new File(['dummy'], 'img.png', { type: 'image/png' });
  render(<VectorEditor />);
  const input = screen.getByLabelText(/drag/i);
  fireEvent.change(input, { target: { files: [file] } });
  fireEvent.click(screen.getByText('Vectorize'));
  expect(screen.getByText(/Paths:/)).toHaveTextContent('Paths: 1');
  fireEvent.click(screen.getByText('Duplicate'));
  expect(screen.getByText(/Paths:/)).toHaveTextContent('Paths: 2');
  fireEvent.click(screen.getByText('Undo'));
  expect(screen.getByText(/Paths:/)).toHaveTextContent('Paths: 1');
});

