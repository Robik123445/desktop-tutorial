import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import ProjectManager from './ProjectManager';
jest.mock('./logger', () => jest.fn());
import log from './logger';
jest.mock('file-saver', () => ({ saveAs: jest.fn() }));
import { saveAs } from 'file-saver';

global.FileReader = class {
  readAsText() { this.result = '{"toolpaths": [1]}'; this.onload(); }
};

test('saves and loads project data', () => {
  const onLoad = jest.fn();
  render(
    <ProjectManager
      toolpaths={[{ x: 0 }]} material="MDF" machineProfile={{}}
      settings={{ feed: 100 }} onLoad={onLoad}
    />
  );
  fireEvent.click(screen.getByText('Save Project'));
  expect(saveAs).toHaveBeenCalled();
  fireEvent.change(screen.getByTestId('load-input'), { target: { files: [new File([''], 'proj.json', { type: 'application/json' })] } });
  expect(log).toHaveBeenCalledWith('Project loaded: proj.json');
  expect(onLoad).toHaveBeenCalledWith({ toolpaths: [1] });
});
