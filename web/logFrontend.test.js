import fs from 'fs';
import path from 'path';
import log from './src/log.js';

test('logs to console and file', () => {
  const spy = jest.spyOn(console, 'log').mockImplementation(() => {});
  const file = path.join(__dirname, 'log.txt');
  if (fs.existsSync(file)) fs.unlinkSync(file);
  log('hello');
  expect(spy).toHaveBeenCalledWith('[LOG]: hello');
  const data = fs.readFileSync(file, 'utf8');
  expect(data).toMatch(/hello/);
  spy.mockRestore();
});
