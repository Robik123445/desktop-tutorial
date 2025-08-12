import fs from 'fs';
import path from 'path';
import log from './log.js';

test('writes to log.txt asynchronously', async () => {
  const file = path.join(__dirname, 'log.txt');
  if (fs.existsSync(file)) fs.unlinkSync(file);
  await log('test entry');
  const content = fs.readFileSync(file, 'utf8');
  expect(content).toMatch(/test entry/);
});
