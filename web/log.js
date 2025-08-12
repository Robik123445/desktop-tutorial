import { promises as fs } from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const logFile = path.join(__dirname, 'log.txt');

/**
 * Append a timestamped message to web/log.txt.
 */
export default async function log(message) {
  const line = `${new Date().toISOString()} ${message}\n`;
  try {
    await fs.appendFile(logFile, line);
  } catch (err) {
    console.error('Failed to write log.txt', err);
  }
}
