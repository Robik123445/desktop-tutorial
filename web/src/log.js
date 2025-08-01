/**
 * Simple logging utility for browser and Node environments.
 *
 * Logs message to the console with a standard prefix. If executed
 * under Node.js, also appends the message to `web/log.txt` for
 * persistent logging.
 *
 * @param {string} msg - Message to log.
 */
export default function log(msg) {
  console.log(`[LOG]: ${msg}`);
  if (typeof window === 'undefined') {
    // Node.js environment - append to log file
    const fs = require('fs');
    const path = require('path');
    const file = path.join(__dirname, '..', 'log.txt');
    try {
      fs.appendFileSync(file, `[${new Date().toISOString()}] ${msg}\n`);
    } catch (err) {
      console.error('Failed to write log.txt', err);
    }
  }
}
