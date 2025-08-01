const fs = require('fs').promises;
const path = require('path');

const logFile = path.join(__dirname, 'log.txt');

/**
 * Append a timestamped message to web/log.txt.
 */
async function log(message) {
  const line = `${new Date().toISOString()} ${message}\n`;
  try {
    await fs.appendFile(logFile, line);
  } catch (err) {
    console.error('Failed to write log.txt', err);
  }
}

module.exports = log;
