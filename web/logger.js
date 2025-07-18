const fs = require('fs').promises;
const path = require('path');

const logPath = path.join(__dirname, '..', 'logs', 'central.log');
const logDir = path.dirname(logPath);
const MAX_SIZE = 1024 * 1024; // 1 MB

const LEVELS = { DEBUG: 0, INFO: 1, WARNING: 2, ERROR: 3 };
let currentLevel = LEVELS[(process.env.LOG_LEVEL || 'INFO').toUpperCase()] || LEVELS.INFO;

/**
 * Append a log entry to the shared central log file.
 */
async function log(message, level = 'INFO', error) {
  const lvl = LEVELS[level.toUpperCase()] ?? LEVELS.INFO;
  if (lvl < currentLevel) return;

  let entry = `${new Date().toISOString()} ${level}: ${message}`;
  if (error) {
    entry += ` - ${error.stack || error}`;
  }
  entry += '\n';
  try {
    await fs.mkdir(logDir, { recursive: true });
    try {
      const stat = await fs.stat(logPath);
      if (stat.size > MAX_SIZE) {
        await fs.rename(logPath, `${logPath}.1`);
      }
    } catch (err) {
      if (err.code !== 'ENOENT') throw err;
    }
    await fs.appendFile(logPath, entry);
  } catch (err) {
    console.error('Failed to write log:', err);
  }
}

/**
 * Read log file content for export.
 */
function getLogContent() {
  try {
    fs.mkdirSync(logDir, { recursive: true });
    return fs.existsSync(logPath) ? fs.readFileSync(logPath, 'utf8') : '';
  } catch (err) {
    console.error('Failed to read log:', err);
    return '';
  }
}

module.exports = log;
module.exports.getLogContent = getLogContent;
module.exports.logPath = logPath;
