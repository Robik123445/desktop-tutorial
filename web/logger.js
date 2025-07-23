// Simple browser-friendly logger with in-memory history.
// Logs are stored in localStorage under the key 'centralLog'.
// Provides default export `log` and helper `getLogContent`.

const STORAGE_KEY = 'centralLog';
const LEVELS = { DEBUG: 0, INFO: 1, WARNING: 2, ERROR: 3 };

// Determine initial level from environment variable if provided.
let currentLevel = LEVELS[(typeof import.meta !== 'undefined' && import.meta.env && import.meta.env.VITE_LOG_LEVEL || 'INFO').toUpperCase()] ?? LEVELS.INFO;

// Load previous log entries from localStorage when available.
let entries = [];
if (typeof window !== 'undefined' && window.localStorage) {
  try {
    const saved = window.localStorage.getItem(STORAGE_KEY);
    entries = saved ? JSON.parse(saved) : [];
  } catch {
    entries = [];
  }
}

function save() {
  if (typeof window !== 'undefined' && window.localStorage) {
    try {
      window.localStorage.setItem(STORAGE_KEY, JSON.stringify(entries));
    } catch {
      // ignore write failures
    }
  }
}

/**
 * Log a message with optional severity and error object.
 * Severity levels: DEBUG, INFO, WARNING, ERROR.
 */
export default function log(message, level = 'INFO', error) {
  const lvl = LEVELS[level.toUpperCase()] ?? LEVELS.INFO;
  if (lvl < currentLevel) return;

  let entry = `${new Date().toISOString()} ${level}: ${message}`;
  if (error) entry += ` - ${error.stack || error}`;

  // Output to console
  if (lvl >= LEVELS.ERROR) console.error(entry);
  else if (lvl === LEVELS.WARNING) console.warn(entry);
  else console.log(entry);

  entries.push(entry);
  if (entries.length > 1000) entries.shift();
  save();
}

/**
 * Return the collected log entries as a newline separated string.
 */
export function getLogContent() {
  return entries.join('\n');
}

/**
 * Allow tests or callers to change the current log level.
 */
export function setLogLevel(level) {
  const lvl = LEVELS[level.toUpperCase()];
  if (lvl !== undefined) currentLevel = lvl;
}
