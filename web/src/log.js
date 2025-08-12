// Unified logger used by both browser components and Node tests.
// Stores log history in memory and localStorage when available and
// writes to log.txt when running under Node.

const STORAGE_KEY = 'centralLog';
const LEVELS = { DEBUG: 0, INFO: 1, WARNING: 2, ERROR: 3 };
let currentLevel = LEVELS.INFO;
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
      // ignore
    }
  }
}

export default function log(message, level = 'INFO', error) {
  const lvl = LEVELS[level.toUpperCase()] ?? LEVELS.INFO;
  if (lvl < currentLevel) return;

  let entry = `${new Date().toISOString()} ${level}: ${message}`;
  if (error) entry += ` - ${error.stack || error}`;

  // console output without timestamp for readability
  if (lvl >= LEVELS.ERROR) console.error(`[ERROR]: ${message}`);
  else if (lvl === LEVELS.WARNING) console.warn(`[WARNING]: ${message}`);
  else console.log(`[LOG]: ${message}`);

  entries.push(entry);
  if (entries.length > 1000) entries.shift();
  save();

  // Node file append
  if (typeof window === 'undefined') {
    const fs = require('fs');
    const path = require('path');
    const file = path.join(__dirname, '..', 'log.txt');
    try {
      fs.appendFileSync(file, `${entry}\n`);
    } catch (err) {
      console.error('Failed to write log.txt', err);
    }
  }
}

export function getLogContent() {
  return entries.join('\n');
}

export function setLogLevel(level) {
  const lvl = LEVELS[level.toUpperCase()];
  if (lvl !== undefined) currentLevel = lvl;
}
