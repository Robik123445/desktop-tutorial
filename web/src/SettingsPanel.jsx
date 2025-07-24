import React, { useState, useEffect } from 'react';

/**
 * SettingsPanel allows the user to configure basic dashboard settings.
 * The values are persisted to localStorage so they survive page reloads.
 */
export default function SettingsPanel() {
  const [theme, setTheme] = useState('Light');
  const [apiUrl, setApiUrl] = useState('');
  const [saved, setSaved] = useState(false);

  // Load settings from localStorage on first render
  useEffect(() => {
    const storedTheme = localStorage.getItem('theme');
    const storedUrl = localStorage.getItem('apiUrl');
    if (storedTheme) setTheme(storedTheme);
    if (storedUrl) setApiUrl(storedUrl);
  }, []);

  /**
   * Persist current settings and show a short confirmation message.
   */
  const handleSave = () => {
    localStorage.setItem('theme', theme);
    localStorage.setItem('apiUrl', apiUrl);
    setSaved(true);
    // log action but ignore network errors
    fetch('/log', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ message: 'settings saved' })
    }).catch(() => {});
    setTimeout(() => setSaved(false), 2000);
  };

  return (
    <div className="bg-gray-900 text-gray-100 p-4 rounded space-y-3">
      <h2 className="text-lg font-semibold">Settings</h2>
      <div>
        <label className="block text-sm mb-1">Theme</label>
        <select
          className="bg-gray-800 p-2 w-full rounded"
          value={theme}
          onChange={(e) => setTheme(e.target.value)}
        >
          <option>Light</option>
          <option>Dark</option>
        </select>
      </div>
      <div>
        <label className="block text-sm mb-1" htmlFor="api-url">API URL</label>
        <input
          id="api-url"
          type="text"
          placeholder="http://localhost:8000"
          className="bg-gray-800 p-2 w-full rounded"
          value={apiUrl}
          onChange={(e) => setApiUrl(e.target.value)}
        />
      </div>
      <button
        className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded w-full"
        onClick={handleSave}
      >
        Save
      </button>
      {saved && <div className="text-green-400 text-sm">Settings saved</div>}
    </div>
  );
}
