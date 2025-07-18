import React, { useState, useEffect } from 'react';

/**
 * GCodeExporter converts a toolpath JSON to G-code and optionally sends it
 * to the selected controller. All actions are logged via `/log`.
 */
//------------------------------------------------------------------------------
// GCodeExporter component
//------------------------------------------------------------------------------
export default function GCodeExporter() {
  const [toolpath, setToolpath] = useState('');
  const [filename, setFilename] = useState('');
  const [controller, setController] = useState('GRBL');
  const [ports, setPorts] = useState(['/dev/ttyUSB0', '/dev/ttyACM0']);
  const [port, setPort] = useState('/dev/ttyUSB0');
  const [gcode, setGcode] = useState('');
  const [output, setOutput] = useState('');
  const [loading, setLoading] = useState(false);
  const [sending, setSending] = useState(false);

  /**
   * Fetch list of serial ports from the backend. When it fails we keep
   * the current list so the user can still select a default port.
   */
  const fetchPorts = async () => {
    try {
      const res = await fetch('/ports');
      const data = await res.json();
      if (Array.isArray(data) && data.length) {
        setPorts(data);
        setPort(data[0]);
      }
    } catch {
      // keep defaults on failure
    }
  };

  // Fetch ports once on mount so the dropdown shows the latest devices.
  useEffect(() => {
    fetchPorts();
  }, []);

  /**
   * Send a message to /log asynchronously.
   */
  const postLog = (message) =>
    fetch('/log', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ message })
    }).catch(() => {});

  /**
   * Convert the toolpath JSON to G-code by calling /export.
   */
  const generate = async () => {
    setLoading(true);
    postLog(`export ${controller}`);
    try {
      let pathData = [];
      try {
        pathData = JSON.parse(toolpath || '[]');
      } catch {
        setOutput('Invalid toolpath JSON');
        setLoading(false);
        return;
      }
      const resp = await fetch('/export', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ toolpath: pathData, controller })
      });
      const data = await resp.json();
      setGcode(data.gcode);
      setOutput(data.gcode);
    } catch (err) {
      setOutput('Export failed');
    } finally {
      setLoading(false);
    }
  };

  const send = async () => {
    setSending(true);
    postLog(`send ${port}`);
    try {
      const resp = await fetch('/send', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ gcode, port })
      });
      const data = await resp.json();
      setOutput(data.status || 'Sent');
    } catch {
      setOutput('Send failed');
    } finally {
      setSending(false);
    }
  };

  return (
    <div className="p-4 space-y-4">
      <textarea
        className="bg-gray-800 p-2 w-full rounded h-40"
        placeholder="Toolpath JSON"
        value={toolpath}
        onChange={(e) => setToolpath(e.target.value)}
      />
      <input
        type="text"
        className="bg-gray-800 p-2 w-full rounded"
        placeholder="File name"
        value={filename}
        onChange={(e) => setFilename(e.target.value)}
      />
      <select
        className="bg-gray-800 p-2 w-full rounded"
        value={controller}
        onChange={(e) => setController(e.target.value)}
      >
        <option>GRBL</option>
        <option>Smoothie</option>
        <option>LinuxCNC</option>
      </select>
      <button
        className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded w-full"
        onClick={generate}
        disabled={loading}
      >
        {loading ? 'Generating...' : 'Generate G-code'}
      </button>
      {gcode && (
        <>
          <div className="flex space-x-2">
            <select
              className="bg-gray-800 p-2 flex-1 rounded"
              value={port}
              onChange={(e) => setPort(e.target.value)}
            >
              {ports.map((p) => (
                <option key={p} value={p}>
                  {p}
                </option>
              ))}
            </select>
            <button
              className="bg-gray-700 hover:bg-gray-600 text-white px-4 rounded"
              onClick={() => {
                postLog('refresh ports');
                fetchPorts();
              }}
              type="button"
            >
              Refresh
            </button>
          </div>
          <button
            className="bg-green-600 hover:bg-green-700 text-white px-4 py-2 rounded w-full"
            onClick={send}
            disabled={sending}
          >
            {sending ? 'Sending...' : 'Send to Machine'}
          </button>
        </>
      )}
      {output && (
        <pre className="bg-gray-800 p-2 rounded text-sm whitespace-pre-wrap">
          {output}
        </pre>
      )}
    </div>
  );
}
