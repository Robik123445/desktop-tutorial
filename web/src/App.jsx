import React, { useState } from 'react';
import Dashboard from './Dashboard';
import SettingsPanel from './SettingsPanel';
import PluginPanel from './PluginPanel';
import AIAnalyzer from './AIAnalyzer';
import HeightmapEditor from './HeightmapEditor';
import HeightmapProbe from './HeightmapProbe';
import GCodeExporter from './GCodeExporter';
import ToolpathPreview from './ToolpathPreview';
import LogViewer from './LogViewer';

/**
 * Map of available panels displayed in the sidebar.
 */
const PANELS = {
  dashboard: { label: 'Dashboard', component: Dashboard },
  settings: { label: 'Settings', component: SettingsPanel },
  plugins: { label: 'Plugins', component: PluginPanel },
  ai: { label: 'AI Analyzer', component: AIAnalyzer },
  editor: { label: 'Heightmap Editor', component: HeightmapEditor },
  probe: { label: 'Heightmap Probe', component: HeightmapProbe },
  export: { label: 'G-code Export', component: GCodeExporter },
  preview: { label: 'Toolpath Preview', component: ToolpathPreview },
  logs: { label: 'Log Viewer', component: LogViewer },
};

/**
 * Main application component with simple sidebar navigation.
 */
export default function App() {
  const [panel, setPanel] = useState('dashboard');
  const Current = PANELS[panel].component;

  return (
    <div className="min-h-screen flex bg-gray-900 text-gray-100">
      <aside className="w-48 bg-gray-800 p-4 space-y-1">
        {Object.entries(PANELS).map(([key, { label }]) => (
          <button
            key={key}
            onClick={() => setPanel(key)}
            className={`block w-full text-left px-2 py-1 rounded ${
              key === panel ? 'bg-gray-700' : 'hover:bg-gray-700'
            }`}
          >
            {label}
          </button>
        ))}
      </aside>
      <main className="flex-1 p-4 overflow-auto">
        <Current />
      </main>
    </div>
  );
}
