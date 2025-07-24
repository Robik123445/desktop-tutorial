import React, { useState } from 'react';
import Dashboard from '../Dashboard';
import SettingsPanel from './SettingsPanel';
import PluginPanel from './PluginPanel';
import AIAnalyzer from './AIAnalyzer';
import HeightmapEditor from './HeightmapEditor';
import HeightmapProbe from './HeightmapProbe';
import GCodeExporter from './GCodeExporter';
import ToolpathPreview from './ToolpathPreview';
import LogViewer from './LogViewer';

/**
 * Map of all available panels. Each key defines a sidebar button
 * and the component rendered in the main area.
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
  logs: { label: 'Log Viewer', component: LogViewer }
};

/**
 * Root component with sidebar navigation. `activePanel` controls which
 * view is displayed. Uses Tailwind for a clean layout.
 */
export default function App() {
  const [activePanel, setActivePanel] = useState('dashboard');
  const ActiveComponent = PANELS[activePanel].component;

  return (
    <div className="flex flex-col md:flex-row min-h-screen">
      {/* Sidebar navigation */}
      <aside className="md:w-60 bg-gray-900 text-gray-100 p-4">
        <nav className="space-y-1">
          {Object.entries(PANELS).map(([key, { label }]) => (
            <button
              key={key}
              onClick={() => setActivePanel(key)}
              className={`block w-full px-3 py-2 text-left rounded ${
                activePanel === key ? 'bg-gray-800' : 'hover:bg-gray-700'
              }`}
            >
              {label}
            </button>
          ))}
        </nav>
      </aside>

      {/* Main content area */}
      <main className="flex-1 p-4 overflow-y-auto">
        <ActiveComponent />
      </main>
    </div>
  );
}
