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
