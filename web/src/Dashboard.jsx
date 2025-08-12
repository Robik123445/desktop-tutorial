import React from 'react';
import log from './log.js';

// fallback data used in tests or when no projects are provided
const mockProjects = [
  { name: 'Demo Job', date: '2025-01-01', type: 'GCODE' },
  { name: 'Sample Engrave', date: '2025-01-02', type: 'SVG' },
];

/**
 * Dashboard component showing sidebar and recent projects.
 * Logs when the user starts a new project.
 */
export default function Dashboard({ projects }) {
  // ensure we always work with an array
  const list = Array.isArray(projects) ? projects.filter(p => p) : [];
  const data = list.length ? list : mockProjects;
  const recent = data.slice(0, 3);

  const handleNewProject = () => {
    log('New Project clicked');
  };

  return (
    <div className="min-h-screen flex">
      <aside className="w-60 bg-gray-800 text-white p-4">
        <h2 className="text-xl font-semibold mb-4">Menu</h2>
        <ul className="space-y-2">
          <li className="hover:text-gray-300 cursor-pointer">Projects</li>
          <li className="hover:text-gray-300 cursor-pointer">Settings</li>
          <li className="hover:text-gray-300 cursor-pointer">About</li>
        </ul>
      </aside>
      <main className="flex-1 p-8">
        <div className="flex items-center justify-between mb-6">
          <h1 className="text-3xl font-bold">CAM Slicer</h1>
          <button
            className="bg-blue-600 text-white px-4 py-2 rounded hover:bg-blue-700"
            onClick={handleNewProject}
          >
            New Project
          </button>
        </div>
        <section>
          <h2 className="text-xl font-semibold mb-2">Recent Projects</h2>
          <ul className="divide-y divide-gray-200">
            {recent.length === 0 ? (
              <li className="py-2">Žiadne nedávne projekty.</li>
            ) : (
              recent.map((p, idx) => (
                <li key={idx} className="py-2 flex justify-between">
                  <span className="font-medium">{p?.name || 'Unnamed'}</span>
                  <span className="text-gray-500 text-sm">{p?.date || '-'}</span>
                  <span className="text-gray-400 text-sm">{p?.type || '-'}</span>
                </li>
              ))
            )}
          </ul>
        </section>
      </main>
    </div>
  );
}
