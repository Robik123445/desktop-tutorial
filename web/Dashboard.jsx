import React from 'react';
import log from './logger';

/**
 * Dashboard component showing recent projects and new project button.
 * Layout has sidebar menu on the left and main content area on the right.
 */
export default function Dashboard({ projects = [] }) {
  const recent = projects.slice(0, 3);

  const handleNewProject = () => {
    log('new project click');
    console.log('New project');
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
            {recent.map((p, idx) => (
              <li key={idx} className="py-2 flex justify-between">
                <span className="font-medium">{p.name}</span>
                <span className="text-gray-500 text-sm">{p.date}</span>
                <span className="text-gray-400 text-sm">{p.type}</span>
              </li>
            ))}
          </ul>
        </section>
      </main>
    </div>
  );
}
