import React, { useState, useEffect } from 'react';
import log from './src/log.js';
import RobotDiagnosticsDashboard from './RobotDiagnosticsDashboard';
import BugReportForm from './BugReportForm';

// Default kinematic profile used when none stored in localStorage
const DEFAULT_PROFILE = {
  name: '6axis_basic',
  link_lengths: [100, 100],
  joint_limits: [
    [-180, 180],
    [-90, 90],
    [-135, 135],
    [-180, 180],
    [-120, 120],
    [-360, 360],
  ],
  tcp: [0, 0, 0, 0, 0, 0],
};

// simple FK for planar arms using revolute joints
function forwardKinematics(profile, joints) {
  let x = 0;
  let y = 0;
  let a = 0;
  profile.link_lengths.forEach((len, i) => {
    a += (joints[i] || 0) * Math.PI / 180;
    x += len * Math.cos(a);
    y += len * Math.sin(a);
  });
  return { x: x.toFixed(1), y: y.toFixed(1) };
}

export default function RobotArmPanel() {
  const storedEnabled = localStorage.getItem('robotArmEnabled') === 'true';
  const storedProfile = localStorage.getItem('robotArmProfile');
  const [enabled, setEnabled] = useState(storedEnabled);
  const [profiles, setProfiles] = useState([DEFAULT_PROFILE]);
  const [index, setIndex] = useState(0);
  const [angles, setAngles] = useState([]);
  const [dryRun, setDryRun] = useState(false);
  const [status, setStatus] = useState('Idle');
  const [warnings, setWarnings] = useState([]);
  const profile = profiles[index];

  useEffect(() => {
    if (storedProfile) {
      try {
        const p = JSON.parse(storedProfile);
        setProfiles([p]);
      } catch (err) {
        /* ignore */
      }
    }
  }, []);

  useEffect(() => {
    localStorage.setItem('robotArmEnabled', enabled);
    log(`robot arm mode -> ${enabled}`);
  }, [enabled]);

  useEffect(() => {
    localStorage.setItem('robotArmProfile', JSON.stringify(profile));
  }, [profile]);

  useEffect(() => {
    if (!enabled || !dryRun) return;
    setStatus('Running dry run');
    const id = setInterval(() => {
      const newAngles = profile.joint_limits.map(l =>
        l[0] + Math.random() * (l[1] - l[0])
      );
      setAngles(newAngles);
      // compute warnings
      const w = [];
      newAngles.forEach((a, i) => {
        const [lo, hi] = profile.joint_limits[i];
        if (a < lo || a > hi) w.push(`Joint ${i+1} limit`);
        else if (a - lo < 5 || hi - a < 5) w.push(`Joint ${i+1} near limit`);
      });
      if (Math.random() < 0.05) w.push('Overload detected');
      if (Math.random() < 0.05) w.push('Possible collision');
      setWarnings(w);
      if (w.length) log(`warnings: ${w.join(', ')}`, 'WARNING');
    }, 1000);
    return () => {
      clearInterval(id);
      setStatus('Idle');
      setWarnings([]);
    };
  }, [enabled, profile, dryRun]);

  const tcp = forwardKinematics(profile, angles);

  const handleImport = (e) => {
    const file = e.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const obj = JSON.parse(reader.result);
        setProfiles([obj]);
        setIndex(0);
        log(`profile imported: ${obj.name}`);
      } catch (err) {
        log('profile import failed', 'ERROR', err);
      }
    };
    reader.readAsText(file);
  };

  const handleExport = () => {
    const blob = new Blob([JSON.stringify(profile, null, 2)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `${profile.name}.json`;
    a.click();
    log('profile exported');
  };

  const saveProfile = () => {
    localStorage.setItem('robotArmProfile', JSON.stringify(profile));
    log('profile saved');
  };

  const [port, setPort] = useState('COM1');

  const streamCommands = async () => {
    try {
      await fetch('/stream_robotic', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ points: [[0,0,0],[10,0,0]], port, profile }),
      });
      log('robotic stream started');
    } catch (err) {
      log('robotic stream failed', 'ERROR', err);
    }
  };

  const exportCommands = () => {
    const blob = new Blob([
      JSON.stringify({ toolpath: [[0,0,0],[10,0,0]], profile }, null, 2),
    ], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'robot_commands.json';
    a.click();
    log('robotic commands exported');
  };

  const setLinkLength = (idx, val) => {
    setProfiles(pfs => {
      const p = { ...pfs[index] };
      p.link_lengths = p.link_lengths.map((v,i)=> i===idx ? val : v);
      const arr = [...pfs];
      arr[index] = p;
      return arr;
    });
  };

  const setLimit = (jIdx, boundIdx, val) => {
    setProfiles(pfs => {
      const p = { ...pfs[index] };
      const limits = p.joint_limits.map(l => [...l]);
      limits[jIdx][boundIdx] = val;
      p.joint_limits = limits;
      const arr = [...pfs];
      arr[index] = p;
      return arr;
    });
  };

  const setTcp = (idx, val) => {
    setProfiles(pfs => {
      const p = { ...pfs[index], tcp: pfs[index].tcp.map((v,i)=> i===idx ? val : v) };
      const arr = [...pfs];
      arr[index] = p;
      return arr;
    });
  };

  return (
    <div className="p-4 space-y-4 text-sm">
      <label className="block">
        <input
          type="checkbox"
          className="mr-1"
          checked={enabled}
          onChange={(e) => setEnabled(e.target.checked)}
          data-testid="robot-toggle"
        />
        Enable robot arm mode
      </label>

      <label className="block text-sm">
        <input
          type="checkbox"
          className="mr-1"
          checked={dryRun}
          onChange={(e) => setDryRun(e.target.checked)}
          disabled={!enabled}
          data-testid="dry-toggle"
        />
        Dry run (no cutting)
      </label>

      <div className="text-sm">Status: {status}</div>

      <div>
        <label className="block font-medium">Profile</label>
        <select
          value={index}
          onChange={(e) => setIndex(Number(e.target.value))}
          className="border rounded p-1 w-full"
          data-testid="profile-select"
        >
          {profiles.map((p, i) => (
            <option key={i} value={i}>{p.name}</option>
          ))}
        </select>
        <div className="flex space-x-2 mt-1">
          <input type="file" accept="application/json" onChange={handleImport} data-testid="import-file" />
          <button onClick={handleExport} className="bg-blue-600 px-2 py-1 rounded" data-testid="export-btn">Export</button>
          <button onClick={saveProfile} className="bg-green-600 px-2 py-1 rounded" data-testid="save-btn">Save</button>
        </div>
      </div>

      <div className="space-y-1">
        <h4 className="font-medium">Command Streaming</h4>
        <input
          type="text"
          value={port}
          onChange={(e) => setPort(e.target.value)}
          className="border rounded p-1 text-black w-32"
          placeholder="Port"
          data-testid="port-input"
        />
        <div className="flex space-x-2 mt-1">
          <button onClick={streamCommands} className="bg-blue-700 px-2 py-1 rounded text-white" data-testid="stream-btn">Stream</button>
          <button onClick={exportCommands} className="bg-gray-700 px-2 py-1 rounded text-white" data-testid="cmd-export">Export Cmds</button>
        </div>
      </div>

      <div>
        <h4 className="font-medium">Edit Links</h4>
        <div className="grid grid-cols-3 gap-1">
          {profile.link_lengths.map((v, idx) => (
            <input
              key={idx}
              type="number"
              value={v}
              onChange={(e) => setLinkLength(idx, Number(e.target.value))}
              className="border rounded p-1 text-black"
              data-testid={`len-${idx}`}
            />
          ))}
        </div>
      </div>

      <div>
        <h4 className="font-medium">Edit Limits</h4>
        {profile.joint_limits.map((limit, i) => (
          <div key={i} className="flex space-x-1 items-center">
            <span>J{i+1}</span>
            <input
              type="number"
              value={limit[0]}
              onChange={(e) => setLimit(i, 0, Number(e.target.value))}
              className="border rounded p-1 w-16 text-black"
              data-testid={`lim-${i}-0`}
            />
            <input
              type="number"
              value={limit[1]}
              onChange={(e) => setLimit(i, 1, Number(e.target.value))}
              className="border rounded p-1 w-16 text-black"
              data-testid={`lim-${i}-1`}
            />
          </div>
        ))}
      </div>

      <div>
        <h4 className="font-medium">TCP Position</h4>
        <p>X: {tcp.x} mm, Y: {tcp.y} mm</p>
      </div>

      <div>
        <h4 className="font-medium">Diagnostics</h4>
        {profile.joint_limits.map((limit, i) => {
          const angle = angles[i] || 0;
          const warn = angle < limit[0] || angle > limit[1];
          return (
            <div key={i} className={warn ? 'text-red-500' : ''} data-testid={`joint-${i}`}>J{i+1}: {angle.toFixed(1)}Â° {warn ? 'LIMIT' : 'OK'}</div>
          );
        })}
        {warnings.map((w, i) => (
          <div key={i} className="text-red-600" data-testid={`warn-${i}`}>{w}</div>
        ))}
      </div>

      <div>
        <h4 className="font-medium">Edit TCP</h4>
        <div className="grid grid-cols-3 gap-1">
          {profile.tcp.map((v, idx) => (
            <input
              key={idx}
              type="number"
              value={v}
              onChange={(e) => setTcp(idx, Number(e.target.value))}
              className="border rounded p-1 text-black"
              data-testid={`tcp-${idx}`}
            />
          ))}
        </div>
      </div>

      <div>
        <h4 className="font-medium">Diagnostics</h4>
        <RobotDiagnosticsDashboard />
      </div>

      <div>
        <h4 className="font-medium">Bug Report</h4>
        <BugReportForm />
      </div>
    </div>
  );
}

