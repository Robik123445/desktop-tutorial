import * as THREE from 'three';
import { ModelData, ToolpathData } from './viewportUtils';

export interface AIDiagnostic {
  position: THREE.Vector3;
  message: string;
  severity: 'info' | 'warning' | 'error';
}

/**
 * Analyze models and toolpaths to find common issues.
 * For demo purposes we check workspace bounds and Z collisions.
 */
export function analyzeScene(models: ModelData[], toolpaths: ToolpathData[]): AIDiagnostic[] {
  const diags: AIDiagnostic[] = [];
  const limit = 200; // workspace is 400x400 mm

  models.forEach((m) => {
    const box = m.bbox.clone().translate(m.object.position);
    if (box.min.x < -limit || box.max.x > limit || box.min.y < -limit || box.max.y > limit) {
      diags.push({
        position: box.getCenter(new THREE.Vector3()),
        message: `Model ${m.name} outside workspace`,
        severity: 'warning',
      });
    }
  });

  toolpaths.forEach((tp) => {
    const box = tp.bbox;
    if (box.min.x < -limit || box.max.x > limit || box.min.y < -limit || box.max.y > limit) {
      diags.push({
        position: box.getCenter(new THREE.Vector3()),
        message: `${tp.name} out of bounds`,
        severity: 'error',
      });
    }
    tp.points.forEach((p) => {
      if (p.z < -5) {
        diags.push({ position: p, message: 'Possible collision', severity: 'error' });
      }
    });
  });

  return diags;
}

export interface LoadPoint {
  position: THREE.Vector3;
  load: number; // 0..1 predicted spindle load
}

/** Predict simple load for each toolpath point. */
export function predictLoadPoints(toolpaths: ToolpathData[]): LoadPoint[] {
  const loads: LoadPoint[] = [];
  toolpaths.forEach(tp => {
    const pts = tp.points;
    for (let i = 1; i < pts.length; i++) {
      const prev = pts[i - 1];
      const cur = pts[i];
      const dist = cur.distanceTo(prev);
      const load = Math.min(1, dist / 10);
      loads.push({ position: cur.clone(), load });
    }
  });
  return loads;
}
