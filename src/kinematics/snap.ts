/**
 * Snap-lock feasibility: whether two open connectors are close enough and
 * facing each other closely enough to preview/commit a weld.
 */
import type { Connector, Pose } from '../types/module';
import { SNAP_ANGLE_TOLERANCE, SNAP_DISTANCE_TOLERANCE } from '../constants/geometry';
import { outwardNormal, worldPosition } from './frame';

export interface SnapCandidate {
  connectorA: Connector;
  connectorB: Connector;
  distance: number;
  angleError: number;
}

/**
 * Two connectors "face each other" when their outward normals point in
 * opposite directions (dot product near -1) -- i.e. the domes are pointed at
 * one another, not side-by-side or back-to-back.
 */
export function canSnap(poseA: Pose, poseB: Pose): { ok: boolean; distance: number; angleError: number } {
  const distance = worldPosition(poseA).distanceTo(worldPosition(poseB));
  const normalA = outwardNormal(poseA);
  const normalB = outwardNormal(poseB);
  const dot = Math.max(-1, Math.min(1, normalA.dot(normalB)));
  const angleError = Math.PI - Math.acos(dot); // 0 when normals are perfectly opposed
  const ok = distance <= SNAP_DISTANCE_TOLERANCE && Math.abs(angleError) <= SNAP_ANGLE_TOLERANCE;
  return { ok, distance, angleError };
}

/** Finds every open (unlocked) connector pair across the assembly within snap tolerance. */
export function findSnapCandidates(
  openConnectors: Array<{ connector: Connector; pose: Pose }>,
): SnapCandidate[] {
  const candidates: SnapCandidate[] = [];
  for (let i = 0; i < openConnectors.length; i += 1) {
    for (let j = i + 1; j < openConnectors.length; j += 1) {
      const { connector: a, pose: poseA } = openConnectors[i];
      const { connector: b, pose: poseB } = openConnectors[j];
      if (a.moduleId === b.moduleId) continue; // a module can't snap to itself
      const { ok, distance, angleError } = canSnap(poseA, poseB);
      if (ok) candidates.push({ connectorA: a, connectorB: b, distance, angleError });
    }
  }
  return candidates.sort((c1, c2) => c1.distance - c2.distance);
}

export { SNAP_DISTANCE_TOLERANCE, SNAP_ANGLE_TOLERANCE };
