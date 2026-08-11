/**
 * Picks the connector a branching chain should weld onto.
 *
 * The cube builder decomposes a voxel structure into maximal paths, which
 * means a cube with 3+ neighbours becomes a point where several chains meet.
 * Only two connectors can ever weld at one point (same position, opposed
 * normals), so before modules grew side connectors the extra arms at a branch
 * had nowhere to attach and were simply left floating with a warning.
 *
 * With 4 more connectors riding each module's big rod, an arm no longer has to
 * attach exactly at the branch cube's centre -- it can grab whichever nearby
 * connector on the already-placed chains actually faces the way the arm wants
 * to go, and be fitted outward from there. This module answers only "which
 * connector should that be"; the fitting itself stays in the caller.
 */
import { Vector3 } from 'three';
import type { Assembly, ConnectorEnd, ModuleId, Pose } from '../types/module';
import { allConnectors } from '../types/module';
import { computeAssemblyWorldTransforms, connectorPose } from './assemblyGraph';
import { outwardNormal, worldPosition } from './frame';

/**
 * How closely a candidate connector's outward normal must line up with the
 * direction the arm runs, as a dot product. An arm welded onto a connector
 * pointing sideways (or backwards) would have to double back on itself
 * immediately, which fits far worse than simply leaving the arm free-standing.
 * 0.5 is a 60-degree cone -- loose enough that a side connector on a trunk
 * running at an angle still qualifies, tight enough to exclude the ones facing
 * away.
 */
export const MIN_ANCHOR_ALIGNMENT = 0.5;

export interface AnchorCandidateKey {
  /** Identifies which already-fitted chain the connector belongs to. */
  chainKey: string;
  /** Index of the module within that chain, in chain order. */
  moduleIndex: number;
  end: ConnectorEnd;
}

export interface WeldAnchor extends AnchorCandidateKey {
  moduleId: ModuleId;
  /** World pose of the chosen connector -- weld the arm's connector A onto this. */
  pose: Pose;
  /** How far the connector sits from the branch cube it stands in for. */
  distance: number;
}

export interface FittedChainRef {
  chainKey: string;
  assembly: Assembly;
}

function candidateId(key: AnchorCandidateKey): string {
  return `${key.chainKey}:${key.moduleIndex}:${key.end}`;
}

export function anchorKeyId(key: AnchorCandidateKey): string {
  return candidateId(key);
}

/**
 * Finds the free connector nearest `branchCenter` that faces along
 * `armDirection`, searching every already-fitted chain. Returns null when
 * nothing qualifies, which the caller should treat as "leave this arm
 * free-standing" rather than as an error.
 *
 * `consumed` holds the ids (see `anchorKeyId`) of connectors already claimed
 * by earlier arms at this or another branch -- a single connector can only
 * ever take one weld.
 */
export function findWeldAnchor(
  chains: readonly FittedChainRef[],
  branchCenter: Vector3,
  armDirection: Vector3,
  consumed: ReadonlySet<string>,
): WeldAnchor | null {
  const direction = armDirection.clone().normalize();
  let best: WeldAnchor | null = null;

  for (const chain of chains) {
    const transforms = computeAssemblyWorldTransforms(chain.assembly);
    const modules = Object.values(chain.assembly.modules);

    modules.forEach((module, moduleIndex) => {
      const moduleTransforms = transforms.get(module.id);
      if (!moduleTransforms) return;

      for (const connector of allConnectors(module)) {
        // Connectors welded inside their own chain are already spoken for.
        if (connector.locked) continue;
        const key: AnchorCandidateKey = { chainKey: chain.chainKey, moduleIndex, end: connector.end };
        if (consumed.has(candidateId(key))) continue;

        const pose = connectorPose(moduleTransforms, connector.end);
        if (outwardNormal(pose).dot(direction) < MIN_ANCHOR_ALIGNMENT) continue;

        const distance = worldPosition(pose).distanceTo(branchCenter);
        if (!best || distance < best.distance) {
          best = { ...key, moduleId: module.id, pose, distance };
        }
      }
    });
  }

  return best;
}
