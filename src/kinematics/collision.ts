/**
 * Self-collision: does this configuration have the robot passing through
 * itself?
 *
 * Every kinematic result so far answers "can the joints reach this pose"
 * without asking whether getting there means driving a rod through a
 * neighbouring one. VTT's published experience is that self-collision, not
 * kinematics, is what actually limits a reconfiguring truss in practice -- so
 * a plan that is verified reachable is still not verified buildable until it
 * has been through here.
 *
 * Geometry model: every part becomes a CAPSULE -- a line segment with a
 * radius. Rods already are capsules in all but name, and a connector dome is
 * near enough a sphere (a capsule of zero length). Capsule-capsule distance is
 * exactly segment-segment distance minus the two radii, which is a short
 * closed-form calculation with no iteration and no special cases. Using the
 * real triangle meshes would be far more code for a more precise answer than
 * this problem needs.
 *
 * What is deliberately NOT a collision:
 *  - Two parts of the same module. They are built to fold past each other, and
 *    the rods either side of a joint always touch at the joint.
 *  - Parts either side of a weld. Not merely the two welded connectors -- the
 *    rod each one sits on overlaps its opposite number too, because a weld
 *    pulls both modules' end geometry into the same place on purpose. Excusing
 *    only the connector pair still reports a plain straight chain as colliding.
 *
 * The second exclusion is scoped as tightly as it can be: only the parts
 * immediately adjacent to that specific weld are excused, not the two modules
 * wholesale. Robotics tooling usually disables collision between adjacent
 * links entirely, but these modules have three +/-90 degree bends each and can
 * genuinely fold back onto a welded neighbour -- a real collision that a
 * blanket exclusion would hide.
 */
import { Quaternion, Vector3 } from 'three';
import type { Assembly, ConnectorEnd, Module, ModuleId } from '../types/module';
import { allConnectors } from '../types/module';
import { BIG_ROD_INDEX, HEMISPHERE_RADIUS, ROD_RADIUS } from '../constants/geometry';
import { computeAssemblyKinematics, connectorPose, findConnector } from './assemblyGraph';
import { rodLength, type ModuleWorldTransforms } from './forwardKinematics';

/** A line segment with a radius -- the swept volume of a rod or a connector dome. */
interface Capsule {
  moduleId: ModuleId;
  /** Identifies the part within its module, for reporting. */
  part: string;
  start: Vector3;
  end: Vector3;
  radius: number;
  /** Connector id when this capsule IS a connector, so welded pairs can be excused. */
  connectorId?: string;
}

export interface CollisionPair {
  a: { moduleId: ModuleId; part: string };
  b: { moduleId: ModuleId; part: string };
  /** How deeply they overlap, in world units. Always > 0 for a reported pair. */
  penetration: number;
}

export interface CollisionReport {
  collides: boolean;
  pairs: CollisionPair[];
  /** Worst overlap found, 0 when clear. */
  worstPenetration: number;
  /** Capsules tested, for a sense of cost. */
  capsuleCount: number;
}

export interface CollisionOptions {
  /**
   * Overlap below this is ignored. Parts that legitimately meet -- a connector
   * resting against the rod it is mounted on -- share surfaces by design, and
   * the fitted geometry has its own small residual, so a strict zero would
   * report contacts that are not faults.
   */
  tolerance?: number;
  /** Stop at the first real overlap. Much faster when the answer is just yes/no. */
  firstOnly?: boolean;
}

/**
 * Rods lying within one connector-radius of each connector, and therefore
 * inside the ball a weld there legitimately occupies.
 *
 * A connector dome is `HEMISPHERE_RADIUS` (0.42) across while a rod is only
 * 0.5 long, so the sphere formed by a welded pair swallows the whole first rod
 * on each side and reaches into the second. Excusing only the rod each
 * connector directly caps still leaves a plain straight chain reporting a
 * 0.06 overlap against its neighbour's second rod -- geometry that is real,
 * intended, and not a fault. Connector A caps rod 1 and B caps rod 6; the four
 * side connectors ride the middle of the big spine rod and so reach into the
 * rods on either side of it.
 */
const WELD_REGION_RODS: Record<ConnectorEnd, number[]> = {
  A: [0, 1],
  B: [5, 4],
  UP: [BIG_ROD_INDEX - 1, BIG_ROD_INDEX, BIG_ROD_INDEX + 1],
  RIGHT: [BIG_ROD_INDEX - 1, BIG_ROD_INDEX, BIG_ROD_INDEX + 1],
  DOWN: [BIG_ROD_INDEX - 1, BIG_ROD_INDEX, BIG_ROD_INDEX + 1],
  LEFT: [BIG_ROD_INDEX - 1, BIG_ROD_INDEX, BIG_ROD_INDEX + 1],
};

/**
 * The parts a weld at `end` legitimately pushes into the other module's
 * geometry: the connector itself, the rods within its reach, and -- for a side
 * connector -- its three siblings.
 *
 * That last clause matters. The four side connectors sit only 0.5 from the rod
 * axis while each dome is 0.42 across, so neighbouring domes overlap ~0.13 by
 * design and read as one solid hub (see SIDE_CONNECTOR_RADIAL_OFFSET, where
 * that offset is boxed in from both directions). Anything welded onto one of
 * them lands in the middle of that hub and inherits the same overlap, so
 * without this every branch in a synthesized shape reports a false collision.
 */
function weldRegion(end: ConnectorEnd): string[] {
  const rods = WELD_REGION_RODS[end].map((i) => `rod${i + 1}`);
  if (end === 'A' || end === 'B') return [`connector${end}`, ...rods];
  return ['connectorUP', 'connectorRIGHT', 'connectorDOWN', 'connectorLEFT', ...rods];
}

function partKey(moduleId: ModuleId, part: string): string {
  return `${moduleId}:${part}`;
}

function pairKey(a: string, b: string): string {
  return a < b ? `${a}|${b}` : `${b}|${a}`;
}

/** Squared distance between two segments, plus nothing else -- the whole geometric core. */
function segmentDistanceSquared(p1: Vector3, q1: Vector3, p2: Vector3, q2: Vector3): number {
  const d1 = q1.clone().sub(p1);
  const d2 = q2.clone().sub(p2);
  const r = p1.clone().sub(p2);
  const a = d1.dot(d1);
  const e = d2.dot(d2);
  const f = d2.dot(r);

  const EPS = 1e-12;
  let s: number;
  let t: number;

  if (a <= EPS && e <= EPS) return r.dot(r); // both degenerate to points
  if (a <= EPS) {
    s = 0;
    t = Math.min(1, Math.max(0, f / e));
  } else {
    const c = d1.dot(r);
    if (e <= EPS) {
      t = 0;
      s = Math.min(1, Math.max(0, -c / a));
    } else {
      const b = d1.dot(d2);
      const denom = a * e - b * b;
      // denom is zero exactly when the segments are parallel; pinning s to 0
      // and letting t clamp handles that without a separate branch.
      s = denom > EPS ? Math.min(1, Math.max(0, (b * f - c * e) / denom)) : 0;
      t = (b * s + f) / e;
      if (t < 0) {
        t = 0;
        s = Math.min(1, Math.max(0, -c / a));
      } else if (t > 1) {
        t = 1;
        s = Math.min(1, Math.max(0, (b - c) / a));
      }
    }
  }

  const closest1 = p1.clone().addScaledVector(d1, s);
  const closest2 = p2.clone().addScaledVector(d2, t);
  return closest1.distanceToSquared(closest2);
}

/** Builds the capsule set for one module at its current world pose. */
function moduleCapsules(module: Module, transforms: ModuleWorldTransforms): Capsule[] {
  const capsules: Capsule[] = [];

  module.rods.forEach((rod, i) => {
    const pose = transforms.rods[i]!;
    const origin = new Vector3(...pose.position);
    // A rod runs from its pivot along its frame's +Z for its own physical
    // length -- rodLength already special-cases the oversized spine rod.
    const forward = new Vector3(0, 0, 1)
      .applyQuaternion(new Quaternion(...pose.quaternion))
      .multiplyScalar(rodLength(rod.kind, i));
    capsules.push({
      moduleId: module.id,
      part: `rod${i + 1}`,
      start: origin,
      end: origin.clone().add(forward),
      radius: ROD_RADIUS,
    });
  });

  // A connector dome is a sphere: a capsule whose segment has zero length.
  for (const connector of allConnectors(module)) {
    const centre = new Vector3(...connectorPose(transforms, connector.end).position);
    capsules.push({
      moduleId: module.id,
      part: `connector${connector.end}`,
      start: centre,
      end: centre.clone(),
      radius: HEMISPHERE_RADIUS,
      connectorId: connector.id,
    });
  }

  return capsules;
}

/**
 * Checks an assembly for parts passing through each other.
 *
 * Cost is quadratic in capsule count, but each test is a handful of dot
 * products and the bounding-sphere reject below throws out almost every pair
 * before the exact test runs -- parts of a sprawling structure are mostly
 * nowhere near each other.
 */
export function checkSelfCollision(assembly: Assembly, options: CollisionOptions = {}): CollisionReport {
  const tolerance = options.tolerance ?? 0.02;
  const { transforms } = computeAssemblyKinematics(assembly);

  const capsules: Capsule[] = [];
  /** Part pairs excused because a weld deliberately pushes them together. */
  const excused = new Set<string>();

  for (const module of Object.values(assembly.modules)) {
    const moduleTransforms = transforms.get(module.id);
    if (!moduleTransforms) continue;
    capsules.push(...moduleCapsules(module, moduleTransforms));

    for (const connector of allConnectors(module)) {
      if (!connector.locked || !connector.connectedTo) continue;
      const partner = findConnector(assembly, connector.connectedTo);
      if (!partner) continue;
      for (const here of weldRegion(connector.end)) {
        for (const there of weldRegion(partner.end)) {
          excused.add(pairKey(partKey(module.id, here), partKey(partner.moduleId, there)));
        }
      }
    }
  }

  const pairs: CollisionPair[] = [];
  let worstPenetration = 0;

  for (let i = 0; i < capsules.length; i += 1) {
    for (let j = i + 1; j < capsules.length; j += 1) {
      const a = capsules[i]!;
      const b = capsules[j]!;

      // Same module: built to fold past itself, and rods sharing a joint touch by design.
      if (a.moduleId === b.moduleId) continue;
      // Either side of a weld: overlapping there is the point of a weld.
      if (excused.has(pairKey(partKey(a.moduleId, a.part), partKey(b.moduleId, b.part)))) continue;

      const clearance = a.radius + b.radius - tolerance;
      // Cheap reject: if the segment endpoints cannot be close enough even at
      // their most generous, skip the exact test entirely.
      const reach = clearance + a.start.distanceTo(a.end) + b.start.distanceTo(b.end);
      if (a.start.distanceToSquared(b.start) > reach * reach) continue;

      const distanceSquared = segmentDistanceSquared(a.start, a.end, b.start, b.end);
      if (distanceSquared >= clearance * clearance) continue;

      const penetration = clearance - Math.sqrt(distanceSquared);
      worstPenetration = Math.max(worstPenetration, penetration);
      pairs.push({
        a: { moduleId: a.moduleId, part: a.part },
        b: { moduleId: b.moduleId, part: b.part },
        penetration,
      });
      if (options.firstOnly) {
        return { collides: true, pairs, worstPenetration, capsuleCount: capsules.length };
      }
    }
  }

  return {
    collides: pairs.length > 0,
    pairs,
    worstPenetration,
    capsuleCount: capsules.length,
  };
}

/** Convenience predicate for planners, which only ever want the yes/no. */
export function isSelfColliding(assembly: Assembly, options: CollisionOptions = {}): boolean {
  return checkSelfCollision(assembly, { ...options, firstOnly: true }).collides;
}
