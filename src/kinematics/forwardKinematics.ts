/**
 * Forward kinematics: pure functions from joint angles -> world poses.
 *
 * Frame convention
 * -----------------
 * Every connector has its own local frame whose +Z axis is the connector's
 * *outward normal* (the direction the hemisphere dome faces). `Module.basePose`
 * IS connector A's world pose under this convention.
 *
 * To walk from connector A to connector B we flip 180 degrees about local X
 * once (`FLIP_X_180`) to turn "A's outward frame" into "the frame that looks
 * into the chain", then walk rod-by-rod: gap -> joint rotation (pivot) ->
 * rod length. After the last rod the running frame's +Z already points away
 * from the module -- that's exactly connector B's outward frame, no further
 * flip needed.
 */
import type { ConnectorEnd, Module, Pose, Rod, RodKind } from '../types/module';
import { ROD_KIND_SEQUENCE, isSideConnectorEnd, sideConnectorIndex } from '../types/module';
import {
  BEND_ROD_LENGTH,
  BIG_ROD_INDEX,
  BIG_ROD_LENGTH_SCALE,
  SEGMENT_GAP,
  SIDE_CONNECTOR_RADIAL_OFFSET,
  TWIST_ROD_LENGTH,
} from '../constants/geometry';
import { FLIP_X_180, IDENTITY_POSE, composePoses, rotateX, rotateY, rotateZ, translateZ } from './frame';

/**
 * Physical length of a rod. Rod index 3 (the twist rod flanked by a bend on
 * both sides -- the module's central "spine" segment) is rendered and
 * kinematically treated as `BIG_ROD_LENGTH_SCALE` times longer than the
 * other rods of its kind -- length only, same radius as every other rod.
 */
export function rodLength(kind: RodKind, rodIndex: number): number {
  const base = kind === 'twist' ? TWIST_ROD_LENGTH : BEND_ROD_LENGTH;
  return rodIndex === BIG_ROD_INDEX ? base * BIG_ROD_LENGTH_SCALE : base;
}

/** The local joint transform a rod contributes at its pivot, given its current angle. */
function jointTransform(rod: Rod): Pose {
  return rod.kind === 'twist' ? rotateZ(rod.angle) : rotateX(rod.angle);
}

export interface ModuleWorldTransforms {
  connectorA: Pose;
  /** World pose of each rod's pivot frame, in chain order 1-6. */
  rods: [Pose, Pose, Pose, Pose, Pose, Pose];
  connectorB: Pose;
  /** World poses of the 4 big-rod side connectors, in `SIDE_CONNECTOR_ENDS` order. */
  sides: [Pose, Pose, Pose, Pose];
}

/**
 * Turns the big rod's own frame (+Z running along the rod) to face each side
 * connector's outward direction, in `SIDE_CONNECTOR_ENDS` order: UP (+Y),
 * RIGHT (+X), DOWN (-Y), LEFT (-X). Each is a single-axis quarter turn, which
 * keeps the resulting connector frame's roll deterministic -- a welded
 * neighbour's orientation about the lock axis follows from these directly.
 */
const SIDE_ROTATIONS: readonly Pose[] = [
  rotateX(-Math.PI / 2),
  rotateY(Math.PI / 2),
  rotateX(Math.PI / 2),
  rotateY(-Math.PI / 2),
];

/**
 * Local transform from the big rod's pivot frame out to side connector
 * `index`: forward to the rod's midpoint, turn to face radially outward, then
 * straight out to where the flat lock face sits.
 */
function sideConnectorLocalTransform(index: number): Pose {
  const halfRod = rodLength('twist', BIG_ROD_INDEX) / 2;
  return composePoses(
    composePoses(translateZ(halfRod), SIDE_ROTATIONS[index]),
    translateZ(SIDE_CONNECTOR_RADIAL_OFFSET),
  );
}

/**
 * Walks the 6-rod chain starting from `connectorAWorldPose`, returning the
 * world pose of connector A itself, each rod pivot, and connector B.
 */
export function computeModuleTransforms(
  rods: Module['rods'],
  connectorAWorldPose: Pose,
): ModuleWorldTransforms {
  let running = composePoses(connectorAWorldPose, FLIP_X_180);
  const rodPoses: Pose[] = [];

  rods.forEach((rod, rodIndex) => {
    running = composePoses(running, translateZ(SEGMENT_GAP));
    running = composePoses(running, jointTransform(rod));
    rodPoses.push(running);
    running = composePoses(running, translateZ(rodLength(rod.kind, rodIndex)));
  });
  running = composePoses(running, translateZ(SEGMENT_GAP));

  const bigRodPose = rodPoses[BIG_ROD_INDEX];
  const sides = SIDE_ROTATIONS.map((_, i) =>
    composePoses(bigRodPose, sideConnectorLocalTransform(i)),
  ) as ModuleWorldTransforms['sides'];

  return {
    connectorA: connectorAWorldPose,
    rods: rodPoses as ModuleWorldTransforms['rods'],
    connectorB: running,
    sides,
  };
}

/**
 * The local transform from connector A's frame to connector B's frame for a
 * given set of joint angles -- i.e. `computeModuleTransforms` anchored at the
 * identity pose. Used to back-solve a module's connector-A world pose when it
 * is reached through connector B during assembly graph traversal.
 */
export function localChainTransform(rods: Module['rods']): Pose {
  return computeModuleTransforms(rods, IDENTITY_POSE).connectorB;
}

/**
 * The local transform from connector A's frame to `end`'s frame, for ANY of
 * the module's 6 connectors. Generalizes `localChainTransform` so graph
 * traversal can back-solve a module's connector-A pose no matter which of its
 * connectors it was reached through -- A itself needs no correction, B walks
 * the full rod chain, and a side connector branches off the big rod midway.
 */
export function localConnectorTransform(rods: Module['rods'], end: ConnectorEnd): Pose {
  if (end === 'A') return IDENTITY_POSE;
  const transforms = computeModuleTransforms(rods, IDENTITY_POSE);
  if (!isSideConnectorEnd(end)) return transforms.connectorB;
  return transforms.sides[sideConnectorIndex(end)];
}

/** Sanity guard: throws if a module's rods aren't in the canonical kind order. */
export function assertCanonicalRodOrder(rods: Module['rods']): void {
  rods.forEach((rod, i) => {
    if (rod.kind !== ROD_KIND_SEQUENCE[i]) {
      throw new Error(
        `Rod ${i + 1} must be '${ROD_KIND_SEQUENCE[i]}' per the fixed module anatomy, got '${rod.kind}'`,
      );
    }
  });
}
