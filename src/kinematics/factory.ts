/**
 * Constructors for fresh modules/rods/connectors with sane defaults, plus a
 * lightweight id generator (avoids depending on `crypto.randomUUID`, which
 * isn't guaranteed in every embedding target we might run in later).
 */
import type { Connector, ConnectorEnd, Module, Pose, Rod, RodKind } from '../types/module';
import { ROD_KIND_SEQUENCE, SIDE_CONNECTOR_ENDS } from '../types/module';
import {
  BEND_LIMIT_DEFAULT_MAX_DEG,
  BEND_LIMIT_DEFAULT_MIN_DEG,
  TWIST_LIMIT_DEFAULT_MAX_DEG,
  TWIST_LIMIT_DEFAULT_MIN_DEG,
} from '../constants/geometry';
import { IDENTITY_POSE } from './frame';

let counter = 0;
export function makeId(prefix: string): string {
  counter += 1;
  return `${prefix}_${Date.now().toString(36)}_${counter}_${Math.floor(Math.random() * 1e6).toString(36)}`;
}

const DEG = Math.PI / 180;

function createRod(kind: RodKind): Rod {
  const [minDeg, maxDeg] =
    kind === 'twist'
      ? [TWIST_LIMIT_DEFAULT_MIN_DEG, TWIST_LIMIT_DEFAULT_MAX_DEG]
      : [BEND_LIMIT_DEFAULT_MIN_DEG, BEND_LIMIT_DEFAULT_MAX_DEG];
  return {
    id: makeId('rod'),
    kind,
    angle: 0,
    min: minDeg * DEG,
    max: maxDeg * DEG,
    home: 0,
    // De-energized by default -- a fresh joint free-swings until the user
    // explicitly turns torque on, same convention as the electromagnet locks.
    torqueEnabled: false,
  };
}

function createConnector(moduleId: string, end: ConnectorEnd): Connector {
  return {
    id: makeId('conn'),
    moduleId,
    end,
    locked: false,
    connectedTo: null,
  };
}

export function createModule(basePose: Pose = IDENTITY_POSE): Module {
  const moduleId = makeId('mod');
  const rods = ROD_KIND_SEQUENCE.map(createRod) as Module['rods'];
  const connectorA = createConnector(moduleId, 'A');
  const connectorB = createConnector(moduleId, 'B');
  const sides = SIDE_CONNECTOR_ENDS.map((end) => createConnector(moduleId, end)) as Module['sides'];
  return { id: moduleId, basePose, rods, connectorA, connectorB, sides };
}

/**
 * Clamp a proposed joint angle to the rod's configured [min, max] range.
 *
 * A rotation angle is only meaningful modulo 2π, but a raw computed angle
 * (e.g. from `atan2`) always comes back in (-180°, 180°]. For a rod whose
 * range extends past that -- TWIST defaults to 0-360° -- a "needed" angle
 * like -87° is the exact same physical rotation as 273°, which fits the
 * range fine; naively clamping the raw -87° straight to the 0° boundary
 * throws away nearly the whole intended rotation for no reason. So: try the
 * angle as given, then its -360°/+360° equivalents, and use whichever one
 * actually lands inside [min, max] without any clamping at all. Only if
 * none of the three do (a genuinely out-of-range request, e.g. BEND asking
 * for 150° against a ±90° limit) do we fall back to clamping the original.
 */
export function clampRodAngle(rod: Pick<Rod, 'min' | 'max'>, angle: number): number {
  const TWO_PI = Math.PI * 2;
  for (const candidate of [angle, angle - TWO_PI, angle + TWO_PI]) {
    if (candidate >= rod.min && candidate <= rod.max) return candidate;
  }
  return Math.min(rod.max, Math.max(rod.min, angle));
}
