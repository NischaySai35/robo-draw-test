/**
 * Shared BufferGeometry singletons. Every module in the assembly has the
 * exact same topology, so geometry is built once here and reused across all
 * module instances -- only the per-mesh transform differs.
 *
 * All shapes are authored with their local +Z as the "forward" axis (chain
 * direction / outward normal), since that's the axis our pose convention
 * (see kinematics/frame.ts) treats as canonical. Three.js primitives default
 * to +Y, so each geometry is rotated +90 deg about X right after construction
 * to re-align its axis to +Z before anything reads it -- note the sign: +90
 * (not -90) is the one that actually sends +Y to +Z under THREE's rotation
 * convention.
 */
import { BoxGeometry, CircleGeometry, CylinderGeometry, Matrix4, Quaternion, SphereGeometry, TorusGeometry, Vector3 } from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import {
  BEND_KNUCKLE_LENGTH,
  BEND_KNUCKLE_RADIUS,
  BEND_ROD_LENGTH,
  BIG_ROD_LENGTH_SCALE,
  DOCK_NUB_COUNT,
  DOCK_NUB_POLAR_ANGLE_DEG,
  DOCK_NUB_RADIUS,
  HEMISPHERE_RADIUS,
  RADIAL_SEGMENTS,
  ROD_RADIUS,
  TWIST_ROD_LENGTH,
  TWIST_STRIPE_DEPTH,
  TWIST_STRIPE_LENGTH_FRACTION,
  TWIST_STRIPE_WIDTH,
} from '../constants/geometry';

const ALIGN_TO_Z = new Matrix4().makeRotationX(Math.PI / 2);
const FORWARD = new Vector3(0, 0, 1);

/**
 * A connector dome: half-sphere with a flat disc closing its base, oriented
 * FLAT-FACE-OUT -- the flat side faces along the connector's outward normal
 * (+Z) and the curved dome bulges backward, in toward the module's own rod.
 *
 * This orientation is used for every connector, locked or free. At a locked
 * joint the two connectors' flat faces meet flush against each other, so the
 * joint reads as a flat disc junction with the two domes bulging away into
 * their respective rods -- there is deliberately no separate "locked" shape.
 *
 * The flat side is a real `CircleGeometry` disc merged into the same
 * BufferGeometry as the dome, not a separate mesh: a bare `SphereGeometry`
 * with a partial `thetaLength` does NOT close its own open end, so without
 * this the connector would be a hollow shell with a visible hole where the
 * flat face belongs. Merging (rather than adding a second mesh flush against
 * the first) means there's no gap between the two pieces for a seam to show
 * through.
 */
function makeHemisphereGeometry() {
  const dome = new SphereGeometry(
    HEMISPHERE_RADIUS,
    RADIAL_SEGMENTS,
    Math.max(8, RADIAL_SEGMENTS / 2),
    0,
    Math.PI * 2,
    0,
    Math.PI / 2,
  );
  // Default CircleGeometry lies flat in XY at z=0 facing +Z; rotating +90 deg about X moves it
  // to the dome's open base (the XZ plane at y=0, where SphereGeometry's equator sits for
  // thetaLength=PI/2) AND flips its normal to face away from the dome's apex, which is the
  // direction a mounting-face cap needs to point.
  const cap = new CircleGeometry(HEMISPHERE_RADIUS, RADIAL_SEGMENTS);
  cap.rotateX(Math.PI / 2);
  const geo = mergeGeometries([dome, cap]);
  geo.applyMatrix4(ALIGN_TO_Z);
  // ALIGN_TO_Z leaves the dome bulging toward +Z (flat cap at the origin). Rotating a further
  // 180 deg about X swaps them: flat cap now faces +Z (outward), dome bulges toward -Z (back
  // toward the rod). This is the flat-face-out orientation described above.
  geo.rotateX(Math.PI);
  return geo;
}

/** A rod cylinder running from the local origin to `length` along +Z, with the given radius. */
function makeRodGeometry(length: number, radius: number) {
  const geo = new CylinderGeometry(radius, radius, length, RADIAL_SEGMENTS);
  geo.applyMatrix4(ALIGN_TO_Z);
  geo.translate(0, 0, length / 2);
  return geo;
}

/** A thin surface groove marking rotation on a twist rod, offset to the rod's radius. */
function makeStripeGeometry(length: number, radius: number) {
  const grooveLength = length * TWIST_STRIPE_LENGTH_FRACTION;
  const geo = new BoxGeometry(TWIST_STRIPE_WIDTH, TWIST_STRIPE_DEPTH, grooveLength);
  geo.translate(0, radius - TWIST_STRIPE_DEPTH / 2, length / 2);
  return geo;
}

/** A hinge-pin knuckle at a bend joint's pivot, its axis along local X (the bend axis). */
function makeKnuckleGeometry() {
  const geo = new CylinderGeometry(BEND_KNUCKLE_RADIUS, BEND_KNUCKLE_RADIUS, BEND_KNUCKLE_LENGTH, RADIAL_SEGMENTS);
  geo.rotateZ(Math.PI / 2); // cylinder defaults to +Y axis -> rotate so its axis is local X
  return geo;
}

function makePadlockBodyGeometry() {
  return new BoxGeometry(0.08, 0.06, 0.045);
}

function makePadlockShackleGeometry() {
  const geo = new TorusGeometry(0.035, 0.009, 8, 16, Math.PI);
  geo.rotateZ(Math.PI);
  geo.translate(0, 0.035, 0);
  return geo;
}

/** One small "lock nub" hemisphere -- a docking-face keying detail, not an independent lock point. */
function makeDockNubGeometry() {
  const geo = new SphereGeometry(DOCK_NUB_RADIUS, 12, 8, 0, Math.PI * 2, 0, Math.PI / 2);
  geo.applyMatrix4(ALIGN_TO_Z);
  return geo;
}

/**
 * Position + orientation for each of the 4 dock nubs, evenly spaced around
 * the main dome. Each nub sits exactly ON the dome's spherical surface --
 * `DOCK_NUB_POLAR_ANGLE_DEG` from the dome's apex -- and is oriented so it
 * bulges further outward along ITS OWN radial direction from the dome
 * center, the same way a rivet head pokes out of a curved panel. Placing
 * nubs at a fixed *fraction* of the dome's radius (rather than on its
 * surface) buried them inside the larger dome's volume, where they were
 * fully occluded and invisible.
 *
 * Note the negated Z: the connector dome bulges toward -Z (flat face out,
 * see `makeHemisphereGeometry`), so the nubs' radial directions point back
 * that way too. Using +Z here would scatter them across the flat mounting
 * face instead of the curved surface they belong on.
 */
export interface DockNubPlacement {
  position: Vector3;
  quaternion: Quaternion;
}

export const DOCK_NUB_PLACEMENTS: readonly DockNubPlacement[] = Array.from({ length: DOCK_NUB_COUNT }, (_, i) => {
  const azimuth = (i / DOCK_NUB_COUNT) * Math.PI * 2;
  const polar = (DOCK_NUB_POLAR_ANGLE_DEG * Math.PI) / 180;
  const radial = new Vector3(Math.sin(polar) * Math.cos(azimuth), Math.sin(polar) * Math.sin(azimuth), -Math.cos(polar));
  return {
    position: radial.clone().multiplyScalar(HEMISPHERE_RADIUS),
    quaternion: new Quaternion().setFromUnitVectors(FORWARD, radial),
  };
});

export const geometries = {
  hemisphere: makeHemisphereGeometry(),
  rodTwist: makeRodGeometry(TWIST_ROD_LENGTH, ROD_RADIUS),
  rodBend: makeRodGeometry(BEND_ROD_LENGTH, ROD_RADIUS),
  /** The elongated central spine rod (rod index 3 -- the twist flanked by bends on both sides). Longer only, same radius. */
  rodTwistBig: makeRodGeometry(TWIST_ROD_LENGTH * BIG_ROD_LENGTH_SCALE, ROD_RADIUS),
  stripeTwist: makeStripeGeometry(TWIST_ROD_LENGTH, ROD_RADIUS),
  stripeTwistBig: makeStripeGeometry(TWIST_ROD_LENGTH * BIG_ROD_LENGTH_SCALE, ROD_RADIUS),
  knuckle: makeKnuckleGeometry(),
  padlockBody: makePadlockBodyGeometry(),
  padlockShackle: makePadlockShackleGeometry(),
  dockNub: makeDockNubGeometry(),
};

/** `rodIndex` picks the elongated spine-rod geometry variant when it applies (see `BIG_ROD_INDEX`). */
export function rodGeometryFor(kind: 'twist' | 'bend', isBig: boolean) {
  if (kind === 'twist') return isBig ? geometries.rodTwistBig : geometries.rodTwist;
  return geometries.rodBend;
}

export function stripeGeometryFor(isBig: boolean) {
  return isBig ? geometries.stripeTwistBig : geometries.stripeTwist;
}
