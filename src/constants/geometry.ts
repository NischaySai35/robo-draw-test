/**
 * Every retunable length/radius for the procedural module geometry and joint
 * defaults lives here so the visual proportions can be adjusted in one place.
 * Units are meters (arbitrary scene units); angles in these constants are
 * radians unless the name says "Deg".
 */

export const HEMISPHERE_RADIUS = 0.42;
/** Radial segments used for hemisphere + rod cylinder geometry. */
export const RADIAL_SEGMENTS = 24;

// Rods are kept noticeably thinner than the hemisphere radius so a locked
// connector pair reads clearly as one continuous ball-joint, not a rod-width
// lump with a slightly wider cap on it.
export const ROD_RADIUS = 0.2;
export const TWIST_ROD_LENGTH = 0.5;
export const BEND_ROD_LENGTH = 0.55;

/**
 * Rod 4 (0-indexed 3) is the only twist rod flanked by a BEND joint on both
 * sides (rod3=BEND, rod5=BEND) -- it's the module's central "spine" segment,
 * so it's rendered noticeably longer than the other five rods. Length only --
 * same radius as every other rod, not a fatter one.
 */
export const BIG_ROD_INDEX = 3;
export const BIG_ROD_LENGTH_SCALE = 2;

/**
 * How far a side connector's flat lock face sits from the big rod's axis.
 *
 * The 4 side connectors ride the middle of the big rod pointing +Y/+X/-Y/-X,
 * which together with connectors A and B at the chain ends gives a module 6
 * lock faces -- the 6 faces of a cube, so a module can branch sideways instead
 * of only chaining end-to-end.
 *
 * This value is boxed in from both directions and should not be nudged
 * casually:
 *  - It MUST exceed `HEMISPHERE_RADIUS`. Each lock face is a flat disc of that
 *    radius standing perpendicular to its own outward direction, so at any
 *    smaller offset the +X disc reaches past the +Y disc's plane (and vice
 *    versa) and the four discs visibly slice through each other in an X.
 *  - It MUST stay under `HEMISPHERE_RADIUS + ROD_RADIUS`, or the connector's
 *    dome -- which bulges `HEMISPHERE_RADIUS` back in toward the axis -- stops
 *    reaching the rod's surface at all and the connector floats detached.
 * Adjacent domes do overlap slightly at this offset; that's intentional and
 * reads as one solid hub, since they share a material and both are solid.
 */
export const SIDE_CONNECTOR_RADIAL_OFFSET = 0.5;

/** Width/depth of the visible groove marking a twist rod's rotation indicator -- a subtle recessed line, not a fin. */
export const TWIST_STRIPE_WIDTH = 0.045;
export const TWIST_STRIPE_DEPTH = 0.012;
/** Fraction of the rod's length the stripe groove covers. */
export const TWIST_STRIPE_LENGTH_FRACTION = 0.55;

/** Radius/length of the hinge knuckle drawn at a bend joint -- a small collar, not wider than the rod by much. */
export const BEND_KNUCKLE_RADIUS = 0.23;
export const BEND_KNUCKLE_LENGTH = 0.16;

/** Small axial gap rendered between adjacent segments so joints read visually. */
export const SEGMENT_GAP = 0.02;

/**
 * Each connector's dock face carries 4 small "lock nub" hemispheres arranged
 * around the rim of the main dome, at 0/90/180/270 degrees -- a keying
 * pattern (like a 4-bolt flange), purely a docking-geometry detail. The
 * connection itself is still exactly one big-dome-to-big-dome lock, same
 * math as before; the nubs don't add any new lock points of their own.
 *
 * Nub centers sit exactly ON the main dome's spherical surface (not buried
 * inside it) at `DOCK_NUB_POLAR_ANGLE_DEG` from the pole -- close to the
 * base rim but still clearly on the dome -- and each nub is oriented to
 * bulge further outward along ITS OWN radial direction from the dome
 * center, so it visibly pokes out as a bump rather than disappearing into
 * the larger dome's volume.
 */
export const DOCK_NUB_COUNT = 4;
export const DOCK_NUB_RADIUS = HEMISPHERE_RADIUS * 0.24;
export const DOCK_NUB_POLAR_ANGLE_DEG = 68;

export const TWIST_LIMIT_DEFAULT_MIN_DEG = 0;
export const TWIST_LIMIT_DEFAULT_MAX_DEG = 360;
export const BEND_LIMIT_DEFAULT_MIN_DEG = -90;
export const BEND_LIMIT_DEFAULT_MAX_DEG = 90;

/** Distance within which two open connectors show a snap preview. */
export const SNAP_DISTANCE_TOLERANCE = 0.35;
/** Max angle (radians) between opposing connector normals to still count as "facing". */
export const SNAP_ANGLE_TOLERANCE = 0.35; // ~20 deg

/**
 * Total straight-line chain length from connector A to connector B (rods +
 * gaps only; hemispheres sit outside A/B). Rod 4 is `BIG_ROD_LENGTH_SCALE`
 * longer than the other two twist rods, so it's counted separately.
 */
export const MODULE_CHAIN_LENGTH =
  TWIST_ROD_LENGTH * 2 + TWIST_ROD_LENGTH * BIG_ROD_LENGTH_SCALE + BEND_ROD_LENGTH * 3 + SEGMENT_GAP * 7;

export const COLORS = {
  hemisphereLocked: 0x3a7bd5,
  hemisphereUnlocked: 0x8894a3,
  rodTwist: 0x4a5568,
  rodBend: 0x4a5568,
  stripeEnergized: 0x33d17a,
  stripeDeenergized: 0x9aa4b2,
  knuckleEnergized: 0x33d17a,
  knuckleDeenergized: 0x9aa4b2,
  selection: 0xffb703,
  ghostSnap: 0x33d17a,
  padlockLocked: 0x33d17a,
  padlockUnlocked: 0x5c6577,
} as const;
