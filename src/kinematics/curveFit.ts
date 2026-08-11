/**
 * Draw-to-build / cube-builder curve fitting.
 *
 * PURE-PURSUIT JOINT TRACKING + COORDINATE-DESCENT REFINEMENT -- a
 * from-scratch fit designed for this module's specific joint alphabet
 * (TWIST-BEND-BEND-TWIST-BEND-TWIST), not a generic textbook IK method.
 *
 * PASS 1 -- pure pursuit (the baseline). Adapted from the path-tracking
 * technique used to steer a vehicle along a planned path by always aiming at
 * a lookahead point on the path closest to where the vehicle *actually* is.
 * Every joint aims at a point a short distance further along the stroke from
 * wherever the chain's *actual, already-computed* position currently sits --
 * not at a position that was precomputed once from an assumed arc-length
 * schedule. That grounding makes the pass self-correcting *going forward*:
 * if an earlier joint got clamped short, every joint after it re-anchors
 * against the real curve from wherever the chain actually ended up.
 *
 * It also gives two properties for free: STABILITY WITH SPARSE INPUT (a
 * joint's confidence in its aim point is tied to how much real stroke
 * exists ahead of its projection relative to the lookahead it wants, so a
 * couple of jittery points early in a drag can't swing the fit), and NEVER
 * FORCING A STRETCH (once the closest point is at the stroke's own end,
 * confidence drops to zero and later joints just continue straight instead
 * of overreaching).
 *
 * PASS 2 -- coordinate-descent refinement. Pure pursuit alone is still only
 * a *forward* pass: joint 1 is decided and never reconsidered, even if a
 * slightly different choice there would let joints 5-20 track the rest of
 * the curve much better. So after the baseline pass, every joint is
 * revisited across a few sweeps: for each joint, try a handful of candidate
 * angles, and for each candidate re-run pure pursuit for the *entire rest of
 * the chain* from that point (so downstream joints fully react to it, not
 * just get measured against their old stale angles), keeping whichever
 * candidate yields the lowest whole-chain residual. This is what lets
 * something drawn later genuinely pull earlier bends into a better shape,
 * not just leave them frozen from when less of the curve existed yet.
 *
 * TWIST still solves for the roll that lands the aim point's lateral offset
 * on the axis the next BEND can act on; BEND still pitches within the plane
 * TWIST just prepared. Any angle that would exceed a rod's configured limit
 * is clamped and recorded as a diagnostic instead of silently producing a
 * bad fit.
 */
import { Matrix4, Quaternion, Vector3 } from 'three';
import type { Assembly, Module, Rod } from '../types/module';
import type { FeasibilityReport, FitDiagnostic, FitResult, Stroke } from '../types/draw';
import { MODULE_CHAIN_LENGTH, ROD_RADIUS, SEGMENT_GAP } from '../constants/geometry';
import { FLIP_X_180, composePoses, rotateX, rotateZ, translateZ, worldPosition } from './frame';
import { rodLength } from './forwardKinematics';
import { clampRodAngle, createModule } from './factory';
import type { Pose } from '../types/module';

/** Total arc length of a polyline. */
export function strokeArcLength(points: Vector3[]): number {
  let total = 0;
  for (let i = 1; i < points.length; i += 1) {
    total += points[i]!.distanceTo(points[i - 1]!);
  }
  return total;
}

/** The point on a polyline at absolute arc length `s`, clamped to [0, totalLen]. */
function pointAtArcLength(points: Vector3[], s: number, totalLen: number): Vector3 {
  if (points.length === 0) return new Vector3();
  if (points.length === 1) return points[0]!.clone();
  const clamped = Math.max(0, Math.min(totalLen, s));
  let cumulative = 0;
  for (let i = 1; i < points.length; i += 1) {
    const segLen = points[i]!.distanceTo(points[i - 1]!);
    if (cumulative + segLen >= clamped || i === points.length - 1) {
      const t = segLen === 0 ? 0 : Math.min(1, Math.max(0, (clamped - cumulative) / segLen));
      return points[i - 1]!.clone().lerp(points[i]!, t);
    }
    cumulative += segLen;
  }
  return points[points.length - 1]!.clone();
}

/**
 * The arc-length position of the point on the polyline closest to `from`,
 * restricted to segments at or after `minS` -- found by projecting `from`
 * onto each such segment and keeping the nearest. O(points), which is
 * cheap: strokes are at most a few hundred points.
 *
 * The `minS` floor is what makes this usable for path-following rather than
 * plain nearest-neighbor: without it, a curve that loops back near its own
 * earlier self (or a chain that's drifted a bit off-course) can have its
 * *globally* closest point sitting BEHIND where the chain is actually
 * trying to go, snapping the pursuit target backward and stalling forward
 * progress entirely. Restricting the search to "at or after how far we've
 * already committed" guarantees the walk only ever moves forward along the
 * stroke, exactly like real pure-pursuit path trackers constrain their
 * search window to ahead of the vehicle, not the whole path.
 */
function closestArcLength(points: Vector3[], from: Vector3, minS: number): number {
  if (points.length === 0) return minS;
  let bestS = minS;
  let bestDistSq = Infinity;
  let cumulative = 0;
  for (let i = 1; i < points.length; i += 1) {
    const a = points[i - 1]!;
    const b = points[i]!;
    const segLen = a.distanceTo(b);
    const segEnd = cumulative + segLen;
    if (segLen > 1e-9 && segEnd >= minS) {
      const tRaw = from.clone().sub(a).dot(b.clone().sub(a)) / (segLen * segLen);
      // Also clamp the parametric position itself to not fall before minS within this segment.
      const minT = segLen > 0 ? Math.max(0, (minS - cumulative) / segLen) : 0;
      const t = Math.max(minT, Math.min(1, tRaw));
      const closest = a.clone().lerp(b, t);
      const distSq = closest.distanceToSquared(from);
      if (distSq < bestDistSq) {
        bestDistSq = distSq;
        bestS = cumulative + t * segLen;
      }
    }
    cumulative = segEnd;
  }
  return bestS;
}

/**
 * Pure-pursuit aim: from `from`, find the closest point on the stroke at or
 * after `minS`, look `lookahead` further along it, and return that target
 * plus a confidence in [0,1] for how much real stroke actually supported
 * that lookahead (see module doc comment), plus the closest-point arc
 * length itself so the caller can advance `minS` for the next joint (see
 * `closestArcLength`). Returns null only when the stroke has zero usable
 * length to project against at all.
 */
function pursuitTarget(
  points: Vector3[],
  totalLen: number,
  from: Vector3,
  lookahead: number,
  minS: number,
): { point: Vector3; confidence: number; s: number } | null {
  if (totalLen < 1e-9) return null;
  const s = closestArcLength(points, from, minS);
  const aheadAvailable = totalLen - s;
  const confidence = lookahead > 1e-9 ? Math.min(1, Math.max(0, aheadAvailable) / lookahead) : 0;
  const point = pointAtArcLength(points, s + lookahead, totalLen);
  return { point, confidence, s };
}

/** The minimal angular gap between two angles, accounting for 2π wraparound -- always in [0, π]. */
function angularDistance(a: number, b: number): number {
  const TWO_PI = Math.PI * 2;
  const diff = ((a - b + Math.PI) % TWO_PI + TWO_PI) % TWO_PI - Math.PI;
  return Math.abs(diff);
}

/** Resamples a polyline into `count` arc-length-evenly-spaced points (endpoints included). */
export function resampleByArcLength(points: Vector3[], count: number): Vector3[] {
  if (points.length === 0 || count <= 0) return [];
  if (points.length === 1 || count === 1) return Array.from({ length: count }, () => points[0]!.clone());
  const totalLen = strokeArcLength(points);
  return Array.from({ length: count }, (_, i) => pointAtArcLength(points, (i / (count - 1)) * totalLen, totalLen));
}

/** Whether the modules currently assigned to a stroke have enough reach, or how many more are needed. */
export function computeFeasibility(strokeLength: number, modulesAvailable: number): FeasibilityReport {
  const capacityLength = modulesAvailable * MODULE_CHAIN_LENGTH;
  const modulesNeeded = Math.max(1, Math.ceil(strokeLength / MODULE_CHAIN_LENGTH));
  const deficit = Math.max(0, modulesNeeded - modulesAvailable);
  return {
    strokeLength,
    capacityLength,
    modulesAvailable,
    modulesNeeded,
    deficit,
    status: deficit > 0 ? 'needs-more-modules' : 'fits',
  };
}

/** Converts a chain-forward starting frame into connector A's `Module.basePose` (see `fitChainToStroke` docs). */
export function chainStartPoseToBasePose(chainStartPose: Pose): Pose {
  return composePoses(chainStartPose, FLIP_X_180);
}

function poseQuaternion(pose: Pose): Quaternion {
  return new Quaternion(...pose.quaternion);
}

/** Rotates a world-space direction into a pose's local frame (rotation only, no translation). */
function toLocalDirection(pose: Pose, worldDir: Vector3): Vector3 {
  return worldDir.clone().applyQuaternion(poseQuaternion(pose).invert());
}

/** The world-space direction a pose's own local +Z (forward) axis currently points. */
function forwardWorldDirection(pose: Pose): Vector3 {
  return new Vector3(0, 0, 1).applyQuaternion(poseQuaternion(pose));
}

interface FlatJoint {
  module: Module;
  rod: Rod;
  rodIndex: number;
}

function flattenJoints(modulesInOrder: Module[]): FlatJoint[] {
  const flat: FlatJoint[] = [];
  for (const module of modulesInOrder) {
    module.rods.forEach((rod, rodIndex) => flat.push({ module, rod, rodIndex }));
  }
  return flat;
}

interface WalkOutcome {
  /** Final (post-clamp) angle used at each flat joint, in order. */
  angles: number[];
  /** Pre-clamp intended angle at each flat joint -- what diagnostics compare against. */
  rawAngles: number[];
  sampledPoints: Vector3[];
  sampledArcLengths: number[];
}

/**
 * Walks the whole chain start-to-end, computing each joint's angle via pure
 * pursuit -- except the joint at `overrideIndex` (if given), which is forced
 * to `overrideAngle` instead. Every other joint, before or after the
 * override, still runs its own normal pursuit logic -- joints before it are
 * causally unaffected by a later override (nothing about their own
 * computation depends on it), and joints after it fully react to wherever
 * the override left the chain, exactly as pure pursuit is designed to. This
 * single function is the shared engine behind both the baseline pass and
 * every candidate evaluation in the refinement sweeps below.
 */
function walkChain(
  flatJoints: FlatJoint[],
  strokePoints: Vector3[],
  strokeLen: number,
  chainStartPose: Pose,
  overrideIndex?: number,
  overrideAngle?: number,
): WalkOutcome {
  let framePose: Pose = chainStartPose;
  let traveled = 0;
  let progressFloor = 0; // monotonic pursuit-search floor -- see closestArcLength
  const angles: number[] = [];
  const rawAngles: number[] = [];
  const sampledPoints: Vector3[] = [worldPosition(framePose)];
  const sampledArcLengths: number[] = [0];

  flatJoints.forEach(({ rod, rodIndex }, flatIndex) => {
    framePose = composePoses(framePose, translateZ(SEGMENT_GAP));
    traveled += SEGMENT_GAP;

    let angle: number;
    let raw: number;
    if (flatIndex === overrideIndex && overrideAngle !== undefined) {
      angle = overrideAngle;
      raw = overrideAngle;
      // The override still needs to consume its share of the pursuit search floor,
      // so later joints don't see a stale (too-early) floor and risk snapping backward.
      const overrideEndPos = worldPosition(
        composePoses(framePose, rod.kind === 'twist' ? rotateZ(angle) : rotateX(angle)),
      );
      const projected = closestArcLength(strokePoints, overrideEndPos, progressFloor);
      progressFloor = Math.max(progressFloor, projected);
    } else {
      const lookahead = Math.max(rodLength(rod.kind, rodIndex), 1e-3);
      const currentPos = worldPosition(framePose);
      const pursuit = pursuitTarget(strokePoints, strokeLen, currentPos, lookahead, progressFloor);
      if (pursuit) progressFloor = pursuit.s;
      const worldDir = pursuit
        ? pursuit.point.clone().sub(currentPos).normalize()
        : forwardWorldDirection(framePose);
      const confidence = pursuit?.confidence ?? 0;
      // A degenerate zero-length aim (chain already sitting exactly on its target) has no
      // meaningful direction -- fall back to "no change" rather than an undefined normalize().
      const local = toLocalDirection(framePose, worldDir.lengthSq() > 0 ? worldDir : forwardWorldDirection(framePose));

      // rotateX(angle) sends forward (0,0,1) -> (0, -sin(angle), cos(angle)), so solving
      // for the angle that points forward along (local.y, local.z) needs the negated y term.
      // rotateZ(angle) re-expresses a fixed direction's local X in the ROTATED frame as
      // cos(angle)*x + sin(angle)*y (the frame rotates one way, so a fixed vector's local
      // coordinates rotate the other way) -- zeroing that needs the negated x term.
      //
      // Degenerate case: TWIST's whole objective is "make local.x zero". If it's already
      // (near) zero -- the common case for a curve that's already in the plane the chain is
      // bending within -- that objective is satisfied by *any* angle, and atan2(~0, y<0)
      // will happily hand back a "valid" but maximally disruptive 180deg instead of the
      // obviously-preferable 0deg (no change). Snap to 0 explicitly rather than let atan2
      // pick an arbitrary root of a degenerate equation.
      const twistNeeded = Math.abs(local.x) > 1e-6;
      const rawFullConfidence =
        rod.kind === 'twist' ? (twistNeeded ? Math.atan2(-local.x, local.y) : 0) : Math.atan2(-local.y, local.z);
      // Confidence damps the ANGLE, not the input direction: TWIST's atan2 measures the
      // *ratio* of a tiny lateral offset's components, which stays wildly sensitive even
      // once the offset's magnitude is blended down to near-nothing, so scaling the raw
      // direction doesn't actually suppress a noisy roll -- scaling the resulting angle does.
      raw = rawFullConfidence * confidence;
      angle = clampRodAngle(rod, raw);
    }

    angles.push(angle);
    rawAngles.push(raw);
    framePose = composePoses(framePose, rod.kind === 'twist' ? rotateZ(angle) : rotateX(angle));
    const length = rodLength(rod.kind, rodIndex);
    framePose = composePoses(framePose, translateZ(length));
    traveled += length;
    sampledPoints.push(worldPosition(framePose));
    sampledArcLengths.push(traveled);
  });

  return { angles, rawAngles, sampledPoints, sampledArcLengths };
}

/** Residual compares each sampled point to the stroke at the SAME distance traveled, not a
 * full-stroke resample -- a chain shorter than the stroke is graded only on the prefix it
 * actually covers, never penalized for the part it was never going to reach. */
function computeResidual(outcome: WalkOutcome, strokePoints: Vector3[], strokeLen: number) {
  let sumSq = 0;
  let maxDeviation = 0;
  outcome.sampledPoints.forEach((point, i) => {
    const target = pointAtArcLength(strokePoints, outcome.sampledArcLengths[i]!, strokeLen);
    const d = point.distanceTo(target);
    sumSq += d * d;
    maxDeviation = Math.max(maxDeviation, d);
  });
  return { residual: Math.sqrt(sumSq / outcome.sampledPoints.length), maxDeviation };
}

/** Candidate angles to try for a joint during refinement: the current angle (always kept as a
 * no-regression baseline), a "reset to zero" option, and a spread of perturbations scaled to
 * the rod's own range (TWIST's 360deg range and BEND's 180deg range need different step sizes
 * in absolute terms, so scaling by range keeps the search proportionate for both). Kept
 * deliberately small -- refinement cost scales with candidates x joints², so every extra
 * candidate here is felt directly in how long the "stroke settled" recompute takes. */
const REFINEMENT_PERTURBATION_FRACTIONS = [0.05, -0.05, 0.15, -0.15];

function buildCandidateAngles(rod: { min: number; max: number }, currentAngle: number): number[] {
  const range = rod.max - rod.min;
  const raw = [currentAngle, 0, ...REFINEMENT_PERTURBATION_FRACTIONS.map((f) => currentAngle + f * range)];
  const seen = new Set<number>();
  const unique: number[] = [];
  for (const candidate of raw) {
    const clamped = clampRodAngle(rod, candidate);
    const rounded = Math.round(clamped * 1e6) / 1e6;
    if (!seen.has(rounded)) {
      seen.add(rounded);
      unique.push(clamped);
    }
  }
  return unique;
}

/** Keeps refinement cost bounded for large assemblies -- see fitChainToStroke's doc comment. */
function refinementSweepCount(jointCount: number): number {
  if (jointCount > 60) return 1;
  return 2;
}

/**
 * Solves rod angles for `modulesInOrder` (already-connected chain, in order)
 * so the physical chain approximates `stroke`, using pure-pursuit tracking
 * plus, by default, coordinate-descent refinement (see module doc comment).
 *
 * `chainStartPose` is the *chain-forward* starting frame -- position at the
 * chain's first point with local +Z pointing along the stroke's initial
 * tangent (i.e. the direction the physical rods travel), NOT connector A's
 * outward-normal pose (`Module.basePose`), which faces the opposite way.
 * Callers applying a fit result to `Module.basePose` must convert once via
 * `composePoses(chainStartPose, FLIP_X_180)` -- see `applyFitToAssembly`.
 *
 * `refine: false` skips the refinement sweeps and returns the pure-pursuit
 * baseline only. Refinement is the whole reason earlier joints improve as
 * more of a shape gets drawn, but its cost scales with joints² -- fine for a
 * one-off "the stroke just settled" recompute, too slow to run on every
 * single pointermove of an active drag. Pass `refine: false` for that live,
 * still-being-dragged preview, and leave it on (the default) for the
 * "stroke finished" / "Re-fit" / "Apply" recompute that actually matters.
 */
export function fitChainToStroke(
  modulesInOrder: Module[],
  stroke: Stroke,
  chainStartPose: Pose,
  toleranceWorldUnits: number,
  refine = true,
): FitResult {
  const strokePoints = stroke.points.map((p) => new Vector3(...p));
  const empty: FitResult = {
    strokeId: stroke.id,
    moduleAngles: {},
    residual: 0,
    maxDeviation: 0,
    diagnostics: [],
    withinTolerance: false,
  };
  if (strokePoints.length < 2 || modulesInOrder.length === 0) return empty;

  const strokeLen = strokeArcLength(strokePoints);
  const flatJoints = flattenJoints(modulesInOrder);

  let outcome = walkChain(flatJoints, strokePoints, strokeLen, chainStartPose);
  let best = computeResidual(outcome, strokePoints, strokeLen);

  // Coordinate-descent refinement: revisit every joint across a few sweeps, letting how well
  // the WHOLE chain (including everything downstream) tracks the stroke retroactively pull
  // earlier choices toward a better shape -- see module doc comment. Skipped entirely when
  // there's barely any real stroke yet (the very start of a drag): with almost nothing to
  // measure residual against, "improvement" is just noise, and refinement can chase it into
  // a physically arbitrary angle. The baseline pursuit pass already handles that regime
  // gracefully via its own confidence damping -- refinement only has something real to offer
  // once there's an actual curve to retroactively improve against.
  const hasEnoughDataToRefine = refine && strokeLen > Math.max(rodLength('twist', 0), rodLength('bend', 0));
  for (let sweep = 0; hasEnoughDataToRefine && sweep < refinementSweepCount(flatJoints.length); sweep += 1) {
    let improvedThisSweep = false;
    for (let j = 0; j < flatJoints.length; j += 1) {
      const candidates = buildCandidateAngles(flatJoints[j]!.rod, outcome.angles[j]!);
      for (const candidate of candidates) {
        if (candidate === outcome.angles[j]) continue; // already the current baseline, nothing to gain re-testing it
        const trial = walkChain(flatJoints, strokePoints, strokeLen, chainStartPose, j, candidate);
        const trialResidual = computeResidual(trial, strokePoints, strokeLen);
        if (trialResidual.residual < best.residual - 1e-9) {
          outcome = trial;
          best = trialResidual;
          improvedThisSweep = true;
        }
      }
    }
    if (!improvedThisSweep) break; // converged -- no candidate anywhere improved this sweep
  }

  const diagnostics: FitDiagnostic[] = [];
  const anglesByModule = new Map<string, number[]>();
  flatJoints.forEach(({ module, rodIndex }, i) => {
    // `clampRodAngle` may return a coterminal (±360°) equivalent of `raw` rather than
    // clamping it -- that's the SAME physical rotation, not a distortion, so the
    // diagnostic check has to compare angles modulo 2π, not their raw numeric gap.
    if (angularDistance(outcome.rawAngles[i]!, outcome.angles[i]!) > 1e-3) {
      const kindLabel = flatJoints[i]!.rod.kind === 'twist' ? 'Twist' : 'Bend';
      diagnostics.push({
        moduleId: module.id,
        rodIndex,
        message: `${kindLabel} limit exceeded at rod ${rodIndex + 1} of module ${module.id} -- needed ${Math.round(
          (outcome.rawAngles[i]! * 180) / Math.PI,
        )}°, clamped to ${Math.round((outcome.angles[i]! * 180) / Math.PI)}°.`,
      });
    }
    const bucket = anglesByModule.get(module.id) ?? [];
    bucket.push(outcome.angles[i]!);
    anglesByModule.set(module.id, bucket);
  });
  const moduleAngles: FitResult['moduleAngles'] = {};
  for (const module of modulesInOrder) {
    moduleAngles[module.id] = anglesByModule.get(module.id) as FitResult['moduleAngles'][string];
  }

  if (findSelfIntersection(outcome.sampledPoints, outcome.sampledArcLengths)) {
    diagnostics.push({
      moduleId: modulesInOrder[0]!.id,
      rodIndex: 0,
      message:
        'This chain doubles back close enough to touch or pass through itself -- the fit is geometrically valid ' +
        'per joint limits, but the physical rods would collide. Try more modules, a gentler curve, or nudge the stroke apart at the crossing.',
    });
  }

  return {
    strokeId: stroke.id,
    moduleAngles,
    residual: best.residual,
    maxDeviation: best.maxDeviation,
    diagnostics,
    withinTolerance: diagnostics.length === 0 && best.residual <= toleranceWorldUnits,
  };
}

/**
 * Detects self-intersection: any two points on the fitted chain that are far
 * apart *along* the chain (so this doesn't flag a joint's own naturally-tight
 * neighbors) but too close *in space* for their rod bodies not to overlap.
 *
 * This is detection, not avoidance -- the solver has no notion of "steer
 * away from where I've already been", so a tight enough drawn shape can
 * still produce a colliding fit. Flagging it clearly (see the diagnostic
 * message above) is the honest thing to do instead of silently rendering a
 * tangle and calling it a successful fit. Full collision *avoidance* would
 * need an iterative/constraint-based solve (or Rapier, once physics lands
 * per the project's own phasing) -- a materially bigger undertaking than a
 * geometric fit heuristic, so it's flagged, not attempted, here.
 */
export function findSelfIntersection(points: Vector3[], arcLengths: number[]): boolean {
  const minSeparation = ROD_RADIUS * 2.2;
  const minArcGap = MODULE_CHAIN_LENGTH / 4; // skip a joint's own close neighbors, not a real crossing
  for (let i = 0; i < points.length; i += 1) {
    for (let j = i + 1; j < points.length; j += 1) {
      if (arcLengths[j]! - arcLengths[i]! < minArcGap) continue;
      if (points[i]!.distanceTo(points[j]!) < minSeparation) return true;
    }
  }
  return false;
}

/** Lookahead distance (world units) used to aim the very first frame, before any rod length exists to anchor it to. */
const START_LOOKAHEAD = 0.15;

/**
 * A default reference "up" for `buildStableFrame` when the caller doesn't
 * know the curve's plane -- world Z unless `forward` is nearly parallel to
 * it, in which case world Y (avoids a near-degenerate cross product).
 */
function pickDefaultUpReference(forward: Vector3): Vector3 {
  return Math.abs(forward.dot(new Vector3(0, 0, 1))) < 0.9 ? new Vector3(0, 0, 1) : new Vector3(0, 1, 0);
}

/**
 * Builds a frame with local +Z = `forward` and local X chosen via
 * Gram-Schmidt against `upHint` (or a sensible default), NOT via
 * `Quaternion.setFromUnitVectors`.
 *
 * This matters because BEND only rotates about local X -- a fitted chain
 * stays inside whatever plane local X is normal to, since rotating a frame
 * about its own local X axis never moves that axis. `setFromUnitVectors`
 * computes the minimal rotation from (0,0,1) to `forward`, which is correct
 * for where local Z ends up but leaves the roll *around* forward
 * uncontrolled -- for a generic forward direction, that uncontrolled roll
 * puts local X nowhere near a stroke's actual drawing-plane normal, so a
 * chain fitted to a perfectly flat stroke would immediately bend out of
 * that plane. Explicitly constructing local X from a known "up" reference
 * (the drawing plane's normal, when the caller knows it) fixes that at the
 * source: get the START frame's local X right, and it's exactly right for
 * every joint after it too.
 */
function buildStableFrame(forward: Vector3, upHint?: Vector3): Quaternion {
  const reference = upHint && upHint.lengthSq() > 1e-8 ? upHint.clone().normalize() : pickDefaultUpReference(forward);
  let localX = reference.clone().sub(forward.clone().multiplyScalar(forward.dot(reference)));
  if (localX.lengthSq() < 1e-8) {
    // forward is parallel to the reference -- fall back to a different axis for the projection.
    const fallback = Math.abs(forward.y) < 0.9 ? new Vector3(0, 1, 0) : new Vector3(1, 0, 0);
    localX = fallback.clone().sub(forward.clone().multiplyScalar(forward.dot(fallback)));
  }
  localX.normalize();
  const localY = new Vector3().crossVectors(forward, localX);
  return new Quaternion().setFromRotationMatrix(new Matrix4().makeBasis(localX, localY, forward));
}

/**
 * The natural chain-forward starting frame for a freshly drawn stroke:
 * positioned at its first point, local +Z aimed along a short lookahead
 * from that point (the same pure-pursuit logic every later joint uses --
 * this avoids the instability of aiming straight at whatever the second
 * captured point happens to be, which can be a near-degenerate, jittery
 * direction very early in a drag).
 *
 * `planeNormalHint` should be passed whenever the caller knows the stroke
 * is confined to a plane (e.g. 2D draw mode) -- it pins local X to that
 * exact normal so the whole fit stays on the plane (see `buildStableFrame`).
 * Left undefined for free 3D strokes, where no such plane exists.
 */
export function computeChainStartPoseFromStroke(stroke: Stroke, planeNormalHint?: Vector3): Pose {
  const points = stroke.points.map((p) => new Vector3(...p));
  const first = points[0];
  if (!first || points.length < 2) {
    return { position: first ? [first.x, first.y, first.z] : [0, 0, 0], quaternion: [0, 0, 0, 1] };
  }
  const totalLen = strokeArcLength(points);
  const pursuit = pursuitTarget(points, totalLen, first, START_LOOKAHEAD, 0);
  const rawDir = pursuit ? pursuit.point.clone().sub(first) : new Vector3(0, 0, 0);
  // Blending the direction itself (rather than damping an angle, as fitChainToStroke's
  // joints do) is safe here: the frame construction below is continuous in how far
  // `forward` sits from (0,0,1), so a low-confidence, mostly-forward estimate correctly
  // yields a small initial tilt rather than an unstable one.
  const forward =
    pursuit && rawDir.lengthSq() > 1e-10
      ? new Vector3(0, 0, 1).lerp(rawDir.normalize(), pursuit.confidence).normalize()
      : new Vector3(0, 0, 1);
  const quaternion = buildStableFrame(forward, planeNormalHint);
  return { position: [first.x, first.y, first.z], quaternion: [quaternion.x, quaternion.y, quaternion.z, quaternion.w] };
}

/**
 * Builds a throwaway, self-contained `Assembly` of `moduleCount` fresh
 * modules, fitted and rigidly chained to approximate `stroke`. Used both to
 * render the draw-mode preview and, index-for-index, to know what angles to
 * apply once the user commits real modules via "Apply" (see `ui/drawActions.ts`).
 */
export function buildFittedChain(
  stroke: Stroke,
  moduleCount: number,
  chainStartPose: Pose,
  toleranceWorldUnits: number,
  refine = true,
): { assembly: Assembly; fitResult: FitResult } {
  const modules = Array.from({ length: Math.max(1, moduleCount) }, () => createModule());
  const fitResult = fitChainToStroke(modules, stroke, chainStartPose, toleranceWorldUnits, refine);

  modules.forEach((module) => {
    const angles = fitResult.moduleAngles[module.id];
    if (!angles) return;
    module.rods.forEach((rod, i) => {
      rod.angle = angles[i]!;
    });
  });

  for (let i = 0; i < modules.length - 1; i += 1) {
    const a = modules[i]!.connectorB;
    const b = modules[i + 1]!.connectorA;
    a.locked = true;
    b.locked = true;
    a.connectedTo = b.id;
    b.connectedTo = a.id;
  }
  modules[0]!.basePose = chainStartPoseToBasePose(chainStartPose);

  const assembly: Assembly = {
    modules: Object.fromEntries(modules.map((m) => [m.id, m])),
    edges: modules.slice(0, -1).map((m, i) => ({ a: m.connectorB.id, b: modules[i + 1]!.connectorA.id })),
  };
  return { assembly, fitResult };
}
