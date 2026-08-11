/**
 * Loop closure: makes a weld graph with cycles physically consistent.
 *
 * The spanning-tree traversal in `assemblyGraph.ts` places every module by
 * walking outward from an anchor, which works perfectly while the weld graph
 * is a tree. The moment it contains a cycle, one weld per cycle is left over
 * (a "cut edge"): the traversal never uses it, so nothing makes its two
 * connectors actually meet. They can end up arbitrarily far apart while both
 * still reporting `locked: true` -- the structure claims a loop it does not
 * physically have.
 *
 * This module closes that gap the way multibody dynamics does it: keep the
 * tree propagation, and treat each cut edge as a constraint to be solved.
 * For each one we measure a 6-vector residual between where the weld says the
 * connector should be and where the tree actually put it, then search joint
 * space for angles that drive every residual to zero.
 *
 * The search is Levenberg-Marquardt -- Gauss-Newton with adaptive damping.
 * Plain Gauss-Newton diverges here in practice: a robot arm folded into a
 * closed loop sits near kinematic singularities constantly, where the Jacobian
 * loses rank and the undamped step explodes. Damping is what makes it behave.
 *
 * Two outputs matter as much as the joint angles themselves:
 *  - Mobility: the Jacobian's numerical rank gives the closed structure's true
 *    degrees of freedom, usually far fewer than 6 per module.
 *  - Infeasibility: some loops cannot be closed at all within the joints'
 *    limits. That is a real, useful answer about the target shape -- not an
 *    error -- so it is reported rather than thrown.
 */
import { Quaternion, Vector3 } from 'three';
import type { Assembly, ConnectorEnd, ModuleId, Pose } from '../types/module';
import { clampRodAngle } from './factory';
import { computeAssemblyKinematics, connectorPose, weldedPartnerPose } from './assemblyGraph';

/** Rods per module -- the joint vector is this many numbers per module. */
const RODS_PER_MODULE = 6;

export interface LoopClosureOptions {
  /** Stop once the residual norm falls below this (world units / radians). */
  tolerance?: number;
  /** Cap on descent steps per attempt, not across all attempts. */
  maxIterations?: number;
  /** Finite-difference step for the numerical Jacobian, radians. */
  jacobianStep?: number;
  /**
   * Extra descents from random starting configurations, tried only while the
   * best result so far has not converged. Loop closure is non-convex, so a
   * single descent lands in a local minimum often enough that this is not
   * optional.
   */
  restarts?: number;
  /** Seeds the restart RNG. Fixed by default, so solving is deterministic. */
  seed?: number;
}

interface Attempt {
  q: number[];
  residual: number[];
  jacobian: number[][];
  iterations: number;
}

/** Small deterministic PRNG -- restarts must be reproducible for tests to mean anything. */
function mulberry32(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export interface LoopClosureReport {
  /** Independent kinematic loops -- one per cut edge. Zero means the graph is a tree. */
  loopCount: number;
  /** Constraint rows solved: 6 per loop. */
  constraintCount: number;
  /** Joint angles that were free to move. */
  jointCount: number;
  /**
   * Degrees of freedom left after the loops take their toll, from the
   * Jacobian's numerical rank. A structure at 0 is rigid; negative never
   * happens (rank is bounded by the joint count) but a value far below what
   * the joints suggest means heavily over-constrained.
   */
  mobility: number;
  /** Worst single cut-edge error at the returned angles: position and rotation. */
  maxPositionError: number;
  maxRotationError: number;
  /**
   * Worst distance from a goal's target to where its connector ended up. Zero
   * when there were no goals. Independent of `converged`, which reports only
   * on the loops -- a solve can hold every weld perfectly and still be unable
   * to reach the target, which is exactly the case worth distinguishing.
   */
  goalError: number;
  residualNorm: number;
  iterations: number;
  /** True when every loop closed inside `tolerance`. */
  converged: boolean;
  /**
   * Set when the solver stalled with residual left over -- the loops cannot be
   * closed with these joint limits. The shape is unreachable, not the code broken.
   */
  overConstrained: boolean;
}

export interface LoopClosureResult {
  /** Solved angles per module, in rod order. Apply these to make the loops real. */
  angles: Map<ModuleId, number[]>;
  report: LoopClosureReport;
}

/**
 * Rotation error as a rotation vector (axis * angle), which is zero exactly
 * when the two orientations match.
 *
 * This is deliberately NOT the true se(3) logarithm -- that couples the
 * rotational and translational parts through an extra matrix factor. For
 * root-finding, all that matters is that the residual is smooth and vanishes
 * only at the solution, and stacking [position error, rotation vector] gives
 * that with far less machinery. It is the same error used by most IK solvers.
 */
function rotationError(a: Quaternion, b: Quaternion): Vector3 {
  // Relative rotation from a to b.
  const relative = a.clone().invert().multiply(b);
  // Canonicalise sign: q and -q are the same rotation, but only one of them
  // gives the short way round. Without this the solver can chase a 2*pi turn.
  if (relative.w < 0) {
    relative.set(-relative.x, -relative.y, -relative.z, -relative.w);
  }
  const vecLength = Math.hypot(relative.x, relative.y, relative.z);
  if (vecLength < 1e-12) return new Vector3(0, 0, 0);
  const angle = 2 * Math.atan2(vecLength, relative.w);
  return new Vector3(relative.x, relative.y, relative.z).multiplyScalar(angle / vecLength);
}

/** Modules in stable order -- fixes which joint owns which column of the Jacobian. */
function orderedModules(assembly: Assembly): ModuleId[] {
  return Object.keys(assembly.modules);
}

function readAngles(assembly: Assembly, order: readonly ModuleId[]): number[] {
  const q: number[] = [];
  for (const id of order) {
    for (const rod of assembly.modules[id]!.rods) q.push(rod.angle);
  }
  return q;
}

/** Writes a joint vector back into a working copy, clamping each rod to its own limits. */
function applyAngles(assembly: Assembly, order: readonly ModuleId[], q: number[]): void {
  order.forEach((id, moduleIndex) => {
    assembly.modules[id]!.rods.forEach((rod, rodIndex) => {
      rod.angle = clampRodAngle(rod, q[moduleIndex * RODS_PER_MODULE + rodIndex]!);
    });
  });
}

/**
 * A pose the caller wants one of the assembly's connectors to reach -- the
 * "move this here" half of constrained IK. Loops are still respected; a goal
 * that fights them loses (see `LOOP_WEIGHT`).
 */
export interface ConnectorGoal {
  moduleId: ModuleId;
  end: ConnectorEnd;
  target: Pose;
  /**
   * Match position only and let the connector point wherever it likes. Right
   * for a translate-only drag, where demanding an orientation the user never
   * asked for just makes the goal harder to reach for no benefit.
   */
  positionOnly?: boolean;
  /** Relative importance among goals. Loops always outrank all of them. */
  weight?: number;
}

/**
 * How much more a loop-closure row counts than a goal row.
 *
 * Least squares trades every residual off against every other, so with equal
 * weights an unreachable goal would be met halfway by pulling a weld apart --
 * the solver would cheerfully break the robot to get closer to the target.
 * Weighting loops far higher makes them behave as hard constraints while
 * staying in one unconstrained least-squares problem: a goal that cannot be
 * reached ends up missed, which is the honest outcome, rather than reached by
 * cheating.
 */
const LOOP_WEIGHT = 100;

/**
 * The stacked residual: 6 rows per cut edge, then 3 or 6 rows per goal, each
 * ordered [position error xyz, rotation error xyz].
 *
 * A weld means the two connectors sit at the same point with opposed outward
 * normals, which `weldedPartnerPose` expresses exactly. So the target for one
 * side is the welded-partner pose of the other, and the residual is the
 * difference between that target and where the tree traversal actually landed.
 */
function computeResidual(assembly: Assembly, goals: readonly ConnectorGoal[] = []): number[] {
  const { transforms, cutEdges } = computeAssemblyKinematics(assembly);
  const residual: number[] = [];

  for (const edge of cutEdges) {
    const poseA = connectorPose(transforms.get(edge.a.moduleId)!, edge.a.end);
    const poseB = connectorPose(transforms.get(edge.b.moduleId)!, edge.b.end);
    const target = weldedPartnerPose(poseA);

    residual.push(
      (target.position[0] - poseB.position[0]) * LOOP_WEIGHT,
      (target.position[1] - poseB.position[1]) * LOOP_WEIGHT,
      (target.position[2] - poseB.position[2]) * LOOP_WEIGHT,
    );
    const error = rotationError(new Quaternion(...poseB.quaternion), new Quaternion(...target.quaternion));
    residual.push(error.x * LOOP_WEIGHT, error.y * LOOP_WEIGHT, error.z * LOOP_WEIGHT);
  }

  for (const goal of goals) {
    const moduleTransforms = transforms.get(goal.moduleId);
    if (!moduleTransforms) continue;
    const actual = connectorPose(moduleTransforms, goal.end);
    const weight = goal.weight ?? 1;

    residual.push(
      (goal.target.position[0] - actual.position[0]) * weight,
      (goal.target.position[1] - actual.position[1]) * weight,
      (goal.target.position[2] - actual.position[2]) * weight,
    );
    if (goal.positionOnly) continue;
    const error = rotationError(new Quaternion(...actual.quaternion), new Quaternion(...goal.target.quaternion));
    residual.push(error.x * weight, error.y * weight, error.z * weight);
  }

  return residual;
}

function norm(v: readonly number[]): number {
  let sum = 0;
  for (const x of v) sum += x * x;
  return Math.sqrt(sum);
}

/**
 * Solves `(A + lambda * diag(A)) x = b` by Gaussian elimination with partial
 * pivoting. The systems here are small -- one column per joint -- so a dense
 * direct solve is far simpler than anything iterative and never the bottleneck.
 * Returns null if the matrix is numerically singular even after damping, which
 * the caller handles by increasing damping and retrying.
 */
function solveDamped(ata: number[][], atb: number[], lambda: number): number[] | null {
  const n = atb.length;
  const m = ata.map((row, i) => {
    const copy = row.slice();
    copy[i] = copy[i]! * (1 + lambda) + lambda * 1e-9;
    return copy;
  });
  const rhs = atb.slice();

  for (let col = 0; col < n; col += 1) {
    let pivot = col;
    for (let row = col + 1; row < n; row += 1) {
      if (Math.abs(m[row]![col]!) > Math.abs(m[pivot]![col]!)) pivot = row;
    }
    if (Math.abs(m[pivot]![col]!) < 1e-14) return null;
    if (pivot !== col) {
      [m[col], m[pivot]] = [m[pivot]!, m[col]!];
      [rhs[col], rhs[pivot]] = [rhs[pivot]!, rhs[col]!];
    }
    const diagonal = m[col]![col]!;
    for (let row = col + 1; row < n; row += 1) {
      const factor = m[row]![col]! / diagonal;
      if (factor === 0) continue;
      for (let k = col; k < n; k += 1) m[row]![k] = m[row]![k]! - factor * m[col]![k]!;
      rhs[row] = rhs[row]! - factor * rhs[col]!;
    }
  }

  const x = new Array<number>(n).fill(0);
  for (let row = n - 1; row >= 0; row -= 1) {
    let sum = rhs[row]!;
    for (let k = row + 1; k < n; k += 1) sum -= m[row]![k]! * x[k]!;
    x[row] = sum / m[row]![row]!;
  }
  return x;
}

/**
 * Numerical rank by Gram-Schmidt: count how many Jacobian rows are linearly
 * independent of the ones before them. Enough to report mobility honestly
 * without pulling in a full SVD.
 *
 * The cutoff is RELATIVE to the largest row. An absolute one silently reports
 * a different rank whenever the residual is rescaled -- weighting the loop
 * rows by LOOP_WEIGHT multiplies every entry by 100, which lifts genuinely
 * null directions above a fixed threshold and inflates the reported rank.
 */
function numericalRank(rows: readonly (readonly number[])[], relativeTolerance = 1e-8): number {
  let largest = 0;
  for (const row of rows) largest = Math.max(largest, norm(row));
  if (largest === 0) return 0;
  const tolerance = largest * relativeTolerance;

  const basis: number[][] = [];
  for (const row of rows) {
    const residual = row.slice();
    for (const b of basis) {
      let dot = 0;
      for (let i = 0; i < residual.length; i += 1) dot += residual[i]! * b[i]!;
      for (let i = 0; i < residual.length; i += 1) residual[i] = residual[i]! - dot * b[i]!;
    }
    const length = norm(residual);
    if (length > tolerance) {
      basis.push(residual.map((x) => x / length));
    }
  }
  return basis.length;
}

/** Deep-copies an assembly so the solver can probe joint angles without touching the caller's state. */
function cloneAssembly(assembly: Assembly): Assembly {
  return JSON.parse(JSON.stringify(assembly)) as Assembly;
}

export interface LoopError {
  loopCount: number;
  /** Worst gap across the loop-closing welds, at the CURRENT angles. */
  maxPositionError: number;
  maxRotationError: number;
  /**
   * Grubler-Kutzbach count: joints minus 6 per rigid-weld loop. Cheap, and
   * exact for a well-behaved mechanism -- but it is only an estimate. An
   * over-constrained structure has redundant constraints, so its real mobility
   * is higher than this predicts; `solveLoopClosure` reports the true figure
   * from the Jacobian's rank.
   */
  nominalMobility: number;
}

/**
 * Measures how badly the loops are currently violated, WITHOUT solving.
 *
 * One forward-kinematics pass, so it is cheap enough to call on every render --
 * unlike `solveLoopClosure`, which runs a full Levenberg-Marquardt descent and
 * is far too slow for that. Use this to decide whether solving is needed, and
 * to show the user how consistent their structure currently is.
 */
export function measureLoopError(assembly: Assembly): LoopError {
  const { cutEdges } = computeAssemblyKinematics(assembly);
  const jointCount = Object.keys(assembly.modules).length * RODS_PER_MODULE;

  if (cutEdges.length === 0) {
    return { loopCount: 0, maxPositionError: 0, maxRotationError: 0, nominalMobility: jointCount };
  }

  // Loop rows come back scaled by LOOP_WEIGHT; divide it back out so the
  // numbers reported are in real world units.
  const residual = computeResidual(assembly);
  let maxPositionError = 0;
  let maxRotationError = 0;
  for (let i = 0; i < cutEdges.length * 6; i += 6) {
    maxPositionError = Math.max(
      maxPositionError,
      Math.hypot(residual[i]!, residual[i + 1]!, residual[i + 2]!) / LOOP_WEIGHT,
    );
    maxRotationError = Math.max(
      maxRotationError,
      Math.hypot(residual[i + 3]!, residual[i + 4]!, residual[i + 5]!) / LOOP_WEIGHT,
    );
  }

  return {
    loopCount: cutEdges.length,
    maxPositionError,
    maxRotationError,
    nominalMobility: Math.max(0, jointCount - 6 * cutEdges.length),
  };
}

/** Worst distance between a goal's target and where its connector actually sits. */
function maxGoalError(assembly: Assembly, goals: readonly ConnectorGoal[]): number {
  if (goals.length === 0) return 0;
  const { transforms } = computeAssemblyKinematics(assembly);
  let worst = 0;
  for (const goal of goals) {
    const moduleTransforms = transforms.get(goal.moduleId);
    if (!moduleTransforms) continue;
    const actual = connectorPose(moduleTransforms, goal.end);
    worst = Math.max(
      worst,
      Math.hypot(
        goal.target.position[0] - actual.position[0],
        goal.target.position[1] - actual.position[1],
        goal.target.position[2] - actual.position[2],
      ),
    );
  }
  return worst;
}

/**
 * Finds joint angles that close every kinematic loop in `assembly`.
 *
 * Does not mutate the input. Returns the solved angles plus a report; apply the
 * angles yourself (or ignore them and act on `report.overConstrained`).
 * An assembly whose weld graph is already a tree returns immediately with its
 * angles untouched -- there is nothing to solve.
 */
export function solveLoopClosure(
  assembly: Assembly,
  options: LoopClosureOptions = {},
): LoopClosureResult {
  return solveConstrained(assembly, [], options);
}

/**
 * Constrained inverse kinematics: drives `goals` toward their target poses
 * while holding every kinematic loop closed and every joint inside its limits.
 *
 * Same machinery as `solveLoopClosure` -- the goals simply add rows to the
 * residual it already assembles. Loops outrank goals (see `LOOP_WEIGHT`), so
 * an unreachable target is missed rather than met by pulling a weld apart, and
 * `report.converged` continues to mean "the loops hold", with `goalError`
 * saying separately how close the targets came.
 *
 * For interactive dragging, pass `restarts: 0` and a small `maxIterations`:
 * the solve is seeded from the assembly's current angles, so successive frames
 * warm-start from the previous answer and converge across the drag. Restarts
 * are actively unwanted there -- they would jump to a different branch of the
 * solution and make the structure visibly teleport mid-drag.
 */
export function solveConstrained(
  assembly: Assembly,
  goals: readonly ConnectorGoal[],
  options: LoopClosureOptions = {},
): LoopClosureResult {
  const tolerance = options.tolerance ?? 1e-5;
  const maxIterations = options.maxIterations ?? 120;
  const step = options.jacobianStep ?? 1e-5;
  const restarts = options.restarts ?? 8;

  const order = orderedModules(assembly);
  const working = cloneAssembly(assembly);
  const { cutEdges } = computeAssemblyKinematics(working);

  const emptyAngles = () =>
    new Map(order.map((id) => [id, working.modules[id]!.rods.map((r) => r.angle)]));

  // Nothing to solve: a tree with no goals is already consistent by construction.
  if (cutEdges.length === 0 && goals.length === 0) {
    return {
      angles: emptyAngles(),
      report: {
        loopCount: 0,
        constraintCount: 0,
        jointCount: order.length * RODS_PER_MODULE,
        mobility: order.length * RODS_PER_MODULE,
        maxPositionError: 0,
        maxRotationError: 0,
        goalError: 0,
        residualNorm: 0,
        iterations: 0,
        converged: true,
        overConstrained: false,
      },
    };
  }

  /** One Levenberg-Marquardt descent from a given starting joint vector. */
  function descendFrom(startQ: number[]): Attempt {
  let q = startQ.slice();
  applyAngles(working, order, q);
  q = readAngles(working, order);
  let residual = computeResidual(working, goals);
  let lambda = 1e-3;
  let iterations = 0;
  let jacobian: number[][] = [];

  while (iterations < maxIterations && norm(residual) > tolerance) {
    iterations += 1;

    // Jacobian by forward differences: one extra forward-kinematics pass per
    // joint. Analytic derivatives would be faster, but this is exact enough to
    // converge and impossible to get subtly wrong.
    jacobian = [];
    for (let j = 0; j < q.length; j += 1) {
      const probe = q.slice();
      probe[j] = probe[j]! + step;
      applyAngles(working, order, probe);
      const perturbed = computeResidual(working, goals);
      jacobian.push(perturbed.map((value, i) => (value - residual[i]!) / step));
    }
    applyAngles(working, order, q);

    // Normal equations. `jacobian` is stored column-major (one entry per joint),
    // so these loops read as J^T J and J^T r directly.
    const n = q.length;
    const ata: number[][] = Array.from({ length: n }, () => new Array<number>(n).fill(0));
    const atb = new Array<number>(n).fill(0);
    for (let a = 0; a < n; a += 1) {
      for (let b = a; b < n; b += 1) {
        let sum = 0;
        for (let i = 0; i < residual.length; i += 1) sum += jacobian[a]![i]! * jacobian[b]![i]!;
        ata[a]![b] = sum;
        ata[b]![a] = sum;
      }
      let rhs = 0;
      for (let i = 0; i < residual.length; i += 1) rhs += jacobian[a]![i]! * residual[i]!;
      // Negated: Gauss-Newton wants delta = -(J^T J)^-1 J^T r. Solving with +J^T r
      // walks uphill, which the acceptance test below then rejects forever --
      // the solver stalls on its first iteration and reports a false
      // "over-constrained".
      atb[a] = -rhs;
    }

    // Adaptive damping: accept a step that reduces the residual and trust the
    // linearisation more next time; otherwise damp harder and try again.
    let accepted = false;
    for (let attempt = 0; attempt < 8 && !accepted; attempt += 1) {
      const delta = solveDamped(ata, atb, lambda);
      if (!delta) {
        lambda *= 10;
        continue;
      }
      const candidate = q.map((value, i) => value + delta[i]!);
      applyAngles(working, order, candidate);
      // Read back through the clamp -- a step that limits would otherwise be
      // scored on angles the robot cannot actually reach.
      const clamped = readAngles(working, order);
      const candidateResidual = computeResidual(working, goals);

      if (norm(candidateResidual) < norm(residual)) {
        q = clamped;
        residual = candidateResidual;
        lambda = Math.max(lambda * 0.3, 1e-9);
        accepted = true;
      } else {
        lambda *= 10;
      }
    }

    applyAngles(working, order, q);
    if (!accepted) break; // damping could not find a downhill step -- stalled
  }

    return { q, residual, jacobian, iterations };
  }

  // Closing a loop is a non-convex root-finding problem, so a single descent
  // can and does land in local minima -- most characteristically at a rotation
  // error of exactly pi, where the error's axis is undefined and the gradient
  // vanishes. Multi-start is the standard remedy: keep the best of several
  // descents, the first from the caller's angles and the rest from random
  // configurations. The RNG is seeded so results stay reproducible.
  const random = mulberry32(options.seed ?? 0x5eed);
  const randomStart = (): number[] =>
    order.flatMap((id) =>
      working.modules[id]!.rods.map((rod) => rod.min + random() * (rod.max - rod.min)),
    );

  let best = descendFrom(readAngles(working, order));
  let totalIterations = best.iterations;
  for (let attempt = 0; attempt < restarts && norm(best.residual) > tolerance; attempt += 1) {
    const candidate = descendFrom(randomStart());
    totalIterations += candidate.iterations;
    if (norm(candidate.residual) < norm(best.residual)) best = candidate;
  }

  const { q, residual, jacobian } = best;
  applyAngles(working, order, q);

  // Report in world units, measured off the solved assembly. Reading these
  // back out of `residual` would report LOOP_WEIGHT-scaled numbers and mix the
  // goal rows in with the loop rows.
  const measured = measureLoopError(working);
  const goalError = maxGoalError(working, goals);

  const residualNorm = norm(residual);
  // "Converged" is about the loops holding, not about the goals being met --
  // an out-of-reach target is a legitimate outcome, a broken weld is not.
  const converged = measured.maxPositionError <= tolerance && measured.maxRotationError <= tolerance;
  const jointCount = q.length;
  const rank = jacobian.length > 0 ? numericalRank(jacobian) : 0;
  const iterations = totalIterations;

  return {
    angles: new Map(order.map((id) => [id, working.modules[id]!.rods.map((r) => r.angle)])),
    report: {
      loopCount: cutEdges.length,
      constraintCount: cutEdges.length * 6,
      jointCount,
      mobility: Math.max(0, jointCount - rank),
      maxPositionError: measured.maxPositionError,
      maxRotationError: measured.maxRotationError,
      goalError,
      residualNorm,
      iterations,
      converged,
      overConstrained: cutEdges.length > 0 && !converged,
    },
  };
}

/**
 * Convenience wrapper: returns a copy of `assembly` with loop-closing angles
 * applied, alongside the report. Leaves the original untouched.
 */
export function withLoopsClosed(
  assembly: Assembly,
  options?: LoopClosureOptions,
): { assembly: Assembly; report: LoopClosureReport } {
  const { angles, report } = solveLoopClosure(assembly, options);
  const next = cloneAssembly(assembly);
  for (const [moduleId, rodAngles] of angles) {
    next.modules[moduleId]!.rods.forEach((rod, i) => {
      rod.angle = rodAngles[i]!;
    });
  }
  return { assembly: next, report };
}
