/**
 * P3 -- reconfiguration planning: getting from one welded topology to another.
 *
 * Everything before this builds a shape from nothing. This is the part the
 * project is actually named for: the robot already IS something, and has to
 * become something else without falling apart on the way.
 *
 * The state is a pair -- which connectors are welded (discrete) and what the
 * joints are doing (continuous) -- and the two are coupled in both directions.
 * The topology constrains the joints, because every loop removes degrees of
 * freedom; and the joints gate the topology, because two connectors can only
 * be welded once the joints have actually brought them together. That coupling
 * is what makes this a search rather than a sequence.
 *
 * The insight that keeps it small: **the loop-closure solver already IS the
 * feasibility test.** Asking "can these two connectors be welded?" is the same
 * question as "if I declare them welded, does the resulting loop close?" -- so
 * a tentative weld plus `solveLoopClosure` answers it, and hands back the joint
 * angles that achieve it as a side effect. No separate sampler is needed.
 *
 * Scope note: this plans the ORDER of weld and unweld operations, verifying
 * each intermediate state is connected, geometrically achievable, and free of
 * self-collision. It does not yet plan the continuous motion BETWEEN those
 * states (the manifold-constrained path), nor check gravity and support. Those
 * are the remaining pieces before a plan is safe to run on hardware.
 */
import type { Assembly, ConnectorId, ModuleId } from '../types/module';
import { allConnectors } from '../types/module';
import { computeAssemblyKinematics, connectedComponents, findConnector } from './assemblyGraph';
import { isSelfColliding } from './collision';
import { solveLoopClosure } from './loopClosure';

export type WeldPair = [ConnectorId, ConnectorId];

export interface ReconfigurationStep {
  action: 'weld' | 'unweld';
  connectors: WeldPair;
  /** Joint angles once this step is complete -- what to drive the robot to. */
  angles: Record<ModuleId, number[]>;
  /** Worst remaining loop error after the step. Zero when the step made no loop. */
  loopError: number;
}

export interface ReconfigurationPlan {
  found: boolean;
  steps: ReconfigurationStep[];
  /** States expanded during the search -- the cost of finding this plan. */
  expanded: number;
  /** Why no plan exists, when `found` is false. */
  reason?: string;
}

export interface ReconfigurationOptions {
  maxExpansions?: number;
  /**
   * Allow an unweld that breaks the structure into disconnected pieces. Off by
   * default: a piece that separates mid-plan is a piece that falls on the
   * floor, unless something else is holding it.
   */
  allowSplit?: boolean;
  /**
   * Reject any state where the robot passes through itself. On by default --
   * a configuration the joints can reach is not necessarily one the robot can
   * occupy. Turn it off only to see what a plan would look like unconstrained,
   * never to make a stubborn target succeed.
   */
  avoidSelfCollision?: boolean;
  /**
   * How many differently-seeded loop solves to try before giving up on a weld.
   * Closing a loop has many solutions and only some avoid self-collision, so a
   * single solve says little about whether the weld is possible.
   */
  weldAttempts?: number;
}

/** Stable key for an undirected weld, so a pair reads the same from either side. */
export function weldKey(a: ConnectorId, b: ConnectorId): string {
  return a < b ? `${a}|${b}` : `${b}|${a}`;
}

function currentWelds(assembly: Assembly): Set<string> {
  const welds = new Set<string>();
  for (const module of Object.values(assembly.modules)) {
    for (const connector of allConnectors(module)) {
      if (connector.locked && connector.connectedTo) {
        welds.add(weldKey(connector.id, connector.connectedTo));
      }
    }
  }
  return welds;
}

function cloneAssembly(assembly: Assembly): Assembly {
  return JSON.parse(JSON.stringify(assembly)) as Assembly;
}

function snapshotAngles(assembly: Assembly): Record<ModuleId, number[]> {
  const angles: Record<ModuleId, number[]> = {};
  for (const [id, module] of Object.entries(assembly.modules)) {
    angles[id] = module.rods.map((rod) => rod.angle);
  }
  return angles;
}

/**
 * Applies a weld, then checks it is physically achievable.
 *
 * Two cases, and they differ in kind. Welding across two separate components
 * needs no solving at all -- the new weld becomes a spanning-tree edge, and one
 * component is simply carried into place relative to the other. Welding within
 * a component closes a LOOP, which is a genuine constraint: the joints must be
 * able to bring both connectors together at once, and often they cannot.
 *
 * Returns null when the loop will not close, which is a real answer about the
 * robot rather than a failure -- some welds this topology asks for cannot be
 * made from where it currently is.
 */
function tryWeld(
  assembly: Assembly,
  a: ConnectorId,
  b: ConnectorId,
  occupiable: (candidate: Assembly) => boolean,
  attempts: number,
): { assembly: Assembly; loopError: number } | null {
  const welded = cloneAssembly(assembly);
  const connA = findConnector(welded, a);
  const connB = findConnector(welded, b);
  if (!connA || !connB || connA.locked || connB.locked) return null;

  connA.locked = true;
  connB.locked = true;
  connA.connectedTo = connB.id;
  connB.connectedTo = connA.id;
  welded.edges.push({ a, b });

  // Across components: no loop, nothing to solve, geometry follows the weld.
  if (computeAssemblyKinematics(welded).cutEdges.length === 0) {
    return occupiable(welded) ? { assembly: welded, loopError: 0 } : null;
  }

  // Within a component the weld closes a loop, and closing it is not enough:
  // the solver will happily satisfy the constraint by coiling the structure
  // through itself. With dozens of joints against six constraints there is
  // enormous freedom and nothing in the residual prefers a configuration the
  // robot can physically occupy -- a four-module ring closes perfectly while
  // its big rods pass straight through each other.
  //
  // Note this canNOT be fixed by asking `solveLoopClosure` for more restarts:
  // it only restarts while it has NOT converged, so once the first descent
  // lands on a colliding solution the restarts never run and every retry
  // returns the identical answer. The starting angles have to be perturbed
  // out here, which puts each attempt in a genuinely different basin.
  let rng = 0x9e37;
  const random = () => {
    rng = (rng * 1103515245 + 12345) & 0x7fffffff;
    return rng / 0x7fffffff;
  };

  for (let attempt = 0; attempt < attempts; attempt += 1) {
    const seedAssembly = cloneAssembly(welded);
    if (attempt > 0) {
      // Widen the spread as attempts go on: near misses first, then further afield.
      const spread = attempt < attempts / 2 ? 1.5 : 3.0;
      for (const module of Object.values(seedAssembly.modules)) {
        for (const rod of module.rods) {
          rod.angle = Math.min(rod.max, Math.max(rod.min, (random() - 0.5) * spread));
        }
      }
    }

    const solved = solveLoopClosure(seedAssembly, { restarts: 0, maxIterations: 150 });
    if (!solved.report.converged) continue;

    const candidate = cloneAssembly(welded);
    for (const [moduleId, angles] of solved.angles) {
      candidate.modules[moduleId]!.rods.forEach((rod, i) => { rod.angle = angles[i]!; });
    }
    if (occupiable(candidate)) {
      return { assembly: candidate, loopError: solved.report.maxPositionError };
    }
  }
  return null;
}

/**
 * Breaks a weld. Always achievable mechanically -- an electromagnet just lets
 * go -- so the only question is whether what is left still holds together.
 */
function tryUnweld(assembly: Assembly, a: ConnectorId, b: ConnectorId, allowSplit: boolean): Assembly | null {
  const next = cloneAssembly(assembly);
  const connA = findConnector(next, a);
  const connB = findConnector(next, b);
  if (!connA || !connB || !connA.locked || !connB.locked) return null;

  // Where everything actually IS right now. A module's `basePose` only gets
  // read when it anchors its own component, so a module that has been carried
  // along through welds is still carrying whatever stale pose it was created
  // with. Break the weld without fixing that and the freed piece teleports back
  // to that stale pose -- typically straight through whatever it just left.
  const before = computeAssemblyKinematics(next).transforms;

  connA.locked = false;
  connB.locked = false;
  connA.connectedTo = null;
  connB.connectedTo = null;
  next.edges = next.edges.filter((e) => weldKey(e.a, e.b) !== weldKey(a, b));

  for (const module of Object.values(next.modules)) {
    const pose = before.get(module.id)?.connectorA;
    if (pose) module.basePose = pose;
  }

  if (!allowSplit && connectedComponents(next).length > connectedComponents(assembly).length) {
    return null;
  }
  return next;
}

interface SearchState {
  assembly: Assembly;
  /** Which of the planned removals/additions are already done. */
  removed: Set<string>;
  added: Set<string>;
  steps: ReconfigurationStep[];
}

function stateKey(state: SearchState): string {
  return `${[...state.removed].sort().join(',')}#${[...state.added].sort().join(',')}`;
}

/**
 * Plans an order of weld and unweld operations that turns `start`'s topology
 * into `targetWelds`, verifying every intermediate state along the way.
 *
 * The target is expressed over the SAME modules that already exist -- this
 * answers "rewire this robot into that shape", not "assign these modules to an
 * arbitrary shape". Deciding which physical module should play which role in a
 * target is a separate assignment problem, and a harder one; keeping it out
 * here means every step of this plan refers to real connectors that really
 * exist.
 *
 * Search is best-first on how many operations remain. Both the operations that
 * must happen and their count are known up front -- only the ORDER is in
 * question -- and order matters because an unweld can strand a piece and a
 * weld can be geometrically impossible until other welds have released first.
 */
export function planReconfiguration(
  start: Assembly,
  targetWelds: WeldPair[],
  options: ReconfigurationOptions = {},
): ReconfigurationPlan {
  const maxExpansions = options.maxExpansions ?? 400;
  const allowSplit = options.allowSplit ?? false;
  const avoidSelfCollision = options.avoidSelfCollision ?? true;
  const weldAttempts = options.weldAttempts ?? 25;
  /** A state is only usable if the robot can actually occupy it. */
  const occupiable = (candidate: Assembly) => !avoidSelfCollision || !isSelfColliding(candidate);

  const startWelds = currentWelds(start);
  const targetSet = new Set(targetWelds.map(([a, b]) => weldKey(a, b)));
  const pairOf = new Map<string, WeldPair>();
  for (const [a, b] of targetWelds) pairOf.set(weldKey(a, b), [a, b]);
  for (const module of Object.values(start.modules)) {
    for (const connector of allConnectors(module)) {
      if (connector.locked && connector.connectedTo) {
        pairOf.set(weldKey(connector.id, connector.connectedTo), [connector.id, connector.connectedTo]);
      }
    }
  }

  const toRemove = [...startWelds].filter((k) => !targetSet.has(k));
  const toAdd = [...targetSet].filter((k) => !startWelds.has(k));

  if (toRemove.length === 0 && toAdd.length === 0) {
    return { found: true, steps: [], expanded: 0 };
  }

  const initial: SearchState = {
    assembly: cloneAssembly(start),
    removed: new Set(),
    added: new Set(),
    steps: [],
  };

  const remaining = (s: SearchState) =>
    toRemove.length - s.removed.size + (toAdd.length - s.added.size);

  const frontier: SearchState[] = [initial];
  const visited = new Set<string>([stateKey(initial)]);
  let expanded = 0;
  let deepest = 0;

  while (frontier.length > 0 && expanded < maxExpansions) {
    // Cheap priority queue: the frontier is small enough that scanning for the
    // best state beats maintaining a heap.
    let bestIndex = 0;
    for (let i = 1; i < frontier.length; i += 1) {
      if (remaining(frontier[i]!) < remaining(frontier[bestIndex]!)) bestIndex = i;
    }
    const state = frontier.splice(bestIndex, 1)[0]!;
    expanded += 1;
    deepest = Math.max(deepest, state.steps.length);

    if (state.removed.size === toRemove.length && state.added.size === toAdd.length) {
      return { found: true, steps: state.steps, expanded };
    }

    // Unwelds first: they free degrees of freedom, which is usually what makes
    // a later weld reachable at all.
    for (const key of toRemove) {
      if (state.removed.has(key)) continue;
      const pair = pairOf.get(key)!;
      // No collision check here, deliberately. Releasing a weld moves nothing,
      // so it cannot introduce a new overlap -- but it does drop the exclusion
      // that excused the two connectors for being coincident, which they still
      // are the instant after release. Checking would reject every unweld for
      // an overlap that was legal one moment earlier. Separating the freed
      // pieces is a motion, and belongs to the motion planner.
      const next = tryUnweld(state.assembly, pair[0], pair[1], allowSplit);
      if (!next) continue;
      const child: SearchState = {
        assembly: next,
        removed: new Set([...state.removed, key]),
        added: state.added,
        steps: [...state.steps, {
          action: 'unweld',
          connectors: pair,
          angles: snapshotAngles(next),
          loopError: 0,
        }],
      };
      const childKey = stateKey(child);
      if (visited.has(childKey)) continue;
      visited.add(childKey);
      frontier.push(child);
    }

    for (const key of toAdd) {
      if (state.added.has(key)) continue;
      const pair = pairOf.get(key)!;
      const welded = tryWeld(state.assembly, pair[0], pair[1], occupiable, weldAttempts);
      if (!welded) continue;
      const child: SearchState = {
        assembly: welded.assembly,
        removed: state.removed,
        added: new Set([...state.added, key]),
        steps: [...state.steps, {
          action: 'weld',
          connectors: pair,
          angles: snapshotAngles(welded.assembly),
          loopError: welded.loopError,
        }],
      };
      const childKey = stateKey(child);
      if (visited.has(childKey)) continue;
      visited.add(childKey);
      frontier.push(child);
    }
  }

  return {
    found: false,
    steps: [],
    expanded,
    reason:
      expanded >= maxExpansions
        ? `Gave up after ${expanded} states (deepest ${deepest} of ${toRemove.length + toAdd.length} operations).`
        : `No valid ordering exists: got ${deepest} of ${toRemove.length + toAdd.length} operations in before every option was blocked.`,
  };
}
