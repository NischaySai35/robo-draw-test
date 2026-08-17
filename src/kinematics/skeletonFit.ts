/**
 * P2 -- shape synthesis: turns a target skeleton into a real welded assembly.
 *
 * A skeleton is just a graph of points in space: nodes and the straight beams
 * between them. That is the natural way to describe what this robot should
 * become, because a module IS an edge -- a long chain with a connector at each
 * end -- so "what shape do I want" and "which modules go where" are the same
 * question asked twice.
 *
 * It is also a good interface for a language model to write. "A chair" is
 * exactly the kind of thing a model can turn into four legs, a seat frame and
 * a back; what a model cannot do is decide whether the result closes, which is
 * why the spec it emits is fed to a solver that proves or refutes it rather
 * than trusted.
 *
 * The algorithm, in order:
 *  1. Split the skeleton into maximal paths, breaking at every junction. A
 *     run of beams passing straight through degree-2 nodes becomes ONE chain
 *     sized for the whole run, not one chain per beam.
 *  2. Order those paths so each is built only after something it touches, so
 *     it has a real connector to weld onto instead of being placed blind.
 *  3. Fit each path, anchoring branches onto free connectors of already-placed
 *     chains -- usually the ones riding a big rod, which is what lets three or
 *     more beams meet at one junction at all.
 *  4. Close the loops AND pull the whole structure back onto the skeleton.
 *     A shape with cycles (a seat frame, a ladder) leaves welds the spanning
 *     tree cannot satisfy, and `solveConstrained` closes them -- but it is
 *     also given every skeleton node as a goal for the chain tip that should
 *     reach it.
 *
 * Step 4's goals are what make this a fit of the whole shape rather than a
 * sequence of independent guesses. Steps 1-3 are greedy and one-directional:
 * each chain is fitted once, starting from a connector that already sits off
 * the junction it stands for, and is never reconsidered. Error therefore does
 * not merely persist, it compounds outward -- each branch inherits its host's
 * offset and adds its own. Nothing in steps 1-3 ever looks at the assembled
 * result, which is why a generated shape could come out visibly wrong while
 * every individual chain honestly reported a small residual: each residual
 * was measured against that chain's own drifted starting point, not against
 * the design. Step 4 is the only pass that sees the whole structure at once.
 */
import { Vector3 } from 'three';
import type { Assembly, Connector, ConnectorEnd, ModuleId, Pose } from '../types/module';
import { connectorByEnd } from '../types/module';
import type { Stroke } from '../types/draw';
import {
  buildFittedChain,
  computeChainStartPoseFromStroke,
  idealModuleCount,
  strokeArcLength,
} from './curveFit';
import { anchorKeyId, findWeldAnchor } from './branchAnchor';
import { solveConstrained, type ConnectorGoal, type LoopClosureReport } from './loopClosure';

export interface SkeletonNode {
  id: string;
  position: [number, number, number];
}

export interface SkeletonEdge {
  from: string;
  to: string;
}

/**
 * A target shape as a graph of beams. This is the format a language model
 * should emit -- small, structured, and impossible to express an ambiguous
 * shape in.
 */
export interface ShapeSpec {
  name: string;
  nodes: SkeletonNode[];
  edges: SkeletonEdge[];
}

export interface ChainSummary {
  /** Skeleton nodes this chain runs through, in order. */
  nodeIds: string[];
  moduleIds: ModuleId[];
  /** How far the fitted chain strays from the beam it represents. */
  residual: number;
  /** The connector this chain welded onto, when it branched off another. */
  anchoredTo: { moduleId: ModuleId; end: ConnectorEnd } | null;
}

export interface SkeletonFitResult {
  assembly: Assembly;
  chains: ChainSummary[];
  /** Junctions where a branch found no free connector and was left unwelded. */
  unanchored: string[];
  /** Null when the shape has no cycles. */
  loopReport: LoopClosureReport | null;
  moduleCount: number;
  /** Independent cycles in the SKELETON -- what the loop solver had to close. */
  loopCount: number;
}

export interface SkeletonFitOptions {
  /** Fit tolerance passed to the curve fitter, in world units. */
  tolerance?: number;
  /** Restarts for the final loop-closure solve. */
  loopRestarts?: number;
  /** Descent steps for the weld-polishing solve -- see `POLISH_ITERATIONS`. */
  loopIterations?: number;
  /** Descent steps for the shaping solve -- see `SHAPE_ITERATIONS`. */
  shapeIterations?: number;
}

/**
 * Descent steps for the two closing solves (see the end of `fitSkeleton`).
 *
 * Both are capped low on purpose. Neither solve can ever reach the solver's
 * own stopping tolerance, because the goal rows keep the residual norm well
 * above it no matter how good the answer gets -- a goal that is as close as
 * the structure allows still contributes. So without a cap, every solve runs
 * its full budget every time.
 *
 * These numbers are where measured quality stops improving: raising the shape
 * pass from 10 to 150 moved the mean tip error of a generated chair from
 * 0.808 to 0.806 and took 3x as long; the polish likewise flattens by 40,
 * holding welds at ~1e-7 against a 1e-5 tolerance.
 */
const SHAPE_ITERATIONS = 10;
const POLISH_ITERATIONS = 40;

/** Extra polish rounds allowed, each double the last -- see the escalation loop. */
const POLISH_MAX_ESCALATIONS = 2;
/**
 * A polish round must cut the worst weld error to at least this fraction of
 * what it was, or escalating again is judged not worth the time. Loose on
 * purpose: the point is to tell "still descending" apart from "stalled", not
 * to demand a particular rate.
 */
const POLISH_PROGRESS_FACTOR = 0.9;

/**
 * How far the goals are turned down for the polish pass -- see the two-stage
 * comment in `fitSkeleton`. Loops already outrank goals 100:1 inside the
 * solver (`LOOP_WEIGHT`), so this puts welds 1e4 ahead: enough that goals
 * cannot measurably disturb a weld, while still steering the polish.
 */
const POLISH_GOAL_WEIGHT = 0.01;

function edgeKey(a: string, b: string): string {
  return a < b ? `${a}|${b}` : `${b}|${a}`;
}

function buildAdjacency(spec: ShapeSpec): Map<string, string[]> {
  const adjacency = new Map<string, string[]>();
  for (const node of spec.nodes) adjacency.set(node.id, []);
  for (const edge of spec.edges) {
    if (!adjacency.has(edge.from) || !adjacency.has(edge.to)) continue;
    adjacency.get(edge.from)!.push(edge.to);
    adjacency.get(edge.to)!.push(edge.from);
  }
  return adjacency;
}

/**
 * Splits the skeleton into maximal paths, breaking only at junctions and dead
 * ends (degree != 2). Straight-through nodes stay inside a path so a long run
 * of beams is fitted as one chain sized for the whole run.
 *
 * A cycle with no junction on it at all -- a free-standing ring -- has no
 * natural place to break, so it is walked once from an arbitrary node.
 */
export function skeletonPaths(spec: ShapeSpec): string[][] {
  const adjacency = buildAdjacency(spec);
  const visited = new Set<string>();
  const paths: string[][] = [];

  const degree = (id: string) => adjacency.get(id)?.length ?? 0;

  function walk(start: string, first: string): string[] {
    const path = [start, first];
    visited.add(edgeKey(start, first));
    let previous = start;
    let current = first;
    while (degree(current) === 2) {
      const next = adjacency.get(current)!.find((n) => n !== previous);
      if (!next || visited.has(edgeKey(current, next))) break;
      visited.add(edgeKey(current, next));
      path.push(next);
      previous = current;
      current = next;
    }
    return path;
  }

  for (const node of spec.nodes) {
    if (degree(node.id) === 2) continue;
    for (const neighbour of adjacency.get(node.id) ?? []) {
      if (visited.has(edgeKey(node.id, neighbour))) continue;
      paths.push(walk(node.id, neighbour));
    }
  }
  // Whatever is left belongs to rings with no junction to break at.
  for (const node of spec.nodes) {
    for (const neighbour of adjacency.get(node.id) ?? []) {
      if (visited.has(edgeKey(node.id, neighbour))) continue;
      paths.push(walk(node.id, neighbour));
    }
  }

  return paths;
}

interface PlannedPath {
  /** Oriented so index 0 is the node this path attaches at, when it has one. */
  nodeIds: string[];
  anchorNode: string | null;
}

/**
 * Orders paths so every one except the first of its component comes after a
 * path it shares a node with. Same reasoning as the cube builder's traversal:
 * a branch can only be fitted outward from a real connector if the thing it
 * branches off has already been placed.
 */
function planTraversalOrder(paths: string[][]): PlannedPath[] {
  const remaining = new Set(paths.map((_, i) => i));
  const byEndpoint = new Map<string, number[]>();
  paths.forEach((path, i) => {
    for (const id of new Set([path[0]!, path[path.length - 1]!])) {
      const list = byEndpoint.get(id) ?? [];
      list.push(i);
      byEndpoint.set(id, list);
    }
  });

  const planned: PlannedPath[] = [];
  while (remaining.size > 0) {
    // Seed with the longest remaining path -- the most trunk-like thing to
    // hang the rest of the component off.
    let seed = -1;
    for (const i of remaining) {
      if (seed < 0 || paths[i]!.length > paths[seed]!.length) seed = i;
    }
    remaining.delete(seed);
    planned.push({ nodeIds: paths[seed]!, anchorNode: null });

    const queue: string[][] = [paths[seed]!];
    while (queue.length > 0) {
      const current = queue.shift()!;
      for (const endpoint of [current[0]!, current[current.length - 1]!]) {
        for (const i of byEndpoint.get(endpoint) ?? []) {
          if (!remaining.has(i)) continue;
          remaining.delete(i);
          const raw = paths[i]!;
          const oriented = raw[0] === endpoint ? raw : [...raw].reverse();
          planned.push({ nodeIds: oriented, anchorNode: endpoint });
          queue.push(oriented);
        }
      }
    }
  }
  return planned;
}

/** Independent cycles in the skeleton: |E| - |V| + components. */
export function skeletonLoopCount(spec: ShapeSpec): number {
  const adjacency = buildAdjacency(spec);
  const seen = new Set<string>();
  let components = 0;
  for (const node of spec.nodes) {
    if (seen.has(node.id)) continue;
    components += 1;
    const queue = [node.id];
    seen.add(node.id);
    while (queue.length > 0) {
      const current = queue.shift()!;
      for (const neighbour of adjacency.get(current) ?? []) {
        if (seen.has(neighbour)) continue;
        seen.add(neighbour);
        queue.push(neighbour);
      }
    }
  }
  const edgeCount = new Set(spec.edges.map((e) => edgeKey(e.from, e.to))).size;
  return Math.max(0, edgeCount - spec.nodes.length + components);
}

function weld(a: Connector, b: Connector, assembly: Assembly): void {
  a.locked = true;
  b.locked = true;
  a.connectedTo = b.id;
  b.connectedTo = a.id;
  assembly.edges.push({ a: a.id, b: b.id });
}

/**
 * Builds a welded assembly approximating `spec`.
 *
 * The returned assembly is self-contained -- fresh modules, welds applied, and
 * loops solved. Check `loopReport.converged` and each chain's `residual`
 * before trusting it: a shape that cannot be built comes back as a report
 * saying so, not an exception.
 */
export function fitSkeleton(spec: ShapeSpec, options: SkeletonFitOptions = {}): SkeletonFitResult {
  const tolerance = options.tolerance ?? 0.15;
  const positions = new Map(spec.nodes.map((n) => [n.id, n.position]));

  const assembly: Assembly = { modules: {}, edges: [] };
  const chains: ChainSummary[] = [];
  const unanchored: string[] = [];
  const consumed = new Set<string>();

  const planned = planTraversalOrder(skeletonPaths(spec));

  // How many path ends land on each node. Anything reached twice or more is a
  // place where beams genuinely meet, so a chain arriving there has something
  // to be joined to -- including a ring, whose single path starts and ends on
  // the same node.
  const endpointCount = new Map<string, number>();
  for (const { nodeIds } of planned) {
    for (const id of [nodeIds[0]!, nodeIds[nodeIds.length - 1]!]) {
      endpointCount.set(id, (endpointCount.get(id) ?? 0) + 1);
    }
  }

  /**
   * Welds already made at each node, against a hard budget of one fewer than
   * the chain-ends meeting there.
   *
   * Joining k things into one takes exactly k-1 welds; every weld past that
   * adds a cycle the target shape does not have. Those extra cycles are not
   * harmless -- each one costs 6 more constraints, and a structure carrying
   * constraints its geometry never asked for is over-constrained and simply
   * will not close. Budgeting here is what keeps the assembly's cycle count
   * equal to the skeleton's.
   */
  const weldsAtNode = new Map<string, number>();
  const canWeldAt = (nodeId: string) =>
    (weldsAtNode.get(nodeId) ?? 0) < (endpointCount.get(nodeId) ?? 0) - 1;
  const recordWeldAt = (nodeId: string) =>
    weldsAtNode.set(nodeId, (weldsAtNode.get(nodeId) ?? 0) + 1);

  for (const { nodeIds, anchorNode } of planned) {
    const centers = nodeIds.map((id) => positions.get(id)!).filter(Boolean);
    if (centers.length < 2) continue;

    let points = centers;
    let chainStartPose: Pose | null = null;
    let anchoredTo: ChainSummary['anchoredTo'] = null;
    let anchorConnector: Connector | null = null;

    if (anchorNode && canWeldAt(anchorNode)) {
      const junction = new Vector3(...positions.get(anchorNode)!);
      const armDirection = new Vector3(...centers[1]!).sub(junction);
      // Everything already built lives in one assembly, so a single ref is the
      // whole search space; `findWeldAnchor` hands back the module id directly.
      const anchor = findWeldAnchor([{ chainKey: 'built', assembly }], junction, armDirection, consumed);

      if (anchor) {
        // `buildFittedChain` wants a chain-FORWARD frame and applies the weld
        // flip itself; the host connector already faces the way this arm runs.
        chainStartPose = anchor.pose;

        // A branch connector rides partway down its host's own rod, so it is
        // never exactly AT the junction node it stands in for -- this chain
        // starts a real distance from where its path nominally begins, and
        // the fit has to absorb that gap. It is corrected globally rather than
        // here: every node this path is meant to hit is handed to the closing
        // solve below as a goal, which pulls the whole structure back onto the
        // design once all of it exists.
        //
        // Re-basing the path's remaining points by the anchor's own offset
        // instead was tried and is not worth it. It cannot touch the last node
        // (that one is either a shared corner a weld must reconcile, or a leaf
        // the goals target -- shifting it just sets the fit against the pass
        // meant to correct it, which measurably broke both). That leaves only
        // a multi-beam path's interior, and paths here are overwhelmingly
        // single beams, so it changed nothing on any shape measured.
        points = [anchor.pose.position, ...centers.slice(1)];
        anchorConnector = connectorByEnd(assembly.modules[anchor.moduleId]!, anchor.end);
        anchoredTo = { moduleId: anchor.moduleId, end: anchor.end };
        consumed.add(anchorKeyId(anchor));
      } else {
        unanchored.push(anchorNode);
      }
    }

    const stroke: Stroke = { id: nodeIds.join('>'), points };
    const length = strokeArcLength(points.map((p) => new Vector3(...p)));
    const moduleCount = idealModuleCount(length, points.length);
    const { assembly: chainAssembly, fitResult } = buildFittedChain(
      stroke,
      moduleCount,
      chainStartPose ?? computeChainStartPoseFromStroke(stroke),
      tolerance,
    );

    const chainModules = Object.values(chainAssembly.modules);
    for (const module of chainModules) assembly.modules[module.id] = module;
    assembly.edges.push(...chainAssembly.edges);

    if (anchorConnector) {
      weld(anchorConnector, chainModules[0]!.connectorA, assembly);
      recordWeldAt(anchorNode!);
    }

    chains.push({
      nodeIds,
      moduleIds: chainModules.map((m) => m.id),
      residual: fitResult.residual,
      anchoredTo,
    });
  }

  // Closing pass. The loop above only ever welds a chain's START (onto whatever
  // it branched from), which leaves every chain's far end dangling -- so a seat
  // frame came out as four beams that merely looked joined, and a ring never
  // met itself at all. Here each chain that ends on a shared node reaches for a
  // free connector there and welds to it. These are precisely the welds that
  // create the kinematic loops, which the closing solve then has to satisfy.
  for (const chain of chains) {
    const lastNode = chain.nodeIds[chain.nodeIds.length - 1]!;
    if (!canWeldAt(lastNode)) continue;

    const lastModule = assembly.modules[chain.moduleIds[chain.moduleIds.length - 1]!]!;
    if (lastModule.connectorB.locked) continue;

    const junction = new Vector3(...positions.get(lastNode)!);
    // A partner here must face back down the beam we arrived on, opposing our
    // own outward normal -- that is what "welded" means geometrically.
    const inbound = new Vector3(...positions.get(chain.nodeIds[chain.nodeIds.length - 2]!)!).sub(junction);
    // Excluding `lastModule` covers welding a connector to itself as well as
    // to any of its own siblings -- both are welds within one rigid body, and
    // neither can ever close. See `findWeldAnchor`.
    const anchor = findWeldAnchor([{ chainKey: 'built', assembly }], junction, inbound, consumed, lastModule.id);
    if (!anchor) continue;

    const partner = connectorByEnd(assembly.modules[anchor.moduleId]!, anchor.end);
    weld(partner, lastModule.connectorB, assembly);
    consumed.add(anchorKeyId(anchor));
    recordWeldAt(lastNode);
  }

  // Junctions and rings leave welds the spanning tree cannot satisfy. Solving
  // them is what makes a seat frame or a ladder an actual closed structure
  // rather than a set of beams that merely look joined.
  //
  // That solve is also handed a GOAL for every chain's far tip: pull it back
  // to the skeleton node it was meant to reach. This is what makes the fit
  // GLOBAL instead of greedy. Up to here each chain is fitted once, in
  // isolation, starting from a connector that already sits off its junction
  // -- so error does not just persist, it compounds outward, every branch
  // inheriting its host's offset and adding its own, with nothing that ever
  // looks at the finished structure as a whole. That is what made a generated
  // shape come out recognisably wrong while every individual chain reported a
  // small residual: the residuals were measured against drifted starting
  // points, not against the design. Measured on a generated chair, tips
  // landed a mean of 3.5 world units from their nodes -- on beams 3 units
  // long -- before this pass; 0.8 after.
  //
  // The machinery is already here: loop closure runs a whole-assembly IK
  // descent over every joint. Goals simply give the joints that no weld pins
  // down something to move toward, instead of leaving them wherever the
  // one-shot forward fit happened to stop. `LOOP_WEIGHT` keeps welds
  // non-negotiable, so a goal can straighten a chain but never pull apart two
  // connectors that must physically meet.
  //
  // At most ONE goal per skeleton node. Several chains can end on the same
  // corner, and giving each its own row toward that corner does not reinforce
  // the target -- the rows sit on different connectors, each pulling along its
  // own Jacobian direction, so they fight each other and the weld already
  // holding them together. One representative tip states it once and lets the
  // weld carry the rest.
  const goalNodes = new Set<string>();
  const goals: ConnectorGoal[] = [];
  for (const chain of chains) {
    const tipNode = chain.nodeIds[chain.nodeIds.length - 1]!;
    if (goalNodes.has(tipNode)) continue;
    goalNodes.add(tipNode);
    goals.push({
      moduleId: chain.moduleIds[chain.moduleIds.length - 1]!,
      end: 'B',
      target: { position: positions.get(tipNode)!, quaternion: [0, 0, 0, 1] },
      positionOnly: true,
    });
  }

  const solveOptions = {
    // No restarts by default: fitting each chain to the skeleton already lands
    // very close to a consistent configuration, so the warm start is excellent
    // and a random restart would throw it away for a far worse guess.
    restarts: options.loopRestarts ?? 0,
    maxIterations: options.loopIterations ?? POLISH_ITERATIONS,
  };

  // Applied unconditionally, where this used to keep the solved angles only if
  // the loops converged. A descent only ever accepts a step that lowers its
  // residual, so its angles hold the welds at least as well as the forward
  // fit's did -- discarding them on failure threw away the best configuration
  // found and kept a strictly worse one. `converged` still reports honestly
  // whether the welds actually hold; that is information for the caller, not a
  // reason to return worse angles.
  const applyAngles = (angles: Map<ModuleId, number[]>) => {
    for (const [moduleId, solvedAngles] of angles) {
      assembly.modules[moduleId]!.rods.forEach((rod, i) => {
        rod.angle = solvedAngles[i]!;
      });
    }
  };

  // Two stages, because the two jobs want different things from the same
  // descent. Goals pull the structure onto the design; welds must hold
  // exactly. Run together at full strength the goal rows keep nudging at the
  // equilibrium and the welds settle around 2e-4 -- visually fine, but an
  // order of magnitude short of the tolerance that makes `converged` mean
  // anything, and a weld that "nearly" holds is the one thing this solver
  // must not report loosely.
  //
  // So: shape first, then polish. Stage 2 warm starts from stage 1's angles
  // and drives the welds to machine precision.
  //
  // The polish keeps the goals, at a fraction of their weight. Dropping them
  // entirely is the obvious thing and it is wrong: closing the loops leaves
  // most of the assembly's joints free (mobility is 40-odd DOF here), so a
  // loops-only descent is free to wander anywhere in that nullspace on its
  // way to a tighter weld -- and it does, undoing the shape it was handed.
  // Measured on the template chair, a goal-free polish threw away nearly the
  // whole gain, mean tip error 0.81 -> 3.72, worse than not having shaped it
  // at all. Held at low weight the goals cannot fight the welds (already 100x
  // via LOOP_WEIGHT, so 1e4x here) but they still pin down which of the many
  // weld-satisfying configurations the polish settles into: the one that
  // still looks like the thing that was asked for.
  const polishGoals = goals.map((goal) => ({ ...goal, weight: POLISH_GOAL_WEIGHT }));
  applyAngles(
    solveConstrained(assembly, goals, {
      ...solveOptions,
      maxIterations: options.shapeIterations ?? SHAPE_ITERATIONS,
    }).angles,
  );

  // The polish runs in escalating rounds rather than one fixed budget, because
  // no single budget is right. Each round warm starts from the last (the
  // angles are applied as we go), so rounds continue the same descent rather
  // than repeating it.
  //
  // A budget has to be chosen blind here: the solver's own early exit cannot
  // fire, since the goal rows hold the total residual far above its stopping
  // tolerance however good the welds get. Shapes vary by more than an order of
  // magnitude in what they need -- most close inside 40 steps, while a table
  // whose beams are all exactly equal was still at 1e-4 there and needed 80 to
  // reach 1e-8. Picking one number either abandons that table just short of a
  // solution it was about to find, or makes every easy shape pay for the
  // hardest one.
  //
  // So: stop as soon as the welds hold, and otherwise keep going only while
  // the extra round is actually buying something. That last condition is what
  // keeps a genuinely unbuildable shape cheap -- an over-constrained structure
  // stalls at the same error no matter how long it is given, and there is
  // nothing to gain by giving it more.
  let solved = solveConstrained(assembly, polishGoals, solveOptions);
  applyAngles(solved.angles);
  for (let round = 1; round <= POLISH_MAX_ESCALATIONS && !solved.report.converged; round += 1) {
    const previousError = solved.report.maxPositionError;
    solved = solveConstrained(assembly, polishGoals, {
      ...solveOptions,
      maxIterations: solveOptions.maxIterations * 2 ** round,
    });
    applyAngles(solved.angles);
    if (solved.report.maxPositionError > previousError * POLISH_PROGRESS_FACTOR) break;
  }

  const loopReport: LoopClosureReport | null = solved.report.loopCount > 0 ? solved.report : null;

  return {
    assembly,
    chains,
    unanchored,
    loopReport,
    moduleCount: Object.keys(assembly.modules).length,
    loopCount: skeletonLoopCount(spec),
  };
}
