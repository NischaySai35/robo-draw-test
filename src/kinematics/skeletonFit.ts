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
 *  4. Close the loops. A skeleton with cycles (a seat frame, a ladder) leaves
 *     welds the spanning tree cannot satisfy; `solveLoopClosure` finishes the
 *     job or reports honestly that this shape is unreachable.
 */
import { Vector3 } from 'three';
import type { Assembly, Connector, ConnectorEnd, ModuleId, Pose } from '../types/module';
import { connectorByEnd } from '../types/module';
import type { Stroke } from '../types/draw';
import {
  buildFittedChain,
  computeChainStartPoseFromStroke,
  computeFeasibility,
  strokeArcLength,
} from './curveFit';
import { anchorKeyId, findWeldAnchor } from './branchAnchor';
import { solveLoopClosure, type LoopClosureReport } from './loopClosure';

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
  loopIterations?: number;
}

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
    const moduleCount = Math.max(1, computeFeasibility(length, 0).modulesNeeded);
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
  // create the kinematic loops, which `solveLoopClosure` then has to satisfy.
  for (const chain of chains) {
    const lastNode = chain.nodeIds[chain.nodeIds.length - 1]!;
    if (!canWeldAt(lastNode)) continue;

    const lastModule = assembly.modules[chain.moduleIds[chain.moduleIds.length - 1]!]!;
    if (lastModule.connectorB.locked) continue;

    const junction = new Vector3(...positions.get(lastNode)!);
    // A partner here must face back down the beam we arrived on, opposing our
    // own outward normal -- that is what "welded" means geometrically.
    const inbound = new Vector3(...positions.get(chain.nodeIds[chain.nodeIds.length - 2]!)!).sub(junction);
    const anchor = findWeldAnchor([{ chainKey: 'built', assembly }], junction, inbound, consumed);
    if (!anchor) continue;

    const partner = connectorByEnd(assembly.modules[anchor.moduleId]!, anchor.end);
    if (partner.id === lastModule.connectorB.id) continue; // never weld a connector to itself
    weld(partner, lastModule.connectorB, assembly);
    consumed.add(anchorKeyId(anchor));
    recordWeldAt(lastNode);
  }

  // Junctions and rings leave welds the spanning tree cannot satisfy. Solving
  // them is what makes a seat frame or a ladder an actual closed structure
  // rather than a set of beams that merely look joined.
  let loopReport: LoopClosureReport | null = null;
  // No restarts by default: fitting each chain to the skeleton already lands
  // very close to a consistent configuration, so the warm start is excellent
  // and a random restart would throw it away for a far worse guess.
  const solved = solveLoopClosure(assembly, {
    restarts: options.loopRestarts ?? 0,
    maxIterations: options.loopIterations ?? 150,
  });
  if (solved.report.loopCount > 0) {
    loopReport = solved.report;
    if (solved.report.converged) {
      for (const [moduleId, angles] of solved.angles) {
        assembly.modules[moduleId]!.rods.forEach((rod, i) => {
          rod.angle = angles[i]!;
        });
      }
    }
  }

  return {
    assembly,
    chains,
    unanchored,
    loopReport,
    moduleCount: Object.keys(assembly.modules).length,
    loopCount: skeletonLoopCount(spec),
  };
}
