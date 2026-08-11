/**
 * Turns whatever a language model actually emitted into a spec `fitSkeleton`
 * can build -- or says clearly why it cannot.
 *
 * This module, not the prompt, is what makes text-to-structure work. A model
 * asked for "a chair" reliably produces the right TOPOLOGY: four legs, a seat
 * frame, a back. What it cannot produce is the right SCALE, because the only
 * length that matters here is `MODULE_CHAIN_LENGTH * 0.8` -- a number that
 * falls out of this robot's rod lengths and means nothing to a model. Ask for
 * metres and you get a chair sized for a person, whose every beam is 20 modules
 * long. Put the real number in the prompt and the model does arithmetic instead
 * of design, badly.
 *
 * So the model is told to work in whatever units it likes, and this rescales.
 * The pipeline, in order, each step existing because of a specific failure:
 *
 *  1. Sanitise. Drop malformed nodes, edges pointing at nodes that were never
 *     declared (models hallucinate an endpoint roughly one time in ten),
 *     self-edges, and duplicates.
 *  2. Keep the largest connected component. `fitSkeleton` builds ONE assembly;
 *     a stray disconnected node would otherwise be silently unbuildable.
 *  3. Scale so the MEDIAN beam is one module long. Median, not mean or min: a
 *     single stray short edge would otherwise inflate the whole structure, and
 *     a single long one would crush it.
 *  4. Merge nodes closer than half a beam. After scaling these are joints the
 *     model meant to be the same point, and a sub-module beam cannot be built
 *     at all -- one module is the smallest thing that exists.
 *  5. Subdivide beams longer than one module. `computeFeasibility` already
 *     rounds up to a multi-module chain, but a chain spanning several beams
 *     coils to fit; splitting the beam gives each module a straight run.
 *  6. Recentre on the origin and stand it on the floor, so the result appears
 *     in front of the camera rather than wherever the model's mental origin was.
 *
 * Every repair is reported rather than applied silently. When a generated shape
 * builds badly, the repair list is the first place to look.
 */
import { MODULE_CHAIN_LENGTH } from '../constants/geometry';
import type { ShapeSpec, SkeletonEdge, SkeletonNode } from '../kinematics/skeletonFit';

/** One module per beam, matching the sizing rule in `shapeLibrary`. */
export const BEAM_LENGTH = MODULE_CHAIN_LENGTH * 0.8;

/** Below this fraction of a beam, two nodes are the same joint. */
const MERGE_FRACTION = 0.5;
/** Above this fraction, a beam is split. The slack stops 1.01x beams splitting in two. */
const SUBDIVIDE_FRACTION = 1.25;

/** Refuse to build past this -- a runaway spec would hang the solver, not fail it. */
export const MAX_NODES = 80;
export const MAX_EDGES = 120;

export interface NormalizeResult {
  spec: ShapeSpec | null;
  /** Human-readable record of every change made, in the order made. */
  repairs: string[];
  /** Why `spec` is null. Empty when it isn't. */
  errors: string[];
  /** Scale factor applied to the model's coordinates. Diagnostic only. */
  scale: number;
}

type Vec3 = [number, number, number];

const distance = (a: Vec3, b: Vec3): number =>
  Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

const edgeKey = (a: string, b: string): string => (a < b ? `${a}|${b}` : `${b}|${a}`);

function median(values: number[]): number {
  const sorted = [...values].sort((x, y) => x - y);
  const mid = sorted.length >> 1;
  return sorted.length % 2 === 0 ? (sorted[mid - 1]! + sorted[mid]!) / 2 : sorted[mid]!;
}

/**
 * Reads a raw parsed JSON value into nodes and edges, discarding anything
 * malformed. Deliberately tolerant: a model that gets 90% of a chair right
 * should give us 90% of a chair, not an exception.
 */
function sanitize(raw: unknown, repairs: string[], errors: string[]): { nodes: SkeletonNode[]; edges: SkeletonEdge[] } | null {
  if (typeof raw !== 'object' || raw === null) {
    errors.push('Model output was not a JSON object.');
    return null;
  }
  const record = raw as Record<string, unknown>;
  if (!Array.isArray(record.nodes) || !Array.isArray(record.edges)) {
    errors.push('Model output is missing a `nodes` or `edges` array.');
    return null;
  }

  const nodes: SkeletonNode[] = [];
  const seenIds = new Set<string>();
  for (const candidate of record.nodes) {
    if (typeof candidate !== 'object' || candidate === null) continue;
    const node = candidate as Record<string, unknown>;
    const id = typeof node.id === 'string' ? node.id.trim() : '';
    const position = node.position;
    if (!id || !Array.isArray(position) || position.length < 3) continue;
    const xyz = position.slice(0, 3).map(Number) as Vec3;
    if (xyz.some((v) => !Number.isFinite(v))) continue;
    if (seenIds.has(id)) {
      repairs.push(`Dropped a second node also called "${id}".`);
      continue;
    }
    seenIds.add(id);
    nodes.push({ id, position: xyz });
  }

  if (nodes.length < 2) {
    errors.push(`Only ${nodes.length} usable node(s) -- a structure needs at least 2.`);
    return null;
  }

  const edges: SkeletonEdge[] = [];
  const seenEdges = new Set<string>();
  let missing = 0;
  let selfEdges = 0;
  let duplicates = 0;
  for (const candidate of record.edges) {
    if (typeof candidate !== 'object' || candidate === null) continue;
    const edge = candidate as Record<string, unknown>;
    const from = typeof edge.from === 'string' ? edge.from.trim() : '';
    const to = typeof edge.to === 'string' ? edge.to.trim() : '';
    if (!from || !to) continue;
    if (!seenIds.has(from) || !seenIds.has(to)) {
      missing += 1;
      continue;
    }
    if (from === to) {
      selfEdges += 1;
      continue;
    }
    const key = edgeKey(from, to);
    if (seenEdges.has(key)) {
      duplicates += 1;
      continue;
    }
    seenEdges.add(key);
    edges.push({ from, to });
  }

  if (missing > 0) repairs.push(`Dropped ${missing} beam(s) pointing at a node that was never defined.`);
  if (selfEdges > 0) repairs.push(`Dropped ${selfEdges} beam(s) joining a node to itself.`);
  if (duplicates > 0) repairs.push(`Dropped ${duplicates} duplicate beam(s).`);

  if (edges.length === 0) {
    errors.push('No usable beams -- every edge referenced a node that does not exist.');
    return null;
  }
  return { nodes, edges };
}

/** Largest connected component, by node count. `fitSkeleton` builds one assembly. */
function largestComponent(
  nodes: SkeletonNode[],
  edges: SkeletonEdge[],
  repairs: string[],
): { nodes: SkeletonNode[]; edges: SkeletonEdge[] } {
  const adjacency = new Map<string, string[]>();
  for (const node of nodes) adjacency.set(node.id, []);
  for (const edge of edges) {
    adjacency.get(edge.from)!.push(edge.to);
    adjacency.get(edge.to)!.push(edge.from);
  }

  const seen = new Set<string>();
  let best: Set<string> = new Set();
  let components = 0;
  for (const node of nodes) {
    if (seen.has(node.id)) continue;
    components += 1;
    const group = new Set<string>();
    const stack = [node.id];
    seen.add(node.id);
    while (stack.length > 0) {
      const current = stack.pop()!;
      group.add(current);
      for (const next of adjacency.get(current) ?? []) {
        if (seen.has(next)) continue;
        seen.add(next);
        stack.push(next);
      }
    }
    if (group.size > best.size) best = group;
  }

  if (components > 1) {
    repairs.push(`Model produced ${components} separate pieces; kept the largest (${best.size} joints).`);
  }
  const isolated = nodes.filter((n) => best.has(n.id) && (adjacency.get(n.id) ?? []).length === 0);
  if (isolated.length > 0) repairs.push(`Dropped ${isolated.length} joint(s) with no beams attached.`);

  return {
    nodes: nodes.filter((n) => best.has(n.id) && (adjacency.get(n.id) ?? []).length > 0),
    edges: edges.filter((e) => best.has(e.from) && best.has(e.to)),
  };
}

/**
 * Collapses nodes closer than half a beam into one. A beam shorter than a
 * single module cannot be built -- one module is the smallest thing that
 * exists -- so these have to go somewhere, and merging them is what the model
 * meant: they are joints it placed at "the same" corner twice.
 */
function mergeCloseNodes(
  nodes: SkeletonNode[],
  edges: SkeletonEdge[],
  repairs: string[],
): { nodes: SkeletonNode[]; edges: SkeletonEdge[] } {
  const threshold = BEAM_LENGTH * MERGE_FRACTION;
  const representative = new Map<string, string>();
  const kept: SkeletonNode[] = [];

  for (const node of nodes) {
    const match = kept.find((k) => distance(k.position as Vec3, node.position as Vec3) < threshold);
    if (match) representative.set(node.id, match.id);
    else {
      representative.set(node.id, node.id);
      kept.push(node);
    }
  }

  const merged = nodes.length - kept.length;
  if (merged > 0) repairs.push(`Merged ${merged} joint(s) that landed closer together than one module.`);

  const seen = new Set<string>();
  const rewritten: SkeletonEdge[] = [];
  let collapsed = 0;
  for (const edge of edges) {
    const from = representative.get(edge.from)!;
    const to = representative.get(edge.to)!;
    if (from === to) {
      collapsed += 1;
      continue;
    }
    const key = edgeKey(from, to);
    if (seen.has(key)) {
      collapsed += 1;
      continue;
    }
    seen.add(key);
    rewritten.push({ from, to });
  }
  if (collapsed > 0) repairs.push(`Dropped ${collapsed} beam(s) that vanished when their endpoints merged.`);

  return { nodes: kept, edges: rewritten };
}

/**
 * Splits any beam longer than one module into equal pieces. `fitSkeleton`
 * would otherwise fit a multi-module chain to a single straight beam, and that
 * chain has to coil to take up its slack -- which is what pulls a generated
 * shape visibly out of square.
 */
function subdivideLongEdges(
  nodes: SkeletonNode[],
  edges: SkeletonEdge[],
  repairs: string[],
): { nodes: SkeletonNode[]; edges: SkeletonEdge[] } {
  const positions = new Map(nodes.map((n) => [n.id, n.position as Vec3]));
  const outNodes = [...nodes];
  const outEdges: SkeletonEdge[] = [];
  let split = 0;
  let index = 0;

  for (const edge of edges) {
    const a = positions.get(edge.from)!;
    const b = positions.get(edge.to)!;
    const length = distance(a, b);
    const pieces = Math.max(1, Math.round(length / BEAM_LENGTH));
    if (length <= BEAM_LENGTH * SUBDIVIDE_FRACTION || pieces < 2) {
      outEdges.push(edge);
      continue;
    }

    split += 1;
    let previous = edge.from;
    for (let i = 1; i < pieces; i += 1) {
      const t = i / pieces;
      const id = `split${index}_${i}`;
      outNodes.push({
        id,
        position: [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t],
      });
      outEdges.push({ from: previous, to: id });
      previous = id;
    }
    outEdges.push({ from: previous, to: edge.to });
    index += 1;
  }

  if (split > 0) repairs.push(`Split ${split} long beam(s) so each one is a single module.`);
  return { nodes: outNodes, edges: outEdges };
}

/**
 * Rescales, repairs, and grounds a raw model response.
 *
 * `raw` is the already-JSON-parsed value; parsing lives in the provider so a
 * truncated stream can be reported as a stream problem rather than a shape one.
 */
export function normalizeShape(raw: unknown, name: string): NormalizeResult {
  const repairs: string[] = [];
  const errors: string[] = [];

  const sanitized = sanitize(raw, repairs, errors);
  if (!sanitized) return { spec: null, repairs, errors, scale: 1 };

  let { nodes, edges } = largestComponent(sanitized.nodes, sanitized.edges, repairs);
  if (nodes.length < 2 || edges.length === 0) {
    errors.push('Nothing connected survived cleanup.');
    return { spec: null, repairs, errors, scale: 1 };
  }

  // Scale so the median beam is exactly one module. Everything downstream --
  // the merge threshold, the subdivision threshold -- is expressed in beams,
  // so this has to happen before them.
  const positions = new Map(nodes.map((n) => [n.id, n.position as Vec3]));
  const lengths = edges.map((e) => distance(positions.get(e.from)!, positions.get(e.to)!)).filter((d) => d > 1e-9);
  if (lengths.length === 0) {
    errors.push('Every beam has zero length -- all joints are at the same point.');
    return { spec: null, repairs, errors, scale: 1 };
  }
  const scale = BEAM_LENGTH / median(lengths);
  nodes = nodes.map((n) => ({
    id: n.id,
    position: [n.position[0] * scale, n.position[1] * scale, n.position[2] * scale],
  }));
  repairs.push(`Rescaled by ${scale.toFixed(3)}x so the typical beam is one module (${BEAM_LENGTH.toFixed(2)} units).`);

  ({ nodes, edges } = mergeCloseNodes(nodes, edges, repairs));
  if (nodes.length < 2 || edges.length === 0) {
    errors.push('The shape collapsed to a point once sub-module beams were merged.');
    return { spec: null, repairs, errors, scale };
  }

  ({ nodes, edges } = subdivideLongEdges(nodes, edges, repairs));

  if (nodes.length > MAX_NODES || edges.length > MAX_EDGES) {
    errors.push(
      `Too big to build: ${nodes.length} joints / ${edges.length} beams (limit ${MAX_NODES}/${MAX_EDGES}). Ask for something simpler.`,
    );
    return { spec: null, repairs, errors, scale };
  }

  // Centre on the origin in X/Z and stand it on the floor, so it lands in front
  // of the camera instead of wherever the model imagined its origin.
  const xs = nodes.map((n) => n.position[0]);
  const ys = nodes.map((n) => n.position[1]);
  const zs = nodes.map((n) => n.position[2]);
  const centreX = (Math.min(...xs) + Math.max(...xs)) / 2;
  const centreZ = (Math.min(...zs) + Math.max(...zs)) / 2;
  const floorY = Math.min(...ys);
  const grounded = nodes.map((n) => ({
    id: n.id,
    position: [n.position[0] - centreX, n.position[1] - floorY, n.position[2] - centreZ] as Vec3,
  }));

  return { spec: { name, nodes: grounded, edges }, repairs, errors, scale };
}

/**
 * Pulls the first balanced JSON object out of a model response. Models wrap
 * JSON in prose or fences however they feel that day, and a stream cut short by
 * a token limit is a different failure from a malformed shape, so this reports
 * which one happened.
 */
export function extractJson(text: string): { value: unknown; error: string | null } {
  const start = text.indexOf('{');
  if (start === -1) return { value: null, error: 'No JSON object in the response.' };

  let depth = 0;
  let inString = false;
  let escaped = false;
  for (let i = start; i < text.length; i += 1) {
    const char = text[i]!;
    if (inString) {
      if (escaped) escaped = false;
      else if (char === '\\') escaped = true;
      else if (char === '"') inString = false;
      continue;
    }
    if (char === '"') inString = true;
    else if (char === '{') depth += 1;
    else if (char === '}') {
      depth -= 1;
      if (depth === 0) {
        try {
          return { value: JSON.parse(text.slice(start, i + 1)), error: null };
        } catch (error) {
          return { value: null, error: `JSON did not parse: ${(error as Error).message}` };
        }
      }
    }
  }
  return { value: null, error: 'JSON object never closed -- the response was cut off (try raising max tokens).' };
}
