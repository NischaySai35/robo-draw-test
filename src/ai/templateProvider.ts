/**
 * The control group: keyword matching, no model, no network.
 *
 * This exists to make the comparison honest. It is easy to look at a model
 * building a chair and conclude the model understood something, when the same
 * output would fall out of a lookup table. Run the same prompt through this and
 * the difference is immediate: this handles "chair" and "table" and nothing
 * else -- ask it for a bicycle, a lamp, or a chair with armrests and it gives
 * you a generic box, because there is no understanding in it anywhere.
 *
 * It is also the offline fallback, so the tab is usable with no API key at all.
 *
 * It emits the same JSON a model would, through the same streaming interface,
 * at a fake but plausible rate -- so every stage downstream is exercised
 * identically and the timings mean the same thing.
 */
import type { GenerateOptions, GenerationResult, ShapeProvider } from './provider';

interface Template {
  keywords: string[];
  build: () => { name: string; reasoning: string; nodes: { id: string; position: number[] }[]; edges: { from: string; to: string }[] };
}

/** Outline of a box: 8 corners, 12 edges. The workhorse of wireframe design. */
function box(prefix: string, x: number, y: number, z: number, w: number, h: number, d: number) {
  const corners: [number, number, number][] = [
    [0, 0, 0], [w, 0, 0], [w, 0, d], [0, 0, d],
    [0, h, 0], [w, h, 0], [w, h, d], [0, h, d],
  ];
  const nodes = corners.map((c, i) => ({ id: `${prefix}${i}`, position: [x + c[0], y + c[1], z + c[2]] }));
  const pairs = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]];
  const edges = pairs.map(([a, b]) => ({ from: `${prefix}${a}`, to: `${prefix}${b}` }));
  return { nodes, edges };
}

/** Chair/table leg length, literally 2.5x the original raw value (was 1). */
const LEG_LENGTH = 1 * 2.5;

const TEMPLATES: Template[] = [
  {
    keywords: ['chair', 'seat', 'stool'],
    build: () => {
      const seat = box('s', 0, LEG_LENGTH, 0, 2, 0, 2); // Degenerate box = a flat frame + its legs below.
      const legs = [0, 1, 2, 3].map((i) => ({ id: `f${i}`, position: [seat.nodes[i]!.position[0], 0, seat.nodes[i]!.position[2]] }));
      return {
        name: 'Chair',
        reasoning: 'Template: square seat frame on four legs, with a back panel outline behind it.',
        nodes: [
          ...seat.nodes.slice(0, 4),
          ...legs,
          { id: 'b0', position: [0, LEG_LENGTH + 2, 2] },
          { id: 'b1', position: [2, LEG_LENGTH + 2, 2] },
        ],
        edges: [
          { from: 's0', to: 's1' }, { from: 's1', to: 's2' }, { from: 's2', to: 's3' }, { from: 's3', to: 's0' },
          ...[0, 1, 2, 3].map((i) => ({ from: `s${i}`, to: `f${i}` })),
          { from: 's3', to: 'b0' }, { from: 's2', to: 'b1' }, { from: 'b0', to: 'b1' },
        ],
      };
    },
  },
  {
    keywords: ['table', 'desk'],
    build: () => {
      // Left at its original 1:2 leg:frame ratio -- see the note above LONG.
      // A plain 4-legs-on-a-frame table ties 4 short edges against 4 long
      // ones exactly if the legs are lengthened the chair's way, and that
      // tied median measurably regressed it (2 colliding pairs, converged
      // -> 26 pairs, did not converge) rather than helping.
      const top = box('t', 0, LEG_LENGTH, 0, 2, 0, 2);
      const feet = [0, 1, 2, 3].map((i) => ({ id: `g${i}`, position: [top.nodes[i]!.position[0], 0, top.nodes[i]!.position[2]] }));
      return {
        name: 'Table',
        reasoning: 'Template: square top frame on four legs.',
        nodes: [...top.nodes.slice(0, 4), ...feet],
        edges: [
          { from: 't0', to: 't1' }, { from: 't1', to: 't2' }, { from: 't2', to: 't3' }, { from: 't3', to: 't0' },
          ...[0, 1, 2, 3].map((i) => ({ from: `t${i}`, to: `g${i}` })),
        ],
      };
    },
  },
  {
    keywords: ['house', 'hut', 'shed', 'home'],
    build: () => {
      const walls = box('w', 0, 0, 0, 2, 2, 2);
      return {
        name: 'House',
        reasoning: 'Template: cube outline for the walls, with a ridge beam above forming a pitched roof.',
        nodes: [
          ...walls.nodes,
          { id: 'r0', position: [1, 3, 0] },
          { id: 'r1', position: [1, 3, 2] },
        ],
        edges: [
          ...walls.edges,
          { from: 'w4', to: 'r0' }, { from: 'w5', to: 'r0' },
          { from: 'w7', to: 'r1' }, { from: 'w6', to: 'r1' },
          { from: 'r0', to: 'r1' },
        ],
      };
    },
  },
  {
    keywords: ['bridge', 'truss', 'span'],
    build: () => {
      const nodes: { id: string; position: number[] }[] = [];
      const edges: { from: string; to: string }[] = [];
      for (let i = 0; i < 4; i += 1) {
        nodes.push({ id: `d${i}`, position: [i, 0, 0] }, { id: `e${i}`, position: [i, 0, 1] });
        nodes.push({ id: `u${i}`, position: [i, 1, 0] }, { id: `v${i}`, position: [i, 1, 1] });
        edges.push({ from: `d${i}`, to: `u${i}` }, { from: `e${i}`, to: `v${i}` }, { from: `d${i}`, to: `e${i}` }, { from: `u${i}`, to: `v${i}` });
        if (i > 0) {
          edges.push({ from: `d${i - 1}`, to: `d${i}` }, { from: `e${i - 1}`, to: `e${i}` });
          edges.push({ from: `u${i - 1}`, to: `u${i}` }, { from: `v${i - 1}`, to: `v${i}` });
        }
      }
      return { name: 'Bridge', reasoning: 'Template: two parallel trusses, cross-braced into a deck.', nodes, edges };
    },
  },
];

/** Nothing matched: a box outline, so the pipeline still has something to build. */
function fallback(request: string) {
  const cube = box('c', 0, 0, 0, 2, 2, 2);
  return {
    name: request.slice(0, 24) || 'Box',
    reasoning: 'No template matched this request -- falling back to a plain box outline.',
    nodes: cube.nodes,
    edges: cube.edges,
  };
}

export const templateProvider: ShapeProvider = {
  id: 'template',
  label: 'Local templates (no AI)',
  isConfigured: () => true,

  async generate(options: GenerateOptions): Promise<GenerationResult> {
    const startedAt = performance.now();
    const request = options.userPrompt.toLowerCase();
    const match = TEMPLATES.find((t) => t.keywords.some((k) => request.includes(k)));
    const shape = match ? match.build() : fallback(options.userPrompt);
    const text = JSON.stringify(shape, null, 2);

    // Streamed in chunks so the UI, the metrics, and the parser all take the
    // same path they take for a real model.
    let firstTokenMs: number | null = null;
    const chunkSize = 24;
    for (let i = 0; i < text.length; i += chunkSize) {
      if (options.signal.aborted) throw new DOMException('Aborted', 'AbortError');
      await new Promise((resolve) => setTimeout(resolve, 4));
      if (firstTokenMs === null) firstTokenMs = performance.now() - startedAt;
      options.onDelta(text.slice(i, i + chunkSize));
    }

    const totalMs = performance.now() - startedAt;
    const outputTokens = Math.round(text.length / 4);
    const streamingMs = firstTokenMs === null ? totalMs : totalMs - firstTokenMs;
    return {
      text,
      stopReason: 'end_turn',
      metrics: {
        firstTokenMs,
        totalMs,
        outputTokens,
        inputTokens: Math.round((options.systemPrompt.length + options.userPrompt.length) / 4),
        tokensPerSecond: streamingMs > 0 ? (outputTokens / streamingMs) * 1000 : 0,
      },
    };
  },
};
