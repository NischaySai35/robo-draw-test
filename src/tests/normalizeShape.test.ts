import { describe, expect, it } from 'vitest';
import { BEAM_LENGTH, extractJson, normalizeShape } from '../ai/normalizeShape';
import { fitSkeleton } from '../kinematics/skeletonFit';
import { isSelfColliding } from '../kinematics/collision';

/** A unit-grid table: exactly the shape the prompt asks a model to produce. */
const TABLE = {
  name: 'Table',
  nodes: [
    { id: 'foot-fl', position: [0, 0, 0] },
    { id: 'foot-fr', position: [1, 0, 0] },
    { id: 'foot-br', position: [1, 0, 1] },
    { id: 'foot-bl', position: [0, 0, 1] },
    { id: 'top-fl', position: [0, 1, 0] },
    { id: 'top-fr', position: [1, 1, 0] },
    { id: 'top-br', position: [1, 1, 1] },
    { id: 'top-bl', position: [0, 1, 1] },
  ],
  edges: [
    { from: 'foot-fl', to: 'top-fl' },
    { from: 'foot-fr', to: 'top-fr' },
    { from: 'foot-br', to: 'top-br' },
    { from: 'foot-bl', to: 'top-bl' },
    { from: 'top-fl', to: 'top-fr' },
    { from: 'top-fr', to: 'top-br' },
    { from: 'top-br', to: 'top-bl' },
    { from: 'top-bl', to: 'top-fl' },
  ],
};

const beamLengths = (spec: NonNullable<ReturnType<typeof normalizeShape>['spec']>): number[] => {
  const positions = new Map(spec.nodes.map((n) => [n.id, n.position]));
  return spec.edges.map((e) => {
    const a = positions.get(e.from)!;
    const b = positions.get(e.to)!;
    return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  });
};

describe('normalizeShape scaling', () => {
  it('rescales a unit-grid shape to real module lengths', () => {
    // The point of the whole module: a model has no idea what a beam is, and
    // must not need to. Unit input, buildable output.
    const { spec } = normalizeShape(TABLE, 'Table');
    expect(spec).not.toBeNull();
    for (const length of beamLengths(spec!)) {
      expect(length).toBeGreaterThan(BEAM_LENGTH * 0.75);
      expect(length).toBeLessThan(BEAM_LENGTH * 1.3);
    }
  });

  it('produces the same structure whether the model used units, centimetres, or metres', () => {
    // Models are wildly inconsistent about scale between runs of the SAME
    // prompt. If scale survived into the output, results would be unrepeatable.
    const scaled = (factor: number) => ({
      ...TABLE,
      nodes: TABLE.nodes.map((n) => ({ id: n.id, position: n.position.map((v) => v * factor) })),
    });
    const small = normalizeShape(scaled(0.01), 'Table').spec!;
    const large = normalizeShape(scaled(100), 'Table').spec!;
    expect(small.nodes).toHaveLength(large.nodes.length);
    expect(small.edges).toHaveLength(large.edges.length);
    expect(beamLengths(small)[0]).toBeCloseTo(beamLengths(large)[0]!, 6);
  });

  it('splits a beam that spans several modules', () => {
    // Three 1-unit beams so the median is unambiguously 1, plus one 4-unit run
    // that has to become four beams.
    const stretched = {
      name: 'L',
      nodes: [
        { id: 'a', position: [0, 0, 0] },
        { id: 'b', position: [1, 0, 0] },
        { id: 'c', position: [1, 1, 0] },
        { id: 'd', position: [0, 1, 0] },
        { id: 'far', position: [5, 0, 0] },
      ],
      edges: [
        { from: 'a', to: 'b' },
        { from: 'b', to: 'c' },
        { from: 'c', to: 'd' },
        { from: 'b', to: 'far' },
      ],
    };
    const { spec, repairs } = normalizeShape(stretched, 'L');
    // 3 unchanged beams + the 4-unit run split into 4.
    expect(spec!.edges).toHaveLength(7);
    expect(repairs.some((r) => r.includes('Split'))).toBe(true);
    for (const length of beamLengths(spec!)) expect(length).toBeLessThan(BEAM_LENGTH * 1.3);
  });

  it('merges joints too close together to be a real beam', () => {
    // One module is the smallest thing that exists; a beam shorter than that
    // is not a small beam, it is a mistake.
    const doubled = {
      name: 'Doubled',
      nodes: [
        { id: 'a', position: [0, 0, 0] },
        { id: 'a2', position: [0.02, 0, 0] },
        { id: 'b', position: [1, 0, 0] },
        { id: 'c', position: [1, 1, 0] },
      ],
      edges: [{ from: 'a', to: 'b' }, { from: 'a2', to: 'c' }, { from: 'b', to: 'c' }],
    };
    const { spec, repairs } = normalizeShape(doubled, 'Doubled');
    expect(spec!.nodes).toHaveLength(3);
    expect(repairs.some((r) => r.includes('Merged'))).toBe(true);
  });

  it('stands the structure on the floor and centres it', () => {
    const floating = {
      ...TABLE,
      nodes: TABLE.nodes.map((n) => ({ id: n.id, position: [n.position[0]! + 500, n.position[1]! + 40, n.position[2]! - 90] })),
    };
    const { spec } = normalizeShape(floating, 'Table');
    const ys = spec!.nodes.map((n) => n.position[1]);
    const xs = spec!.nodes.map((n) => n.position[0]);
    expect(Math.min(...ys)).toBeCloseTo(0, 6);
    expect((Math.min(...xs) + Math.max(...xs)) / 2).toBeCloseTo(0, 6);
  });
});

describe('normalizeShape repair', () => {
  it('drops beams pointing at nodes the model never defined', () => {
    const hallucinated = {
      ...TABLE,
      edges: [...TABLE.edges, { from: 'top-fl', to: 'armrest-left' }],
    };
    const { spec, repairs } = normalizeShape(hallucinated, 'Table');
    expect(spec).not.toBeNull();
    expect(repairs.some((r) => r.includes('never defined'))).toBe(true);
  });

  it('keeps the largest piece when the model emits a detached one', () => {
    const detached = {
      ...TABLE,
      nodes: [...TABLE.nodes, { id: 'lost-a', position: [40, 0, 0] }, { id: 'lost-b', position: [41, 0, 0] }],
      edges: [...TABLE.edges, { from: 'lost-a', to: 'lost-b' }],
    };
    const { spec, repairs } = normalizeShape(detached, 'Table');
    expect(spec!.nodes.every((n) => !n.id.startsWith('lost'))).toBe(true);
    expect(repairs.some((r) => r.includes('separate pieces'))).toBe(true);
  });

  it('refuses, with a reason, when there is nothing to build', () => {
    const { spec, errors } = normalizeShape({ nodes: [{ id: 'a', position: [0, 0, 0] }], edges: [] }, 'X');
    expect(spec).toBeNull();
    expect(errors.join(' ')).toContain('at least 2');
  });

  it('refuses a shape too large to solve rather than hanging on it', () => {
    const nodes = Array.from({ length: 200 }, (_, i) => ({ id: `n${i}`, position: [i, 0, 0] }));
    const edges = nodes.slice(1).map((n, i) => ({ from: `n${i}`, to: n.id }));
    const { spec, errors } = normalizeShape({ nodes, edges }, 'Huge');
    expect(spec).toBeNull();
    expect(errors.join(' ')).toContain('Too big');
  });
});

describe('extractJson', () => {
  it('finds JSON wrapped in prose and markdown fences', () => {
    const text = 'Sure! Here is the design:\n```json\n{"nodes":[],"edges":[]}\n```\nHope that helps.';
    expect(extractJson(text).value).toEqual({ nodes: [], edges: [] });
  });

  it('is not fooled by braces inside strings', () => {
    const { value } = extractJson('{"name":"a }{ tricky name","nodes":[]}');
    expect((value as { name: string }).name).toBe('a }{ tricky name');
  });

  it('names truncation as truncation, not as bad JSON', () => {
    // These need opposite fixes -- raise max tokens vs change the prompt -- so
    // reporting them the same way sends you down the wrong path.
    const { value, error } = extractJson('{"nodes":[{"id":"a","posi');
    expect(value).toBeNull();
    expect(error).toContain('cut off');
  });
});

describe('end to end', () => {
  it('builds a normalized unit-grid table into a real assembly', { timeout: 30000 }, () => {
    // The claim this tab makes: model-shaped JSON in, buildable robot out.
    const { spec } = normalizeShape(TABLE, 'Table');
    const fit = fitSkeleton(spec!);
    // One module per beam -- the sizing rule holding up end to end.
    expect(fit.moduleCount).toBe(spec!.edges.length);
    expect(fit.unanchored).toHaveLength(0);
    // A table top is a closed frame, so this exercises loop closure too.
    expect(fit.loopCount).toBeGreaterThan(0);
    expect(fit.loopReport?.converged).toBe(true);
  });

  it('still collides where three beams meet at a right-angled corner', { timeout: 30000 }, () => {
    // Recorded as a known limit, not asserted away. A box corner joins three
    // beams at 90 degrees, which is tighter than anything in the hand-built
    // shape library: the chair's worst contact is 0.106, a table corner's is
    // 0.279 -- deep enough to be real hardware interference, not fit residual.
    //
    // The cause is anchor choice. `findWeldAnchor` picks the free connector
    // best ALIGNED with the outgoing beam and never asks whether the module it
    // is about to place has room, so at a 3-way corner two branches can be sent
    // to anchors whose end domes then occupy the same space. Fixing it means
    // making anchor selection collision-aware, which is a change to the fitter,
    // not to this pipeline. Flip this test when that lands.
    const { spec } = normalizeShape(TABLE, 'Table');
    const fit = fitSkeleton(spec!);
    expect(isSelfColliding(fit.assembly)).toBe(true);
  });
});
