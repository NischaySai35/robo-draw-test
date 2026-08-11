import { describe, expect, it } from 'vitest';
import { createModule } from '../kinematics/factory';
import { checkSelfCollision, isSelfColliding } from '../kinematics/collision';
import { fitSkeleton } from '../kinematics/skeletonFit';
import { TRIPOD } from '../kinematics/shapeLibrary';
import { IDENTITY_POSE } from '../kinematics/frame';
import {
  HEMISPHERE_RADIUS,
  ROD_RADIUS,
  SIDE_CONNECTOR_RADIAL_OFFSET,
} from '../constants/geometry';
import type { Assembly, Connector, Module } from '../types/module';

function weld(a: Connector, b: Connector): void {
  a.locked = true;
  b.locked = true;
  a.connectedTo = b.id;
  b.connectedTo = a.id;
}

function chainOf(n: number): { assembly: Assembly; modules: Module[] } {
  const modules = Array.from({ length: n }, () => createModule(IDENTITY_POSE));
  for (let i = 0; i < n - 1; i += 1) weld(modules[i]!.connectorB, modules[i + 1]!.connectorA);
  return {
    assembly: { modules: Object.fromEntries(modules.map((m) => [m.id, m])), edges: [] },
    modules,
  };
}

describe('checkSelfCollision', () => {
  it('reports a straight welded chain as clear', () => {
    // The exclusions carry this test. Without excusing the parts either side of
    // a weld, an ordinary straight chain reports overlaps at every joint and
    // the whole check is worthless.
    const report = checkSelfCollision(chainOf(6).assembly);
    expect(report.collides).toBe(false);
    expect(report.worstPenetration).toBe(0);
  });

  it('reports two unrelated modules at the same pose as colliding', () => {
    const a = createModule(IDENTITY_POSE);
    const b = createModule(IDENTITY_POSE);
    const report = checkSelfCollision({ modules: { [a.id]: a, [b.id]: b }, edges: [] });
    expect(report.collides).toBe(true);
    expect(report.pairs.length).toBeGreaterThan(0);
  });

  it('reports two modules far apart as clear', () => {
    const a = createModule(IDENTITY_POSE);
    const b = createModule({ position: [50, 0, 0], quaternion: [0, 0, 0, 1] });
    expect(isSelfColliding({ modules: { [a.id]: a, [b.id]: b }, edges: [] })).toBe(false);
  });

  it('catches a chain folded back through itself', () => {
    // Every bend hard over, so the chain coils into its own volume -- a real
    // collision between modules that are NOT welded neighbours.
    const { assembly } = chainOf(5);
    for (const module of Object.values(assembly.modules)) {
      module.rods[1]!.angle = Math.PI / 2;
      module.rods[2]!.angle = Math.PI / 2;
      module.rods[4]!.angle = Math.PI / 2;
    }
    const report = checkSelfCollision(assembly);
    expect(report.collides).toBe(true);
    expect(report.worstPenetration).toBeGreaterThan(0.5);
  });

  it('does not flag a branch welded onto a side connector', () => {
    // A tripod is three arms off one hub -- the case side connectors exist for.
    expect(isSelfColliding(fitSkeleton(TRIPOD).assembly)).toBe(false);
  });

  it('finds every pair by default and stops at the first when asked', () => {
    const { assembly } = chainOf(5);
    for (const module of Object.values(assembly.modules)) {
      module.rods[1]!.angle = Math.PI / 2;
      module.rods[2]!.angle = Math.PI / 2;
      module.rods[4]!.angle = Math.PI / 2;
    }
    const all = checkSelfCollision(assembly);
    const first = checkSelfCollision(assembly, { firstOnly: true });
    expect(all.pairs.length).toBeGreaterThan(1);
    expect(first.pairs).toHaveLength(1);
    expect(first.collides).toBe(true);
  });
});

describe('side connector spacing', () => {
  it('keeps adjacent side faces far enough apart to both carry a weld', () => {
    // Two modules welded onto faces 90 degrees apart put their domes
    // `offset * sqrt(2)` from each other, and two domes need `2 * radius` to
    // clear. Violating this is not cosmetic: at 0.5 the chair needed an extra
    // module, collided in 30 places, and its loops would not close.
    const adjacentSeparation = SIDE_CONNECTOR_RADIAL_OFFSET * Math.SQRT2;
    expect(adjacentSeparation).toBeGreaterThanOrEqual(2 * HEMISPHERE_RADIUS);
  });

  it('keeps the dome reaching its own rod', () => {
    // The upper bound of the same window: past this the connector floats free
    // of the rod it is supposed to be mounted on.
    expect(SIDE_CONNECTOR_RADIAL_OFFSET).toBeLessThanOrEqual(HEMISPHERE_RADIUS + ROD_RADIUS);
  });
});
