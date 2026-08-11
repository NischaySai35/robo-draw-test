import { describe, expect, it } from 'vitest';
import { createModule } from '../kinematics/factory';
import { computeAssemblyKinematics, connectorPose } from '../kinematics/assemblyGraph';
import {
  measureLoopError,
  solveConstrained,
  solveLoopClosure,
  withLoopsClosed,
} from '../kinematics/loopClosure';
import { useAssemblyStore } from '../state/assemblyStore';
import { IDENTITY_POSE, outwardNormal, worldPosition } from '../kinematics/frame';
import type { Assembly, Connector, Module, Pose } from '../types/module';

function weld(a: Connector, b: Connector): void {
  a.locked = true;
  b.locked = true;
  a.connectedTo = b.id;
  b.connectedTo = a.id;
}

function assemblyOf(...modules: Module[]): Assembly {
  return {
    modules: Object.fromEntries(modules.map((m) => [m.id, m])),
    edges: [],
  };
}

/** Two modules welded at BOTH ends -- the smallest possible closed loop. */
function twoModuleLoop(): Assembly {
  const m1 = createModule(IDENTITY_POSE);
  const m2 = createModule(IDENTITY_POSE);
  weld(m1.connectorB, m2.connectorA);
  weld(m1.connectorA, m2.connectorB);
  return assemblyOf(m1, m2);
}

/** Largest position gap across every weld, once placements are derived. */
function worstWeldGap(assembly: Assembly): { position: number; normalDot: number } {
  const { transforms } = computeAssemblyKinematics(assembly);
  const seen = new Set<string>();
  let position = 0;
  let normalDot = -1;

  for (const module of Object.values(assembly.modules)) {
    for (const connector of [module.connectorA, module.connectorB, ...module.sides]) {
      if (!connector.locked || !connector.connectedTo) continue;
      const key = [connector.id, connector.connectedTo].sort().join('|');
      if (seen.has(key)) continue;
      seen.add(key);

      const partner = Object.values(assembly.modules)
        .flatMap((m) => [m.connectorA, m.connectorB, ...m.sides])
        .find((c) => c.id === connector.connectedTo)!;

      const poseA = connectorPose(transforms.get(connector.moduleId)!, connector.end);
      const poseB = connectorPose(transforms.get(partner.moduleId)!, partner.end);
      position = Math.max(position, worldPosition(poseA).distanceTo(worldPosition(poseB)));
      normalDot = Math.max(normalDot, outwardNormal(poseA).dot(outwardNormal(poseB)));
    }
  }
  return { position, normalDot };
}

describe('cut-edge detection', () => {
  it('reports no cut edges for a tree-shaped assembly', () => {
    const m1 = createModule(IDENTITY_POSE);
    const m2 = createModule(IDENTITY_POSE);
    weld(m1.connectorB, m2.connectorA);
    expect(computeAssemblyKinematics(assemblyOf(m1, m2)).cutEdges).toHaveLength(0);
  });

  it('reports exactly one cut edge for one independent loop', () => {
    expect(computeAssemblyKinematics(twoModuleLoop()).cutEdges).toHaveLength(1);
  });

  it('counts a side-connector branch as a tree edge, not a loop', () => {
    // Branching is not cycling -- a T-junction has no independent loop.
    const trunk = createModule(IDENTITY_POSE);
    const armA = createModule(IDENTITY_POSE);
    const armB = createModule(IDENTITY_POSE);
    weld(trunk.sides[0]!, armA.connectorA);
    weld(trunk.sides[1]!, armB.connectorA);
    expect(computeAssemblyKinematics(assemblyOf(trunk, armA, armB)).cutEdges).toHaveLength(0);
  });
});

describe('assembly store integration', () => {
  function reset() {
    useAssemblyStore.setState({
      assembly: { modules: {}, edges: [] },
      undoStack: [],
      redoStack: [],
      loopClosure: null,
    });
  }

  it('does not run the solver for a weld that only grows the tree', () => {
    reset();
    const store = useAssemblyStore.getState();
    const a = store.addModule();
    const b = store.addModule();
    const assembly = useAssemblyStore.getState().assembly;
    store.connectConnectors(assembly.modules[a]!.connectorB.id, assembly.modules[b]!.connectorA.id);

    // No loop was created, so there is nothing to report.
    expect(useAssemblyStore.getState().loopClosure).toBeNull();
  });

  it('solves and applies angles when a weld closes a loop', () => {
    reset();
    const store = useAssemblyStore.getState();
    const a = store.addModule();
    const b = store.addModule();

    const first = useAssemblyStore.getState().assembly;
    store.connectConnectors(first.modules[a]!.connectorB.id, first.modules[b]!.connectorA.id);

    const second = useAssemblyStore.getState().assembly;
    store.connectConnectors(second.modules[a]!.connectorA.id, second.modules[b]!.connectorB.id);

    const report = useAssemblyStore.getState().loopClosure;
    expect(report).not.toBeNull();
    expect(report!.loopCount).toBe(1);
    expect(report!.converged).toBe(true);

    // The angles the solver found were actually written back, so the rendered
    // structure matches what the data claims.
    expect(worstWeldGap(useAssemblyStore.getState().assembly).position).toBeLessThan(1e-4);
  });

  it('clears a stale report when a weld is broken', () => {
    reset();
    const store = useAssemblyStore.getState();
    const a = store.addModule();
    const b = store.addModule();
    const first = useAssemblyStore.getState().assembly;
    store.connectConnectors(first.modules[a]!.connectorB.id, first.modules[b]!.connectorA.id);
    const second = useAssemblyStore.getState().assembly;
    store.connectConnectors(second.modules[a]!.connectorA.id, second.modules[b]!.connectorB.id);
    expect(useAssemblyStore.getState().loopClosure).not.toBeNull();

    const third = useAssemblyStore.getState().assembly;
    store.disconnectConnector(third.modules[a]!.connectorA.id);
    expect(useAssemblyStore.getState().loopClosure).toBeNull();
  });

  it('closeLoops is a no-op on a tree', () => {
    reset();
    const store = useAssemblyStore.getState();
    store.addModule();
    expect(store.closeLoops()).toBeNull();
  });
});

describe('solveConstrained (P1 — constrained IK)', () => {
  /** Where a connector currently sits, in the given assembly. */
  function poseOf(assembly: Assembly, moduleId: string, end: 'A' | 'B') {
    const { transforms } = computeAssemblyKinematics(assembly);
    return connectorPose(transforms.get(moduleId)!, end);
  }

  function applyAngles(assembly: Assembly, angles: Map<string, number[]>): Assembly {
    const next = JSON.parse(JSON.stringify(assembly)) as Assembly;
    for (const [moduleId, rodAngles] of angles) {
      next.modules[moduleId]!.rods.forEach((rod, i) => { rod.angle = rodAngles[i]!; });
    }
    return next;
  }

  it('drives a free chain\'s end connector toward a reachable target', () => {
    const m1 = createModule(IDENTITY_POSE);
    const m2 = createModule(IDENTITY_POSE);
    weld(m1.connectorB, m2.connectorA);
    const assembly = assemblyOf(m1, m2);

    const start = poseOf(assembly, m2.id, 'B');
    // Somewhere off the straight-line rest pose, but well within reach.
    const target: Pose = {
      position: [start.position[0] - 2, start.position[1] + 1.5, start.position[2] + 1],
      quaternion: start.quaternion,
    };

    const startDistance = worldPosition(start).distanceTo(worldPosition(target));
    const { angles, report } = solveConstrained(
      assembly,
      [{ moduleId: m2.id, end: 'B', target, positionOnly: true }],
      { restarts: 2 },
    );

    expect(report.goalError).toBeLessThan(startDistance * 0.1);
    // And the angles really do put it there, not just the report claiming so.
    const solved = applyAngles(assembly, angles);
    expect(worldPosition(poseOf(solved, m2.id, 'B')).distanceTo(worldPosition(target)))
      .toBeCloseTo(report.goalError, 6);
  });

  it('holds every loop closed while chasing a goal', () => {
    const loop = twoModuleLoop();
    const closed = withLoopsClosed(loop).assembly;
    const [first, second] = Object.values(closed.modules);

    // Ask a side connector on the closed ring to move somewhere it cannot go.
    const current = poseOf(closed, second!.id, 'B');
    const target: Pose = {
      position: [current.position[0] + 40, current.position[1] + 40, current.position[2] + 40],
      quaternion: current.quaternion,
    };

    const { angles, report } = solveConstrained(
      closed,
      [{ moduleId: second!.id, end: 'B', target, positionOnly: true }],
      { restarts: 0 },
    );

    // The goal is hopeless, but that must not cost us the weld: loops outrank
    // goals, so the structure stays physically consistent and the target is
    // simply missed.
    expect(report.goalError).toBeGreaterThan(1);
    expect(report.converged).toBe(true);
    expect(worstWeldGap(applyAngles(closed, angles)).position).toBeLessThan(1e-3);
    expect(first).toBeDefined();
  });

  it('warm-starts: re-solving an already-solved goal costs almost nothing', () => {
    const m1 = createModule(IDENTITY_POSE);
    const m2 = createModule(IDENTITY_POSE);
    weld(m1.connectorB, m2.connectorA);
    const assembly = assemblyOf(m1, m2);

    // Build the target from a pose the chain provably can strike, by bending it
    // and reading off where the end lands. An arbitrary point in space may be
    // unreachable, in which case neither solve converges and comparing their
    // iteration counts says nothing.
    const bent = JSON.parse(JSON.stringify(assembly)) as Assembly;
    bent.modules[m1.id]!.rods[1]!.angle = 0.35;
    bent.modules[m2.id]!.rods[4]!.angle = -0.25;
    const target = poseOf(bent, m2.id, 'B');
    const goal = { moduleId: m2.id, end: 'B' as const, target, positionOnly: true };

    const first = solveConstrained(assembly, [goal], { restarts: 0 });
    expect(first.report.goalError).toBeLessThan(1e-4);

    const warm = solveConstrained(applyAngles(assembly, first.angles), [goal], { restarts: 0 });

    // Seeded at the answer, the warm solve has essentially nothing left to do --
    // this is what makes interactive dragging viable.
    expect(warm.report.iterations).toBeLessThanOrEqual(1);
    expect(first.report.iterations).toBeGreaterThan(warm.report.iterations);
  });

  it('respects joint limits -- a pinned structure cannot chase anything', () => {
    const m1 = createModule(IDENTITY_POSE);
    const m2 = createModule(IDENTITY_POSE);
    weld(m1.connectorB, m2.connectorA);
    const assembly = assemblyOf(m1, m2);
    for (const module of Object.values(assembly.modules)) {
      for (const rod of module.rods) { rod.min = 0; rod.max = 0; rod.angle = 0; }
    }

    const start = poseOf(assembly, m2.id, 'B');
    const target: Pose = {
      position: [start.position[0], start.position[1] + 3, start.position[2]],
      quaternion: start.quaternion,
    };
    const { report } = solveConstrained(
      assembly,
      [{ moduleId: m2.id, end: 'B', target, positionOnly: true }],
      { restarts: 0 },
    );

    // Nothing can move, so the goal stays exactly as far away as it started.
    expect(report.goalError).toBeCloseTo(3, 3);
    // No loops to break, so this is not an over-constrained structure.
    expect(report.overConstrained).toBe(false);
  });
});

describe('measureLoopError', () => {
  it('reports zero error and full mobility for a tree', () => {
    const m1 = createModule(IDENTITY_POSE);
    const m2 = createModule(IDENTITY_POSE);
    weld(m1.connectorB, m2.connectorA);
    const measured = measureLoopError(assemblyOf(m1, m2));
    expect(measured.loopCount).toBe(0);
    expect(measured.maxPositionError).toBe(0);
    expect(measured.nominalMobility).toBe(12);
  });

  it('measures the violation of an unsolved loop without solving it', () => {
    const measured = measureLoopError(twoModuleLoop());
    expect(measured.loopCount).toBe(1);
    expect(measured.maxPositionError).toBeGreaterThan(7);
    // Gruebler-Kutzbach: 12 joints minus 6 for the one rigid-weld loop.
    expect(measured.nominalMobility).toBe(6);
  });

  it('agrees with the solver once the loop is closed', () => {
    const { assembly } = withLoopsClosed(twoModuleLoop());
    expect(measureLoopError(assembly).maxPositionError).toBeLessThan(1e-4);
  });
});

describe('solveLoopClosure', () => {
  it('leaves a tree assembly untouched and reports full mobility', () => {
    const m1 = createModule(IDENTITY_POSE);
    const m2 = createModule(IDENTITY_POSE);
    weld(m1.connectorB, m2.connectorA);

    const { report } = solveLoopClosure(assemblyOf(m1, m2));
    expect(report.loopCount).toBe(0);
    expect(report.converged).toBe(true);
    expect(report.iterations).toBe(0);
    // 2 modules x 6 rods, nothing constrained away.
    expect(report.mobility).toBe(12);
  });

  it('closes the two-module double weld that the tree traversal leaves 7.58 units apart', () => {
    const before = twoModuleLoop();
    // Establish the bug this phase exists to fix: the loop-closing weld is
    // ignored entirely by tree propagation.
    expect(worstWeldGap(before).position).toBeGreaterThan(7);

    const { assembly: after, report } = withLoopsClosed(before);

    expect(report.loopCount).toBe(1);
    expect(report.constraintCount).toBe(6);
    expect(report.converged).toBe(true);
    expect(report.overConstrained).toBe(false);

    // The P0 exit criterion.
    const gap = worstWeldGap(after);
    expect(gap.position).toBeLessThan(1e-4);
    // Every weld still means "coincident, facing opposite ways".
    expect(gap.normalDot).toBeCloseTo(-1, 3);
  });

  it('reports mobility below the raw joint count once a loop constrains the structure', () => {
    const { report } = solveLoopClosure(twoModuleLoop());
    // 12 joints, one rigid weld loop worth up to 6 constraints.
    expect(report.jointCount).toBe(12);
    expect(report.mobility).toBeLessThan(12);
    expect(report.mobility).toBeGreaterThanOrEqual(0);
  });

  it('does not mutate the assembly it is given', () => {
    const assembly = twoModuleLoop();
    const anglesBefore = Object.values(assembly.modules).flatMap((m) => m.rods.map((r) => r.angle));
    solveLoopClosure(assembly);
    const anglesAfter = Object.values(assembly.modules).flatMap((m) => m.rods.map((r) => r.angle));
    expect(anglesAfter).toEqual(anglesBefore);
  });

  it('closes a 4-module ring', () => {
    const mods = Array.from({ length: 4 }, () => createModule(IDENTITY_POSE));
    mods.forEach((m, i) => weld(m.connectorB, mods[(i + 1) % 4]!.connectorA));
    const { assembly, report } = withLoopsClosed(assemblyOf(...mods));

    expect(report.loopCount).toBe(1);
    expect(report.converged).toBe(true);
    expect(worstWeldGap(assembly).position).toBeLessThan(1e-4);
  });

  it('handles two independent loops at once', () => {
    // A figure-eight: two modules form a ring, and a two-module bridge spans
    // between side connectors on opposite faces to close a second, independent
    // loop. A single-module bridge between the SAME face on each (UP to UP) is
    // near the edge of feasible and needs an impractical number of restarts --
    // worth knowing, but not what this test is for.
    const a = createModule(IDENTITY_POSE);
    const b = createModule(IDENTITY_POSE);
    const bridge1 = createModule(IDENTITY_POSE);
    const bridge2 = createModule(IDENTITY_POSE);
    weld(a.connectorB, b.connectorA);
    weld(b.connectorB, a.connectorA);
    weld(bridge1.connectorB, bridge2.connectorA);
    weld(a.sides[0]!, bridge1.connectorA);       // UP
    weld(bridge2.connectorB, b.sides[2]!);       // DOWN

    const { assembly, report } = withLoopsClosed(assemblyOf(a, b, bridge1, bridge2));

    expect(report.loopCount).toBe(2);
    expect(report.constraintCount).toBe(12);
    expect(report.converged).toBe(true);
    expect(report.jointCount).toBe(24);
    // Rank is bounded by the constraint count, so mobility can never fall below
    // joints - 6 per loop. It can sit ABOVE it when constraints are partly
    // redundant, which is exactly what an over-constrained structure looks like
    // -- so this is a bound, not an equality.
    expect(report.mobility).toBeGreaterThanOrEqual(24 - 12);
    expect(report.mobility).toBeLessThan(24);
    expect(worstWeldGap(assembly).position).toBeLessThan(1e-4);
  });

  it('is deterministic -- the same assembly solves identically twice', () => {
    // Restarts are randomized, so this guards the seeding.
    const first = solveLoopClosure(twoModuleLoop()).report;
    const second = solveLoopClosure(twoModuleLoop()).report;
    expect(second.residualNorm).toBeCloseTo(first.residualNorm, 12);
    expect(second.iterations).toBe(first.iterations);
  });

  it('reports an unreachable loop as over-constrained instead of throwing', () => {
    // Pin every joint to a single angle: with no freedom left, a loop that is
    // not already closed cannot be closed.
    const assembly = twoModuleLoop();
    for (const module of Object.values(assembly.modules)) {
      for (const rod of module.rods) {
        rod.min = 0;
        rod.max = 0;
        rod.angle = 0;
      }
    }

    const { report } = solveLoopClosure(assembly, { maxIterations: 30 });
    expect(report.loopCount).toBe(1);
    expect(report.converged).toBe(false);
    expect(report.overConstrained).toBe(true);
    expect(report.maxPositionError).toBeGreaterThan(1);
  });
});
