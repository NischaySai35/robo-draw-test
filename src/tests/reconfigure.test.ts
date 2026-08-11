import { describe, expect, it } from 'vitest';
import { createModule } from '../kinematics/factory';
import { planReconfiguration, weldKey, type WeldPair } from '../kinematics/reconfigure';
import { IDENTITY_POSE } from '../kinematics/frame';
import { connectedComponents } from '../kinematics/assemblyGraph';
import type { Assembly, Connector, Module } from '../types/module';

function weld(a: Connector, b: Connector): void {
  a.locked = true;
  b.locked = true;
  a.connectedTo = b.id;
  b.connectedTo = a.id;
}

/** An open chain of `n` modules, welded end to end. */
function chainOf(n: number): { assembly: Assembly; modules: Module[] } {
  const modules = Array.from({ length: n }, () => createModule(IDENTITY_POSE));
  for (let i = 0; i < n - 1; i += 1) weld(modules[i]!.connectorB, modules[i + 1]!.connectorA);
  return {
    assembly: {
      modules: Object.fromEntries(modules.map((m) => [m.id, m])),
      edges: modules.slice(0, -1).map((m, i) => ({ a: m.connectorB.id, b: modules[i + 1]!.connectorA.id })),
    },
    modules,
  };
}

const existingWelds = (assembly: Assembly): WeldPair[] =>
  assembly.edges.map((e) => [e.a, e.b] as WeldPair);

describe('planReconfiguration', () => {
  it('returns an empty plan when the topology is already the target', () => {
    const { assembly } = chainOf(3);
    const plan = planReconfiguration(assembly, existingWelds(assembly));
    expect(plan.found).toBe(true);
    expect(plan.steps).toHaveLength(0);
  });

  it('closes an open chain into a ring with a single weld', { timeout: 30000 }, () => {
    const { assembly, modules } = chainOf(4);
    const target: WeldPair[] = [
      ...existingWelds(assembly),
      [modules[3]!.connectorB.id, modules[0]!.connectorA.id],
    ];

    const plan = planReconfiguration(assembly, target);
    expect(plan.found).toBe(true);
    expect(plan.steps).toHaveLength(1);
    expect(plan.steps[0]!.action).toBe('weld');
    // The weld creates a loop, and the plan is only valid if that loop closes.
    expect(plan.steps[0]!.loopError).toBeLessThan(1e-4);
  });

  it('opens a ring back into a chain with a single unweld', () => {
    const { assembly, modules } = chainOf(4);
    weld(modules[3]!.connectorB, modules[0]!.connectorA);
    assembly.edges.push({ a: modules[3]!.connectorB.id, b: modules[0]!.connectorA.id });

    const target = existingWelds(assembly).slice(0, 3);
    const plan = planReconfiguration(assembly, target);
    expect(plan.found).toBe(true);
    expect(plan.steps).toHaveLength(1);
    expect(plan.steps[0]!.action).toBe('unweld');
  });

  it('re-routes a ring through a side connector, releasing before re-attaching', { timeout: 30000 }, () => {
    const { assembly, modules } = chainOf(4);
    weld(modules[3]!.connectorB, modules[0]!.connectorA);
    assembly.edges.push({ a: modules[3]!.connectorB.id, b: modules[0]!.connectorA.id });

    // Same modules, same closing module -- but the ring now shuts through a
    // connector on module 0's big rod instead of its chain end.
    const target: WeldPair[] = [
      ...existingWelds(assembly).slice(0, 3),
      [modules[3]!.connectorB.id, modules[0]!.sides[0]!.id],
    ];

    const plan = planReconfiguration(assembly, target);
    expect(plan.found).toBe(true);
    expect(plan.steps).toHaveLength(2);
    // Order is forced, not incidental: module 3's connector B is occupied by
    // the old weld, so it cannot take the new one until the old one releases.
    expect(plan.steps[0]!.action).toBe('unweld');
    expect(plan.steps[1]!.action).toBe('weld');
    expect(plan.steps[1]!.loopError).toBeLessThan(1e-4);
  });

  it('refuses an unweld that would strand a piece', () => {
    // Removing the middle weld of a 3-chain leaves two disconnected halves --
    // in the air, on real hardware.
    const { assembly } = chainOf(3);
    const target = [existingWelds(assembly)[0]!];

    const plan = planReconfiguration(assembly, target);
    expect(plan.found).toBe(false);
    expect(plan.reason).toBeTruthy();
  });

  it('allows the same split when the caller says splitting is intended', () => {
    const { assembly } = chainOf(3);
    const target = [existingWelds(assembly)[0]!];

    const plan = planReconfiguration(assembly, target, { allowSplit: true });
    expect(plan.found).toBe(true);
    expect(plan.steps).toHaveLength(1);
    expect(plan.steps[0]!.action).toBe('unweld');
  });

  it('emits joint angles with every step, so the plan is executable', { timeout: 30000 }, () => {
    const { assembly, modules } = chainOf(4);
    const target: WeldPair[] = [
      ...existingWelds(assembly),
      [modules[3]!.connectorB.id, modules[0]!.connectorA.id],
    ];

    const plan = planReconfiguration(assembly, target);
    const step = plan.steps[0]!;
    // One entry per module, six rods each -- drivable straight to hardware.
    expect(Object.keys(step.angles)).toHaveLength(4);
    for (const angles of Object.values(step.angles)) expect(angles).toHaveLength(6);
  });

  it('reports honestly instead of throwing when a target cannot be reached', { timeout: 30000 }, () => {
    // Ask for a weld between two connectors on the same module: the chain
    // cannot fold that far onto itself within its joint limits.
    const { assembly, modules } = chainOf(2);
    const target: WeldPair[] = [
      ...existingWelds(assembly),
      [modules[0]!.connectorA.id, modules[0]!.sides[0]!.id],
    ];

    const plan = planReconfiguration(assembly, target, { maxExpansions: 40 });
    expect(plan.found).toBe(false);
    expect(plan.reason).toContain('operations');
  });

  it('keeps the structure connected at every step of a plan it accepts', { timeout: 30000 }, () => {
    const { assembly, modules } = chainOf(4);
    weld(modules[3]!.connectorB, modules[0]!.connectorA);
    assembly.edges.push({ a: modules[3]!.connectorB.id, b: modules[0]!.connectorA.id });
    const target: WeldPair[] = [
      ...existingWelds(assembly).slice(0, 3),
      [modules[3]!.connectorB.id, modules[0]!.sides[0]!.id],
    ];

    const plan = planReconfiguration(assembly, target);
    expect(plan.found).toBe(true);
    // Replay the plan and check connectivity after each operation.
    const live = JSON.parse(JSON.stringify(assembly)) as Assembly;
    const all = () => Object.values(live.modules).flatMap((m) => [m.connectorA, m.connectorB, ...m.sides]);
    for (const step of plan.steps) {
      const [a, b] = step.connectors;
      const ca = all().find((c) => c.id === a)!;
      const cb = all().find((c) => c.id === b)!;
      if (step.action === 'unweld') {
        ca.locked = false; cb.locked = false; ca.connectedTo = null; cb.connectedTo = null;
        live.edges = live.edges.filter((e) => weldKey(e.a, e.b) !== weldKey(a, b));
      } else {
        ca.locked = true; cb.locked = true; ca.connectedTo = cb.id; cb.connectedTo = ca.id;
        live.edges.push({ a, b });
      }
      expect(connectedComponents(live)).toHaveLength(1);
    }
  });
});
