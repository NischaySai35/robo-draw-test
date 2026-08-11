import { describe, expect, it } from 'vitest';
import { createModule } from '../kinematics/factory';
import { computeModuleTransforms, localChainTransform } from '../kinematics/forwardKinematics';
import { IDENTITY_POSE, outwardNormal, worldPosition } from '../kinematics/frame';
import {
  anchorModuleId,
  computeAssemblyWorldTransforms,
  computeWeldAnchorPose,
  connectedComponents,
} from '../kinematics/assemblyGraph';
import type { Assembly } from '../types/module';
import {
  BIG_ROD_INDEX,
  HEMISPHERE_RADIUS,
  ROD_RADIUS,
  SIDE_CONNECTOR_RADIAL_OFFSET,
} from '../constants/geometry';

describe('computeModuleTransforms', () => {
  it('places connector A exactly at the given world pose', () => {
    const module = createModule(IDENTITY_POSE);
    const transforms = computeModuleTransforms(module.rods, IDENTITY_POSE);
    expect(transforms.connectorA).toEqual(IDENTITY_POSE);
  });

  it('gives connector B an outward normal opposite to a straight chain\'s connector A, with zero net offset at rest', () => {
    const module = createModule(IDENTITY_POSE);
    const transforms = computeModuleTransforms(module.rods, IDENTITY_POSE);
    // At rest (all angles 0) the chain is straight, so A and B outward normals
    // point in opposite directions along the same axis.
    const normalA = outwardNormal(transforms.connectorA);
    const normalB = outwardNormal(transforms.connectorB);
    expect(normalA.dot(normalB)).toBeCloseTo(-1, 5);
  });

  it('keeps connector B world position independent of home pose translation', () => {
    const module = createModule(IDENTITY_POSE);
    const local = localChainTransform(module.rods);
    const moved = computeModuleTransforms(module.rods, {
      position: [3, 1, -2],
      quaternion: [0, 0, 0, 1],
    });
    expect(moved.connectorB.position[0]).toBeCloseTo(3 + local.position[0], 5);
    expect(moved.connectorB.position[1]).toBeCloseTo(1 + local.position[1], 5);
    expect(moved.connectorB.position[2]).toBeCloseTo(-2 + local.position[2], 5);
  });
});

describe('big-rod side connectors', () => {
  it('points all 4 outward normals perpendicular to the rod axis, 90 degrees apart', () => {
    const module = createModule(IDENTITY_POSE);
    const t = computeModuleTransforms(module.rods, IDENTITY_POSE);
    const rodAxis = outwardNormal(t.rods[BIG_ROD_INDEX]);
    const normals = t.sides.map(outwardNormal);

    normals.forEach((n) => expect(n.dot(rodAxis)).toBeCloseTo(0, 5));
    // Consecutive sides are a quarter turn apart, and each is opposed by the one across from it.
    expect(normals[0]!.dot(normals[1]!)).toBeCloseTo(0, 5);
    expect(normals[0]!.dot(normals[2]!)).toBeCloseTo(-1, 5);
    expect(normals[1]!.dot(normals[3]!)).toBeCloseTo(-1, 5);
  });

  it('sits every lock face the same distance out from the big rod axis', () => {
    const module = createModule(IDENTITY_POSE);
    const t = computeModuleTransforms(module.rods, IDENTITY_POSE);
    const rodOrigin = worldPosition(t.rods[BIG_ROD_INDEX]);
    const rodAxis = outwardNormal(t.rods[BIG_ROD_INDEX]);

    for (const side of t.sides) {
      const offset = worldPosition(side).sub(rodOrigin);
      // Strip the along-the-rod part; what's left is the purely radial distance.
      const radial = offset.clone().sub(rodAxis.clone().multiplyScalar(offset.dot(rodAxis)));
      expect(radial.length()).toBeCloseTo(SIDE_CONNECTOR_RADIAL_OFFSET, 5);
    }
  });

  it('keeps the lock faces far enough out that they cannot slice through each other', () => {
    // Each lock face is a flat disc of HEMISPHERE_RADIUS standing perpendicular to
    // its own outward normal. Two adjacent discs are 90 degrees apart, so unless
    // the offset clears the disc radius the +X disc reaches past the +Y disc's
    // plane and they visibly intersect in an X. See SIDE_CONNECTOR_RADIAL_OFFSET.
    expect(SIDE_CONNECTOR_RADIAL_OFFSET).toBeGreaterThan(HEMISPHERE_RADIUS);
    // ...and close enough in that the dome still reaches the rod it mounts on.
    expect(SIDE_CONNECTOR_RADIAL_OFFSET).toBeLessThan(HEMISPHERE_RADIUS + ROD_RADIUS);
  });
});

describe('assembly graph traversal', () => {
  function twoModuleAssembly(): Assembly {
    const m1 = createModule(IDENTITY_POSE);
    const m2 = createModule({ position: [10, 10, 10], quaternion: [0, 0, 0, 1] });
    m1.connectorB.locked = true;
    m2.connectorA.locked = true;
    m1.connectorB.connectedTo = m2.connectorA.id;
    m2.connectorA.connectedTo = m1.connectorB.id;
    return {
      modules: { [m1.id]: m1, [m2.id]: m2 },
      edges: [{ a: m1.connectorB.id, b: m2.connectorA.id }],
    };
  }

  it('welds a locked connector pair at the same world position with opposed normals', () => {
    const assembly = twoModuleAssembly();
    const transforms = computeAssemblyWorldTransforms(assembly);
    const m1 = Object.values(assembly.modules)[0]!;
    const m2 = Object.values(assembly.modules)[1]!;
    const bPose = transforms.get(m1.id)!.connectorB;
    const aPose = transforms.get(m2.id)!.connectorA;

    expect(worldPosition(bPose).distanceTo(worldPosition(aPose))).toBeCloseTo(0, 5);
    expect(outwardNormal(bPose).dot(outwardNormal(aPose))).toBeCloseTo(-1, 5);
  });

  it('groups welded modules into one connected component', () => {
    const assembly = twoModuleAssembly();
    const components = connectedComponents(assembly);
    expect(components).toHaveLength(1);
    expect(components[0]).toHaveLength(2);
  });

  it('treats an unwelded module as its own connected component', () => {
    const m1 = createModule(IDENTITY_POSE);
    const m2 = createModule(IDENTITY_POSE);
    const assembly: Assembly = { modules: { [m1.id]: m1, [m2.id]: m2 }, edges: [] };
    expect(connectedComponents(assembly)).toHaveLength(2);
    expect(anchorModuleId(assembly, m2.id)).toBe(m2.id);
  });

  it('welds a module onto a SIDE connector, branching perpendicular to the host', () => {
    const host = createModule(IDENTITY_POSE);
    const branch = createModule({ position: [7, -3, 2], quaternion: [0, 0, 0, 1] });
    const assembly: Assembly = { modules: { [host.id]: host, [branch.id]: branch }, edges: [] };

    // UP is index 0 of SIDE_CONNECTOR_ENDS.
    const hostSide = host.sides[0]!;
    const weld = computeWeldAnchorPose(assembly, branch.connectorA.id, hostSide.id);
    expect(weld).not.toBeNull();
    branch.basePose = weld!.anchorPose;

    hostSide.locked = true;
    branch.connectorA.locked = true;
    hostSide.connectedTo = branch.connectorA.id;
    branch.connectorA.connectedTo = hostSide.id;
    assembly.edges.push({ a: hostSide.id, b: branch.connectorA.id });

    const transforms = computeAssemblyWorldTransforms(assembly);
    const sidePose = transforms.get(host.id)!.sides[0];
    const branchPose = transforms.get(branch.id)!.connectorA;

    // The graph must resolve the branch through the side connector, not through
    // the chain ends -- if `localConnectorTransform` fell back to connector B's
    // full-chain offset here, the branch would land a whole module-length away.
    expect(worldPosition(sidePose).distanceTo(worldPosition(branchPose))).toBeCloseTo(0, 4);
    expect(outwardNormal(sidePose).dot(outwardNormal(branchPose))).toBeCloseTo(-1, 4);
    expect(connectedComponents(assembly)).toHaveLength(1);
  });

  it('computeWeldAnchorPose produces a pose that, once applied, welds the two connectors', () => {
    const m1 = createModule(IDENTITY_POSE);
    const m2 = createModule({ position: [5, 5, 5], quaternion: [0, 0, 0, 1] });
    const assembly: Assembly = { modules: { [m1.id]: m1, [m2.id]: m2 }, edges: [] };

    const weld = computeWeldAnchorPose(assembly, m2.connectorA.id, m1.connectorB.id);
    expect(weld).not.toBeNull();
    m2.basePose = weld!.anchorPose;

    const transforms = computeAssemblyWorldTransforms(assembly);
    const bPose = transforms.get(m1.id)!.connectorB;
    const aPose = transforms.get(m2.id)!.connectorA;
    expect(worldPosition(bPose).distanceTo(worldPosition(aPose))).toBeCloseTo(0, 4);
    expect(outwardNormal(bPose).dot(outwardNormal(aPose))).toBeCloseTo(-1, 4);
  });
});
