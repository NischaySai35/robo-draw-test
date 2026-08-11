import { describe, expect, it } from 'vitest';
import { fitSkeleton, skeletonLoopCount, skeletonPaths, type ShapeSpec } from '../kinematics/skeletonFit';
import { CHAIR, TRIANGLE, TRIPOD } from '../kinematics/shapeLibrary';
import { computeAssemblyKinematics } from '../kinematics/assemblyGraph';
import { MODULE_CHAIN_LENGTH } from '../constants/geometry';
import { connectedComponents } from '../kinematics/assemblyGraph';

describe('skeletonPaths', () => {
  it('collapses a run through degree-2 nodes into a single path', () => {
    const line: ShapeSpec = {
      name: 'line',
      nodes: [
        { id: 'a', position: [0, 0, 0] },
        { id: 'b', position: [1, 0, 0] },
        { id: 'c', position: [2, 0, 0] },
        { id: 'd', position: [3, 0, 0] },
      ],
      edges: [{ from: 'a', to: 'b' }, { from: 'b', to: 'c' }, { from: 'c', to: 'd' }],
    };
    const paths = skeletonPaths(line);
    expect(paths).toHaveLength(1);
    expect(paths[0]).toEqual(['a', 'b', 'c', 'd']);
  });

  it('breaks into one path per arm at a junction', () => {
    expect(skeletonPaths(TRIPOD)).toHaveLength(3);
  });

  it('walks a junction-free ring as one path that returns to its start', () => {
    const paths = skeletonPaths(TRIANGLE);
    expect(paths).toHaveLength(1);
    expect(paths[0]![0]).toBe(paths[0]![paths[0]!.length - 1]);
  });
});

describe('skeletonLoopCount', () => {
  it('counts zero for a tree and one for a ring', () => {
    expect(skeletonLoopCount(TRIPOD)).toBe(0);
    expect(skeletonLoopCount(TRIANGLE)).toBe(1);
  });

  it('counts both cycles in a chair -- the seat frame and the back', () => {
    expect(skeletonLoopCount(CHAIR)).toBe(2);
  });
});

describe('fitSkeleton', () => {
  /** Independent cycles actually present in the built assembly's weld graph. */
  function assemblyLoops(spec: ShapeSpec) {
    return computeAssemblyKinematics(fitSkeleton(spec).assembly).cutEdges.length;
  }

  it('builds a tree shape as one connected assembly with no loops', () => {
    const result = fitSkeleton(TRIPOD);
    expect(result.unanchored).toHaveLength(0);
    expect(result.loopReport).toBeNull();
    expect(connectedComponents(result.assembly)).toHaveLength(1);
    expect(assemblyLoops(TRIPOD)).toBe(0);
  });

  it('closes a ring: the chain welds back to its own start', () => {
    const result = fitSkeleton(TRIANGLE);
    expect(assemblyLoops(TRIANGLE)).toBe(1);
    expect(result.loopReport?.converged).toBe(true);
    expect(result.loopReport!.maxPositionError).toBeLessThan(1e-4);
  });

  it('builds a chair: one connected structure with both cycles closed', () => {
    const result = fitSkeleton(CHAIR);

    expect(connectedComponents(result.assembly)).toHaveLength(1);
    expect(result.unanchored).toHaveLength(0);
    // One module per beam -- the sizing rule in shapeLibrary.ts holds.
    expect(result.moduleCount).toBe(CHAIR.edges.length);
    expect(result.loopReport?.converged).toBe(true);
    expect(result.loopReport!.maxPositionError).toBeLessThan(1e-4);
  });

  it('gives the assembly exactly as many loops as the skeleton has', () => {
    // The weld budget guards this. Joining k chain-ends at a node takes k-1
    // welds; one extra anywhere invents a cycle the shape never had, and each
    // invented cycle adds 6 constraints that over-constrain the structure into
    // not closing at all.
    for (const spec of [TRIPOD, TRIANGLE, CHAIR]) {
      expect(assemblyLoops(spec)).toBe(skeletonLoopCount(spec));
    }
  });

  it('spends more modules on a beam longer than one module can span', () => {
    const long: ShapeSpec = {
      name: 'long',
      nodes: [
        { id: 'a', position: [0, 0, 0] },
        { id: 'b', position: [MODULE_CHAIN_LENGTH * 2.5, 0, 0] },
      ],
      edges: [{ from: 'a', to: 'b' }],
    };
    const result = fitSkeleton(long);
    expect(result.moduleCount).toBe(3);
    expect(result.loopReport).toBeNull(); // a single beam has no cycle to close
    expect(connectedComponents(result.assembly)).toHaveLength(1);
  });

  it('does not mutate the shape spec it is given', () => {
    const before = JSON.stringify(CHAIR);
    fitSkeleton(CHAIR);
    expect(JSON.stringify(CHAIR)).toBe(before);
  });
});
