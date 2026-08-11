import { describe, expect, it } from 'vitest';
import type { VoxelGraph } from '../types/voxel';
import {
  branchPoints,
  coordKey,
  voxelConnectedComponents,
  voxelDegree,
  voxelEdges,
  voxelPaths,
  voxelWorldCenter,
} from '../kinematics/voxelGraph';

function graphFromCoords(coords: [number, number, number][]): VoxelGraph {
  const voxels: VoxelGraph['voxels'] = {};
  for (const coord of coords) {
    const id = coordKey(coord);
    voxels[id] = { id, coord };
  }
  return { cellSize: 2, voxels };
}

describe('voxelEdges', () => {
  it('finds each adjacency exactly once for a straight line of cubes', () => {
    const graph = graphFromCoords([
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0],
    ]);
    const edges = voxelEdges(graph);
    expect(edges).toHaveLength(2);
  });

  it('reports no edges for two non-adjacent cubes', () => {
    const graph = graphFromCoords([
      [0, 0, 0],
      [5, 0, 0],
    ]);
    expect(voxelEdges(graph)).toHaveLength(0);
  });
});

describe('voxelDegree / branchPoints', () => {
  it('flags a cube with 3+ occupied neighbors as a branch point', () => {
    // A "T" shape: center cube with three arms.
    const graph = graphFromCoords([
      [0, 0, 0],
      [1, 0, 0],
      [-1, 0, 0],
      [0, 1, 0],
    ]);
    expect(voxelDegree(graph, [0, 0, 0])).toBe(3);
    const branches = branchPoints(graph);
    expect(branches).toHaveLength(1);
    expect(branches[0]).toEqual([0, 0, 0]);
  });

  it('reports no branch points along a simple path', () => {
    const graph = graphFromCoords([
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0],
    ]);
    expect(branchPoints(graph)).toHaveLength(0);
  });
});

describe('voxelConnectedComponents', () => {
  it('treats a straight run as one component', () => {
    const graph = graphFromCoords([
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0],
    ]);
    expect(voxelConnectedComponents(graph)).toHaveLength(1);
  });

  it('detects that removing a middle cube splits the structure into two pieces', () => {
    const graph = graphFromCoords([
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0],
    ]);
    const middleId = coordKey([1, 0, 0]);
    const components = voxelConnectedComponents(graph, middleId);
    expect(components).toHaveLength(2);
  });

  it('does not split when removing an end cube', () => {
    const graph = graphFromCoords([
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0],
    ]);
    const endId = coordKey([2, 0, 0]);
    expect(voxelConnectedComponents(graph, endId)).toHaveLength(1);
  });
});

describe('voxelWorldCenter', () => {
  it('scales grid coordinates by cell size', () => {
    expect(voxelWorldCenter([1, -2, 3], 2.5)).toEqual([2.5, -5, 7.5]);
  });
});

describe('voxelPaths', () => {
  it('collapses a straight run of cubes into a single path spanning all of them', () => {
    const graph = graphFromCoords([
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0],
      [3, 0, 0],
      [4, 0, 0],
    ]);
    const paths = voxelPaths(graph);
    expect(paths).toHaveLength(1);
    expect(paths[0]).toHaveLength(5);
  });

  it('splits into one path per arm at a branch point, not one per edge', () => {
    // A "T": center with three arms.
    const graph = graphFromCoords([
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0], // +x arm, 2 cubes past center
      [-1, 0, 0], // -x arm, 1 cube
      [0, 1, 0], // +y arm, 1 cube
    ]);
    const paths = voxelPaths(graph);
    expect(paths).toHaveLength(3);
    const lengths = paths.map((p) => p.length).sort();
    // center->(-x arm)=2 pts, center->(+y arm)=2 pts, center->+x arm through 2 cubes=3 pts
    expect(lengths).toEqual([2, 2, 3]);
  });

  it('contributes no path for an isolated single cube', () => {
    const graph = graphFromCoords([[0, 0, 0]]);
    expect(voxelPaths(graph)).toHaveLength(0);
  });

  it('walks a closed loop as a single path back to its start', () => {
    const graph = graphFromCoords([
      [0, 0, 0],
      [1, 0, 0],
      [1, 1, 0],
      [0, 1, 0],
    ]);
    const paths = voxelPaths(graph);
    expect(paths).toHaveLength(1);
    expect(paths[0]).toHaveLength(5); // 4 cubes + repeated start closing the loop
    expect(coordKey(paths[0]![0]!)).toBe(coordKey(paths[0]![paths[0]!.length - 1]!));
  });
});
