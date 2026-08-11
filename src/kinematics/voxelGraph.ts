/**
 * Pure grid/graph logic for the cube-builder: occupancy, adjacency, edges,
 * degree, and connected-component analysis. No Three.js, no store -- unit
 * tested directly, same convention as `kinematics/assemblyGraph.ts`.
 */
import type { FaceDirection, VoxelCoord, VoxelGraph, VoxelId } from '../types/voxel';
import { FACE_DIRECTIONS, FACE_NORMALS } from '../types/voxel';

export function coordKey(coord: VoxelCoord): VoxelId {
  return `${coord[0]},${coord[1]},${coord[2]}`;
}

export function neighborCoord(coord: VoxelCoord, direction: FaceDirection): VoxelCoord {
  const n = FACE_NORMALS[direction];
  return [coord[0] + n[0], coord[1] + n[1], coord[2] + n[2]];
}

export function isOccupied(graph: VoxelGraph, coord: VoxelCoord): boolean {
  return coordKey(coord) in graph.voxels;
}

export function occupiedNeighbors(graph: VoxelGraph, coord: VoxelCoord): FaceDirection[] {
  return FACE_DIRECTIONS.filter((dir) => isOccupied(graph, neighborCoord(coord, dir)));
}

export function voxelDegree(graph: VoxelGraph, coord: VoxelCoord): number {
  return occupiedNeighbors(graph, coord).length;
}

/** Voxels with 3+ occupied neighbors -- branch points a single 2-ended module chain can't fully represent. */
export function branchPoints(graph: VoxelGraph): VoxelCoord[] {
  return Object.values(graph.voxels)
    .filter((v) => voxelDegree(graph, v.coord) >= 3)
    .map((v) => v.coord);
}

/** Every grid-adjacency edge, deduplicated (each unordered pair appears once). */
export function voxelEdges(graph: VoxelGraph): { a: VoxelCoord; b: VoxelCoord }[] {
  const edges: { a: VoxelCoord; b: VoxelCoord }[] = [];
  const seen = new Set<string>();
  for (const voxel of Object.values(graph.voxels)) {
    for (const dir of FACE_DIRECTIONS) {
      const neighbor = neighborCoord(voxel.coord, dir);
      if (!isOccupied(graph, neighbor)) continue;
      const key = [coordKey(voxel.coord), coordKey(neighbor)].sort().join('|');
      if (seen.has(key)) continue;
      seen.add(key);
      edges.push({ a: voxel.coord, b: neighbor });
    }
  }
  return edges;
}

function edgeKey(a: VoxelCoord, b: VoxelCoord): string {
  return [coordKey(a), coordKey(b)].sort().join('|');
}

/**
 * Decomposes the voxel grid into maximal simple paths -- one module chain
 * per path, not one per edge. A path runs between two "special" voxels
 * (a leaf with 1 connection, or a branch point with 3+) through any number
 * of plain degree-2 voxels in between, including ones where the path turns
 * a corner (a degree-2 voxel can still have non-collinear neighbors). This
 * is what lets, say, a straight run of ten cubes become one chain sized for
 * the whole run instead of nine separate one-edge chains.
 *
 * A pure closed loop (every voxel degree-2, no special nodes at all) has no
 * natural start/end; it's walked once as its own path starting at an
 * arbitrary voxel. An isolated voxel (degree 0) contributes no path -- there's
 * nothing to bridge.
 */
export function voxelPaths(graph: VoxelGraph): VoxelCoord[][] {
  const voxels = Object.values(graph.voxels);
  const visitedEdges = new Set<string>();
  const paths: VoxelCoord[][] = [];

  function otherNeighbor(coord: VoxelCoord, exclude: VoxelCoord): VoxelCoord | null {
    for (const dir of occupiedNeighbors(graph, coord)) {
      const candidate = neighborCoord(coord, dir);
      if (coordKey(candidate) !== coordKey(exclude)) return candidate;
    }
    return null;
  }

  function walk(start: VoxelCoord, firstStep: VoxelCoord): VoxelCoord[] {
    const path: VoxelCoord[] = [start, firstStep];
    visitedEdges.add(edgeKey(start, firstStep));
    let prev = start;
    let current = firstStep;
    while (voxelDegree(graph, current) === 2) {
      const next = otherNeighbor(current, prev);
      if (!next || visitedEdges.has(edgeKey(current, next))) break;
      visitedEdges.add(edgeKey(current, next));
      path.push(next);
      prev = current;
      current = next;
    }
    return path;
  }

  const specialVoxels = voxels.filter((v) => voxelDegree(graph, v.coord) !== 2);
  for (const voxel of specialVoxels) {
    for (const dir of occupiedNeighbors(graph, voxel.coord)) {
      const neighbor = neighborCoord(voxel.coord, dir);
      if (visitedEdges.has(edgeKey(voxel.coord, neighbor))) continue;
      paths.push(walk(voxel.coord, neighbor));
    }
  }

  // Any remaining unvisited edges belong to pure closed loops (no special nodes).
  for (const voxel of voxels) {
    for (const dir of occupiedNeighbors(graph, voxel.coord)) {
      const neighbor = neighborCoord(voxel.coord, dir);
      if (visitedEdges.has(edgeKey(voxel.coord, neighbor))) continue;
      paths.push(walk(voxel.coord, neighbor));
    }
  }

  return paths;
}

/** World-space center of a grid cell. */
export function voxelWorldCenter(coord: VoxelCoord, cellSize: number): [number, number, number] {
  return [coord[0] * cellSize, coord[1] * cellSize, coord[2] * cellSize];
}

/**
 * Connected components of the voxel grid, optionally as if `excluding` were
 * already removed -- used to detect the "deleting this splits the structure"
 * ambiguous case that needs an orphan-vs-cascade decision.
 */
export function voxelConnectedComponents(graph: VoxelGraph, excluding?: VoxelId): VoxelId[][] {
  const ids = Object.keys(graph.voxels).filter((id) => id !== excluding);
  const idSet = new Set(ids);
  const visited = new Set<VoxelId>();
  const components: VoxelId[][] = [];

  for (const startId of ids) {
    if (visited.has(startId)) continue;
    const component: VoxelId[] = [];
    const queue: VoxelId[] = [startId];
    visited.add(startId);
    while (queue.length > 0) {
      const currentId = queue.shift()!;
      component.push(currentId);
      const voxel = graph.voxels[currentId];
      if (!voxel) continue;
      for (const dir of FACE_DIRECTIONS) {
        const neighborId = coordKey(neighborCoord(voxel.coord, dir));
        if (idSet.has(neighborId) && !visited.has(neighborId)) {
          visited.add(neighborId);
          queue.push(neighborId);
        }
      }
    }
    components.push(component);
  }
  return components;
}
