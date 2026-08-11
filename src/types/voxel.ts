/**
 * Types for the Phase 3 cube-builder mode. A voxel graph is deliberately
 * simpler than the module `Assembly` graph: adjacency is implicit in grid
 * occupancy (two voxels are neighbors iff their coordinates differ by 1 on
 * exactly one axis), so there's no separate edge list to keep in sync --
 * "a face is either open or connected to exactly one neighbor" falls out of
 * the grid geometry for free, with no ambiguous state representable at all.
 */

export type VoxelCoord = readonly [number, number, number];
export type VoxelId = string;

export interface Voxel {
  id: VoxelId;
  coord: VoxelCoord;
}

export interface VoxelGraph {
  /** World-space size of one grid cell (also the fixed cube-to-cube center distance). */
  cellSize: number;
  voxels: Record<VoxelId, Voxel>;
}

export type FaceDirection = '+x' | '-x' | '+y' | '-y' | '+z' | '-z';

export const FACE_DIRECTIONS: FaceDirection[] = ['+x', '-x', '+y', '-y', '+z', '-z'];

export const FACE_NORMALS: Record<FaceDirection, VoxelCoord> = {
  '+x': [1, 0, 0],
  '-x': [-1, 0, 0],
  '+y': [0, 1, 0],
  '-y': [0, -1, 0],
  '+z': [0, 0, 1],
  '-z': [0, 0, -1],
};

/** An edge between two grid-adjacent occupied voxels -- the unit a module chain spans. */
export interface VoxelEdge {
  a: VoxelCoord;
  b: VoxelCoord;
}
