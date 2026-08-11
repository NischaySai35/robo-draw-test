/**
 * Voxel-graph state for the cube-builder mode. Structural undo/redo mirrors
 * `assemblyStore`'s pattern: snapshot before each structural mutation.
 */
import { create } from 'zustand';
import type { FaceDirection, VoxelCoord, VoxelGraph, VoxelId } from '../types/voxel';
import { coordKey, isOccupied, neighborCoord } from '../kinematics/voxelGraph';

const DEFAULT_CELL_SIZE = 2.2;

interface HistoryEntry {
  graph: VoxelGraph;
}

interface VoxelState {
  graph: VoxelGraph;
  selectedVoxelId: VoxelId | null;
  undoStack: HistoryEntry[];
  redoStack: HistoryEntry[];

  addVoxel: (coord: VoxelCoord) => void;
  /** Spawns up to `count` cubes in a row from `fromCoord`'s face, stopping at the first occupied cell. Returns how many were added. */
  addAdjacent: (fromCoord: VoxelCoord, direction: FaceDirection, count: number) => number;
  removeVoxel: (id: VoxelId) => void;
  /** Removes several voxels as a single undo step -- used by cascade deletes. */
  removeVoxels: (ids: VoxelId[]) => void;
  moveVoxel: (id: VoxelId, newCoord: VoxelCoord) => boolean;
  selectVoxel: (id: VoxelId | null) => void;
  clearAll: () => void;

  undo: () => void;
  redo: () => void;
}

function cloneGraph(graph: VoxelGraph): VoxelGraph {
  return JSON.parse(JSON.stringify(graph)) as VoxelGraph;
}

export const useVoxelStore = create<VoxelState>((set, get) => {
  function snapshot() {
    const { graph, undoStack } = get();
    set({ undoStack: [...undoStack, { graph: cloneGraph(graph) }], redoStack: [] });
  }

  return {
    // Cube-builder always starts with a single seed cube at the origin.
    graph: { cellSize: DEFAULT_CELL_SIZE, voxels: { '0,0,0': { id: '0,0,0', coord: [0, 0, 0] } } },
    selectedVoxelId: null,
    undoStack: [],
    redoStack: [],

    addVoxel: (coord) => {
      const { graph } = get();
      if (isOccupied(graph, coord)) return;
      snapshot();
      const id = coordKey(coord);
      set((state) => ({
        graph: { ...state.graph, voxels: { ...state.graph.voxels, [id]: { id, coord } } },
      }));
    },

    addAdjacent: (fromCoord, direction, count) => {
      const { graph } = get();
      let cursor = fromCoord;
      const toAdd: VoxelCoord[] = [];
      for (let i = 0; i < Math.max(1, count); i += 1) {
        cursor = neighborCoord(cursor, direction);
        if (isOccupied(graph, cursor) || toAdd.some((c) => coordKey(c) === coordKey(cursor))) break;
        toAdd.push(cursor);
      }
      if (toAdd.length === 0) return 0;
      snapshot();
      set((state) => {
        const voxels = { ...state.graph.voxels };
        for (const coord of toAdd) {
          const id = coordKey(coord);
          voxels[id] = { id, coord };
        }
        return { graph: { ...state.graph, voxels } };
      });
      return toAdd.length;
    },

    removeVoxel: (id) => {
      snapshot();
      set((state) => {
        const voxels = { ...state.graph.voxels };
        delete voxels[id];
        return {
          graph: { ...state.graph, voxels },
          selectedVoxelId: state.selectedVoxelId === id ? null : state.selectedVoxelId,
        };
      });
    },

    removeVoxels: (ids) => {
      if (ids.length === 0) return;
      snapshot();
      set((state) => {
        const voxels = { ...state.graph.voxels };
        ids.forEach((id) => delete voxels[id]);
        return {
          graph: { ...state.graph, voxels },
          selectedVoxelId: ids.includes(state.selectedVoxelId ?? '') ? null : state.selectedVoxelId,
        };
      });
    },

    moveVoxel: (id, newCoord) => {
      const { graph } = get();
      const voxel = graph.voxels[id];
      if (!voxel) return false;
      const newId = coordKey(newCoord);
      if (newId === id) return false;
      if (isOccupied(graph, newCoord)) return false;

      const totalVoxels = Object.keys(graph.voxels).length;
      if (totalVoxels > 1) {
        // A move must keep the cube face-adjacent to at least one other cube
        // (unless it's the only cube in the graph) -- otherwise it silently
        // floats off as a disconnected island, which is what "move a cube
        // only if that doesn't break face-adjacency" rules out.
        const hasNeighbor = (['+x', '-x', '+y', '-y', '+z', '-z'] as FaceDirection[]).some((dir) =>
          isOccupied(graph, neighborCoord(newCoord, dir)),
        );
        if (!hasNeighbor) return false;
      }

      snapshot();
      set((state) => {
        const voxels = { ...state.graph.voxels };
        delete voxels[id];
        voxels[newId] = { id: newId, coord: newCoord };
        return {
          graph: { ...state.graph, voxels },
          selectedVoxelId: state.selectedVoxelId === id ? newId : state.selectedVoxelId,
        };
      });
      return true;
    },

    selectVoxel: (id) => set({ selectedVoxelId: id }),

    clearAll: () => {
      snapshot();
      // Reseed the origin cube -- otherwise there'd be no cube left to click a face on.
      set((state) => ({
        graph: { ...state.graph, voxels: { '0,0,0': { id: '0,0,0', coord: [0, 0, 0] } } },
        selectedVoxelId: null,
      }));
    },

    undo: () => {
      const { undoStack, redoStack, graph } = get();
      const entry = undoStack[undoStack.length - 1];
      if (!entry) return;
      set({
        graph: entry.graph,
        undoStack: undoStack.slice(0, -1),
        redoStack: [...redoStack, { graph: cloneGraph(graph) }],
        selectedVoxelId: null,
      });
    },

    redo: () => {
      const { undoStack, redoStack, graph } = get();
      const entry = redoStack[redoStack.length - 1];
      if (!entry) return;
      set({
        graph: entry.graph,
        redoStack: redoStack.slice(0, -1),
        undoStack: [...undoStack, { graph: cloneGraph(graph) }],
        selectedVoxelId: null,
      });
    },
  };
});
