/**
 * Selection/UI-focus state -- kept separate from the assembly store so
 * selecting things doesn't churn undo history or assembly subscribers.
 */
import { create } from 'zustand';
import type { ConnectorId, ModuleId } from '../types/module';

export type SelectionKind = 'module' | 'rod' | 'connector' | null;

export interface RodSelection {
  moduleId: ModuleId;
  rodIndex: number;
}

interface SelectionState {
  selectedModuleIds: ModuleId[];
  selectedRod: RodSelection | null;
  selectedConnector: ConnectorId | null;

  selectModule: (moduleId: ModuleId, additive?: boolean) => void;
  selectRod: (moduleId: ModuleId, rodIndex: number) => void;
  selectConnector: (connectorId: ConnectorId) => void;
  clearSelection: () => void;
  isModuleSelected: (moduleId: ModuleId) => boolean;
}

export const useSelectionStore = create<SelectionState>((set, get) => ({
  selectedModuleIds: [],
  selectedRod: null,
  selectedConnector: null,

  selectModule: (moduleId, additive = false) => {
    set((state) => {
      if (additive) {
        const already = state.selectedModuleIds.includes(moduleId);
        return {
          selectedModuleIds: already
            ? state.selectedModuleIds.filter((id) => id !== moduleId)
            : [...state.selectedModuleIds, moduleId],
          selectedRod: null,
          selectedConnector: null,
        };
      }
      return { selectedModuleIds: [moduleId], selectedRod: null, selectedConnector: null };
    });
  },

  selectRod: (moduleId, rodIndex) => {
    set({ selectedModuleIds: [moduleId], selectedRod: { moduleId, rodIndex }, selectedConnector: null });
  },

  selectConnector: (connectorId) => {
    set((state) => ({
      selectedConnector: connectorId,
      selectedRod: null,
      selectedModuleIds: state.selectedModuleIds,
    }));
  },

  clearSelection: () => set({ selectedModuleIds: [], selectedRod: null, selectedConnector: null }),

  isModuleSelected: (moduleId) => get().selectedModuleIds.includes(moduleId),
}));
