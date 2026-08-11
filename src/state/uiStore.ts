/**
 * Transient UI state: inline warnings/toasts and the choice-dialog queue
 * used by destructive/ambiguous actions (delete a locked module, disconnect
 * a lock, delete a cube that would split the voxel graph). Kept out of the
 * assembly store so it never participates in undo/redo.
 */
import { create } from 'zustand';

export interface Warning {
  id: string;
  message: string;
  level: 'warning' | 'error';
}

export interface ChoiceOption {
  label: string;
  value: string;
  danger?: boolean;
}

export interface ChoiceRequest {
  id: string;
  message: string;
  options: ChoiceOption[];
  resolve: (value: string | null) => void;
}

export type AppMode = 'manual' | 'draw' | 'cube' | 'generate';

interface UIState {
  warnings: Warning[];
  choiceRequest: ChoiceRequest | null;
  /** Which top-level mode/window is showing -- manual editor, draw-to-build, or cube-builder. */
  mode: AppMode;

  pushWarning: (message: string, level?: Warning['level']) => void;
  dismissWarning: (id: string) => void;
  /** General 2+ option dialog (e.g. Cancel / Orphan / Cascade). Resolves null if dismissed. */
  requestChoice: (message: string, options: ChoiceOption[]) => Promise<string | null>;
  /** Convenience wrapper over `requestChoice` for the common Cancel/Confirm case. */
  requestConfirm: (message: string, confirmLabel?: string) => Promise<boolean>;
  resolveChoice: (value: string | null) => void;
  setMode: (mode: AppMode) => void;
}

let idCounter = 0;
function nextId(prefix: string) {
  idCounter += 1;
  return `${prefix}_${idCounter}`;
}

const CANCEL = '__cancel__';
const CONFIRM = '__confirm__';

export const useUIStore = create<UIState>((set, get) => ({
  warnings: [],
  choiceRequest: null,
  mode: 'manual',

  pushWarning: (message, level = 'warning') => {
    const warning: Warning = { id: nextId('warn'), message, level };
    set((state) => ({ warnings: [...state.warnings, warning] }));
    // Auto-dismiss after a few seconds so the toast list doesn't pile up.
    setTimeout(() => get().dismissWarning(warning.id), 6000);
  },

  dismissWarning: (id) => {
    set((state) => ({ warnings: state.warnings.filter((w) => w.id !== id) }));
  },

  requestChoice: (message, options) =>
    new Promise<string | null>((resolve) => {
      set({ choiceRequest: { id: nextId('choice'), message, options, resolve } });
    }),

  requestConfirm: async (message, confirmLabel = 'Confirm') => {
    const value = await get().requestChoice(message, [
      { label: 'Cancel', value: CANCEL },
      { label: confirmLabel, value: CONFIRM, danger: true },
    ]);
    return value === CONFIRM;
  },

  resolveChoice: (value) => {
    const request = get().choiceRequest;
    if (!request) return;
    request.resolve(value);
    set({ choiceRequest: null });
  },

  setMode: (mode) => set({ mode }),
}));
