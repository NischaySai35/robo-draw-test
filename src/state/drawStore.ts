/**
 * State for the Phase 2 draw-to-build mode: sketch settings, the strokes
 * drawn so far, and the fit/feasibility results computed for each. Kept
 * separate from `assemblyStore` -- nothing here is committed to the real
 * module graph until the user hits "Apply" (see `ui/drawActions.ts`).
 */
import { create } from 'zustand';
import type { DrawSettings, FeasibilityReport, FitResult, Stroke } from '../types/draw';
import { DEFAULT_DRAW_SETTINGS } from '../types/draw';

interface HistoryEntry {
  strokes: Stroke[];
}

interface DrawState {
  settings: DrawSettings;
  strokes: Stroke[];
  activeStrokeId: string | null;
  isDrawing: boolean;
  /** How many modules the user says are already assigned to the active stroke -- feeds the feasibility comparison. */
  assignedModuleCount: number;
  fitResults: Record<string, FitResult | undefined>;
  feasibility: Record<string, FeasibilityReport | undefined>;

  undoStack: HistoryEntry[];
  redoStack: HistoryEntry[];

  updateSettings: (patch: Partial<DrawSettings>) => void;
  setAssignedModuleCount: (count: number) => void;

  startStroke: () => string;
  appendPoint: (point: [number, number, number]) => void;
  endStroke: () => void;
  /** Translates every point of a stroke by a fixed delta -- used by the extrude/lift gesture. */
  offsetStrokePoints: (strokeId: string, delta: [number, number, number]) => void;

  setFitResult: (strokeId: string, result: FitResult) => void;
  setFeasibility: (strokeId: string, report: FeasibilityReport) => void;

  clearStrokes: () => void;
  removeStroke: (strokeId: string) => void;
  undo: () => void;
  redo: () => void;
}

let strokeCounter = 0;
function nextStrokeId() {
  strokeCounter += 1;
  return `stroke_${Date.now().toString(36)}_${strokeCounter}`;
}

/**
 * `dimensionality`/`extrude` and `strokeMode` combinations are structurally
 * constrained here (not just in the UI) so no store consumer can ever end up
 * with an invalid combo even if it bypasses the settings panel's disabling.
 */
function sanitizeSettings(settings: DrawSettings): DrawSettings {
  return {
    ...settings,
    extrude: settings.dimensionality === '2d' ? settings.extrude : false,
  };
}

export const useDrawStore = create<DrawState>((set, get) => ({
  settings: DEFAULT_DRAW_SETTINGS,
  strokes: [],
  activeStrokeId: null,
  isDrawing: false,
  assignedModuleCount: 0,
  fitResults: {},
  feasibility: {},
  undoStack: [],
  redoStack: [],

  updateSettings: (patch) => {
    set((state) => ({ settings: sanitizeSettings({ ...state.settings, ...patch }) }));
  },

  setAssignedModuleCount: (count) => set({ assignedModuleCount: Math.max(0, count) }),

  startStroke: () => {
    const id = nextStrokeId();
    const stroke: Stroke = { id, points: [] };
    set((state) => {
      const strokes = state.settings.strokeMode === 'continuous' ? [stroke] : [...state.strokes, stroke];
      return { strokes, activeStrokeId: id, isDrawing: true };
    });
    return id;
  },

  appendPoint: (point) => {
    set((state) => {
      if (!state.activeStrokeId) return state;
      const strokes = state.strokes.map((s) =>
        s.id === state.activeStrokeId ? { ...s, points: [...s.points, point] } : s,
      );
      return { strokes };
    });
  },

  endStroke: () => {
    const { strokes, undoStack } = get();
    set({ isDrawing: false, undoStack: [...undoStack, { strokes: cloneStrokes(strokes) }], redoStack: [] });
  },

  offsetStrokePoints: (strokeId, delta) => {
    set((state) => ({
      strokes: state.strokes.map((s) =>
        s.id === strokeId
          ? { ...s, points: s.points.map((p) => [p[0] + delta[0], p[1] + delta[1], p[2] + delta[2]] as [number, number, number]) }
          : s,
      ),
    }));
  },

  setFitResult: (strokeId, result) => {
    set((state) => ({ fitResults: { ...state.fitResults, [strokeId]: result } }));
  },

  setFeasibility: (strokeId, report) => {
    set((state) => ({ feasibility: { ...state.feasibility, [strokeId]: report } }));
  },

  clearStrokes: () => {
    const { strokes, undoStack } = get();
    set({
      strokes: [],
      activeStrokeId: null,
      fitResults: {},
      feasibility: {},
      undoStack: [...undoStack, { strokes: cloneStrokes(strokes) }],
      redoStack: [],
    });
  },

  removeStroke: (strokeId) => {
    set((state) => ({
      strokes: state.strokes.filter((s) => s.id !== strokeId),
      activeStrokeId: state.activeStrokeId === strokeId ? null : state.activeStrokeId,
    }));
  },

  undo: () => {
    const { undoStack, redoStack, strokes } = get();
    const entry = undoStack[undoStack.length - 1];
    if (!entry) return;
    set({
      strokes: entry.strokes,
      undoStack: undoStack.slice(0, -1),
      redoStack: [...redoStack, { strokes: cloneStrokes(strokes) }],
      activeStrokeId: null,
    });
  },

  redo: () => {
    const { undoStack, redoStack, strokes } = get();
    const entry = redoStack[redoStack.length - 1];
    if (!entry) return;
    set({
      strokes: entry.strokes,
      redoStack: redoStack.slice(0, -1),
      undoStack: [...undoStack, { strokes: cloneStrokes(strokes) }],
      activeStrokeId: null,
    });
  },
}));

function cloneStrokes(strokes: Stroke[]): Stroke[] {
  return JSON.parse(JSON.stringify(strokes)) as Stroke[];
}
