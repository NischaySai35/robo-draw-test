/**
 * The assembly store: the single source of truth for the module graph.
 *
 * Undo/redo covers structural edits (add/delete/connect/disconnect) per the
 * Phase 1 requirement -- each structural action snapshots the assembly
 * beforehand. Joint angle/torque/limit/home edits do NOT snapshot (so
 * dragging a slider doesn't flood the undo stack); that's called out as a
 * nice-to-have in the spec and left for a later pass.
 */
import { create } from 'zustand';
import type { Assembly, ConnectorId, ModuleId, Pose } from '../types/module';
import { allConnectors } from '../types/module';
import { clampRodAngle, createModule } from '../kinematics/factory';
import { connectedComponents, findConnector } from '../kinematics/assemblyGraph';
import { solveLoopClosure, type LoopClosureReport } from '../kinematics/loopClosure';

function cloneAssembly(assembly: Assembly): Assembly {
  return JSON.parse(JSON.stringify(assembly)) as Assembly;
}

interface HistoryEntry {
  assembly: Assembly;
}

interface AssemblyState {
  assembly: Assembly;
  undoStack: HistoryEntry[];
  redoStack: HistoryEntry[];
  /**
   * Result of the last loop-closure solve, or null when nothing has needed
   * solving since the topology last changed. A report with `converged: false`
   * means the structure is currently claiming a loop it cannot physically
   * form -- see `kinematics/loopClosure.ts`.
   */
  loopClosure: LoopClosureReport | null;

  addModule: (pose?: Pose) => ModuleId;
  deleteModule: (moduleId: ModuleId) => void;
  connectConnectors: (a: ConnectorId, b: ConnectorId) => void;
  disconnectConnector: (connectorId: ConnectorId) => void;
  setModuleBasePose: (moduleId: ModuleId, pose: Pose) => void;
  /**
   * Merges a fully-formed assembly (fresh modules, welds already applied) into
   * the scene -- how a synthesized shape gets built. Module ids are generated
   * unique, so nothing collides with what is already there.
   */
  importAssembly: (incoming: Assembly) => void;
  /** Re-solves every kinematic loop and applies the result. No-op on a tree. */
  closeLoops: () => LoopClosureReport | null;

  setRodAngle: (moduleId: ModuleId, rodIndex: number, angle: number) => void;
  /**
   * Writes whole joint vectors at once, as returned by the kinematics solvers.
   * One store update instead of six per module, which matters when this runs
   * per pointer-move during an IK drag. Like the other joint edits it does not
   * snapshot, so a drag doesn't flood the undo stack.
   */
  setSolvedAngles: (angles: Map<ModuleId, number[]>) => void;
  setRodTorque: (moduleId: ModuleId, rodIndex: number, enabled: boolean) => void;
  setRodLimits: (moduleId: ModuleId, rodIndex: number, min: number, max: number) => void;
  setRodHomeAngle: (moduleId: ModuleId, rodIndex: number, home: number) => void;
  homeRod: (moduleId: ModuleId, rodIndex: number) => void;
  homeModule: (moduleId: ModuleId) => void;

  moduleHasActiveLocks: (moduleId: ModuleId) => boolean;

  undo: () => void;
  redo: () => void;
  canUndo: () => boolean;
  canRedo: () => boolean;
}

const MAX_HISTORY = 100;

export const useAssemblyStore = create<AssemblyState>((set, get) => {
  /** Push the current assembly onto the undo stack and clear redo -- call before a structural mutation. */
  function snapshot() {
    const { assembly, undoStack } = get();
    const next = [...undoStack, { assembly: cloneAssembly(assembly) }];
    if (next.length > MAX_HISTORY) next.shift();
    set({ undoStack: next, redoStack: [] });
  }

  function mutate(recipe: (assembly: Assembly) => Assembly) {
    set((state) => ({ assembly: recipe(cloneAssembly(state.assembly)) }));
  }

  return {
    assembly: { modules: {}, edges: [] },
    undoStack: [],
    redoStack: [],
    loopClosure: null,

    addModule: (pose) => {
      snapshot();
      const module = createModule(pose);
      mutate((assembly) => {
        assembly.modules[module.id] = module;
        return assembly;
      });
      return module.id;
    },

    deleteModule: (moduleId) => {
      snapshot();
      mutate((assembly) => {
        const module = assembly.modules[moduleId];
        if (!module) return assembly;
        for (const connector of allConnectors(module)) {
          if (connector.connectedTo) {
            const partner = findConnector(assembly, connector.connectedTo);
            if (partner) {
              partner.locked = false;
              partner.connectedTo = null;
            }
            assembly.edges = assembly.edges.filter(
              (e) => e.a !== connector.id && e.b !== connector.id,
            );
          }
        }
        delete assembly.modules[moduleId];
        return assembly;
      });
    },

    connectConnectors: (a, b) => {
      snapshot();
      let report: LoopClosureReport | null = null;
      mutate((assembly) => {
        const connA = findConnector(assembly, a);
        const connB = findConnector(assembly, b);
        if (!connA || !connB || connA.locked || connB.locked) return assembly;

        // Welding two modules that are already in the same component closes a
        // kinematic loop; welding across components just grows the tree. Only
        // the former needs solving, which keeps bulk operations (voxel
        // conversion welds a chain module-by-module) off the expensive path.
        const createsLoop = connectedComponents(assembly).some(
          (component) => component.includes(connA.moduleId) && component.includes(connB.moduleId),
        );

        connA.locked = true;
        connB.locked = true;
        connA.connectedTo = connB.id;
        connB.connectedTo = connA.id;
        assembly.edges.push({ a, b });

        if (createsLoop) {
          const solved = solveLoopClosure(assembly);
          report = solved.report;
          // A failed solve still leaves the weld in place: the user asked for
          // it, and refusing mid-construction is worse than showing an honest
          // "this loop doesn't close" in the inspector.
          if (solved.report.converged) {
            for (const [moduleId, angles] of solved.angles) {
              assembly.modules[moduleId]!.rods.forEach((rod, i) => {
                rod.angle = angles[i]!;
              });
            }
          }
        }
        return assembly;
      });
      set({ loopClosure: report });
    },

    importAssembly: (incoming) => {
      snapshot();
      mutate((assembly) => {
        for (const module of Object.values(incoming.modules)) {
          assembly.modules[module.id] = module;
        }
        assembly.edges.push(...incoming.edges);
        return assembly;
      });
      set({ loopClosure: null });
    },

    closeLoops: () => {
      const solved = solveLoopClosure(get().assembly);
      if (solved.report.loopCount === 0) {
        set({ loopClosure: null });
        return null;
      }
      if (solved.report.converged) {
        snapshot();
        mutate((assembly) => {
          for (const [moduleId, angles] of solved.angles) {
            assembly.modules[moduleId]?.rods.forEach((rod, i) => {
              rod.angle = angles[i]!;
            });
          }
          return assembly;
        });
      }
      set({ loopClosure: solved.report });
      return solved.report;
    },

    disconnectConnector: (connectorId) => {
      snapshot();
      mutate((assembly) => {
        const connector = findConnector(assembly, connectorId);
        if (!connector || !connector.connectedTo) return assembly;
        const partner = findConnector(assembly, connector.connectedTo);
        if (partner) {
          partner.locked = false;
          partner.connectedTo = null;
        }
        connector.locked = false;
        connector.connectedTo = null;
        assembly.edges = assembly.edges.filter(
          (e) => e.a !== connectorId && e.b !== connectorId,
        );
        return assembly;
      });
      // Topology changed, so the old report describes a structure that no
      // longer exists. Breaking a weld can only remove loops, never add one,
      // so there is nothing to re-solve.
      set({ loopClosure: null });
    },

    setModuleBasePose: (moduleId, pose) => {
      mutate((assembly) => {
        const module = assembly.modules[moduleId];
        if (module) module.basePose = pose;
        return assembly;
      });
    },

    setRodAngle: (moduleId, rodIndex, angle) => {
      mutate((assembly) => {
        const rod = assembly.modules[moduleId]?.rods[rodIndex];
        if (rod) rod.angle = clampRodAngle(rod, angle);
        return assembly;
      });
    },

    setSolvedAngles: (angles) => {
      mutate((assembly) => {
        for (const [moduleId, rodAngles] of angles) {
          assembly.modules[moduleId]?.rods.forEach((rod, i) => {
            rod.angle = clampRodAngle(rod, rodAngles[i]!);
          });
        }
        return assembly;
      });
    },

    setRodTorque: (moduleId, rodIndex, enabled) => {
      mutate((assembly) => {
        const rod = assembly.modules[moduleId]?.rods[rodIndex];
        if (rod) rod.torqueEnabled = enabled;
        return assembly;
      });
    },

    setRodLimits: (moduleId, rodIndex, min, max) => {
      mutate((assembly) => {
        const rod = assembly.modules[moduleId]?.rods[rodIndex];
        if (rod) {
          rod.min = min;
          rod.max = max;
          rod.angle = clampRodAngle(rod, rod.angle);
          rod.home = clampRodAngle(rod, rod.home);
        }
        return assembly;
      });
    },

    setRodHomeAngle: (moduleId, rodIndex, home) => {
      mutate((assembly) => {
        const rod = assembly.modules[moduleId]?.rods[rodIndex];
        if (rod) rod.home = clampRodAngle(rod, home);
        return assembly;
      });
    },

    homeRod: (moduleId, rodIndex) => {
      mutate((assembly) => {
        const rod = assembly.modules[moduleId]?.rods[rodIndex];
        if (rod) rod.angle = rod.home;
        return assembly;
      });
    },

    homeModule: (moduleId) => {
      mutate((assembly) => {
        const module = assembly.modules[moduleId];
        if (module) module.rods.forEach((rod) => { rod.angle = rod.home; });
        return assembly;
      });
    },

    moduleHasActiveLocks: (moduleId) => {
      const module = get().assembly.modules[moduleId];
      if (!module) return false;
      return allConnectors(module).some((c) => c.locked);
    },

    undo: () => {
      const { undoStack, redoStack, assembly } = get();
      const entry = undoStack[undoStack.length - 1];
      if (!entry) return;
      set({
        assembly: entry.assembly,
        undoStack: undoStack.slice(0, -1),
        redoStack: [...redoStack, { assembly: cloneAssembly(assembly) }],
      });
    },

    redo: () => {
      const { undoStack, redoStack, assembly } = get();
      const entry = redoStack[redoStack.length - 1];
      if (!entry) return;
      set({
        assembly: entry.assembly,
        redoStack: redoStack.slice(0, -1),
        undoStack: [...undoStack, { assembly: cloneAssembly(assembly) }],
      });
    },

    canUndo: () => get().undoStack.length > 0,
    canRedo: () => get().redoStack.length > 0,
  };
});
