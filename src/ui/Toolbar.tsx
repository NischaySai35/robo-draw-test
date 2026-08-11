/** Top toolbar: add/delete modules, build a target shape, and undo/redo -- mirrored by keyboard shortcuts in App. */
import { useState } from 'react';
import { useAssemblyStore } from '../state/assemblyStore';
import { useSelectionStore } from '../state/selectionStore';
import { useUIStore } from '../state/uiStore';
import { fitSkeleton } from '../kinematics/skeletonFit';
import { SHAPE_LIBRARY } from '../kinematics/shapeLibrary';
import { deleteModuleWithConfirmation } from './actions';

export function Toolbar() {
  const addModule = useAssemblyStore((s) => s.addModule);
  const importAssembly = useAssemblyStore((s) => s.importAssembly);
  const pushWarning = useUIStore((s) => s.pushWarning);
  const [shapeName, setShapeName] = useState(SHAPE_LIBRARY[0]!.name);
  const [building, setBuilding] = useState(false);
  const undo = useAssemblyStore((s) => s.undo);
  const redo = useAssemblyStore((s) => s.redo);
  const canUndo = useAssemblyStore((s) => s.undoStack.length > 0);
  const canRedo = useAssemblyStore((s) => s.redoStack.length > 0);
  const selectedModuleIds = useSelectionStore((s) => s.selectedModuleIds);
  const selectModule = useSelectionStore((s) => s.selectModule);

  function handleAdd() {
    // Spread new modules out along X so they don't spawn stacked on top of each other.
    const count = Object.keys(useAssemblyStore.getState().assembly.modules).length;
    const id = addModule({ position: [count * 2, 0.5, 0], quaternion: [0, 0, 0, 1] });
    selectModule(id);
  }

  function handleDelete() {
    selectedModuleIds.forEach((id) => void deleteModuleWithConfirmation(id));
  }

  /**
   * Synthesizes the chosen target shape into real modules. Solving takes on the
   * order of a second for something the size of a chair, so the button reports
   * that it is working rather than appearing to hang.
   */
  function handleBuildShape() {
    const spec = SHAPE_LIBRARY.find((s) => s.name === shapeName);
    if (!spec) return;
    setBuilding(true);
    // Yield once so the disabled/"Building…" state paints before the solver
    // takes the main thread.
    setTimeout(() => {
      try {
        const result = fitSkeleton(spec);
        importAssembly(result.assembly);
        const loops = result.loopReport;
        const loopNote = loops
          ? loops.converged
            ? `, ${loops.loopCount} loop${loops.loopCount === 1 ? '' : 's'} closed`
            : `, but ${loops.loopCount} loop${loops.loopCount === 1 ? '' : 's'} did NOT close (worst weld ${loops.maxPositionError.toFixed(2)} apart)`
          : '';
        const unanchored = result.unanchored.length > 0
          ? ` ${result.unanchored.length} junction(s) left unwelded.`
          : '';
        pushWarning(
          `Built ${spec.name}: ${result.moduleCount} modules${loopNote}.${unanchored}`,
          loops && !loops.converged ? 'error' : 'warning',
        );
      } finally {
        setBuilding(false);
      }
    }, 0);
  }

  return (
    <div className="toolbar">
      <span className="toolbar__brand">MODULINK</span>
      <button className="btn" onClick={handleAdd} title="Add module (A)">
        + Module
      </button>
      <button className="btn" onClick={handleDelete} disabled={selectedModuleIds.length === 0} title="Delete selected (Del)">
        Delete
      </button>
      <div className="toolbar__divider" />
      <select
        value={shapeName}
        onChange={(e) => setShapeName(e.target.value)}
        title="Target shape to synthesize"
      >
        {SHAPE_LIBRARY.map((spec) => (
          <option key={spec.name} value={spec.name}>{spec.name}</option>
        ))}
      </select>
      <button className="btn" onClick={handleBuildShape} disabled={building} title="Fit modules to the target shape">
        {building ? 'Building…' : 'Build shape'}
      </button>
      <div className="toolbar__divider" />
      <button className="btn" onClick={undo} disabled={!canUndo} title="Undo (Ctrl+Z)">
        ↶ Undo
      </button>
      <button className="btn" onClick={redo} disabled={!canRedo} title="Redo (Ctrl+Y)">
        ↷ Redo
      </button>
    </div>
  );
}
