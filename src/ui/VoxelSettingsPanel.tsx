/** Left dock for cube-builder mode: brief instructions + structural controls + undo/redo. */
import { useUIStore } from '../state/uiStore';
import { useVoxelStore } from '../state/voxelStore';

export function VoxelSettingsPanel() {
  const graph = useVoxelStore((s) => s.graph);
  const clearAll = useVoxelStore((s) => s.clearAll);
  const undo = useVoxelStore((s) => s.undo);
  const redo = useVoxelStore((s) => s.redo);
  const canUndo = useVoxelStore((s) => s.undoStack.length > 0);
  const canRedo = useVoxelStore((s) => s.redoStack.length > 0);
  const requestConfirm = useUIStore((s) => s.requestConfirm);

  const cubeCount = Object.keys(graph.voxels).length;

  async function handleClear() {
    const confirmed = await requestConfirm('Clear all cubes except the seed? This can be undone.', 'Clear');
    if (confirmed) clearAll();
  }

  return (
    <div className="draw-settings">
      <div className="panel__header">Cube builder</div>

      <div className="inspector__section">
        <p className="inspector__hint">
          Click a face's ghost hotspot to spawn an adjacent cube — drag further to spawn a row of them. Click a cube
          body to select it. Right-drag orbits.
        </p>
        <div className="inspector__id">{cubeCount} cube{cubeCount === 1 ? '' : 's'}</div>
      </div>

      <div className="inspector__section draw-status__toolbar">
        <button className="btn" onClick={undo} disabled={!canUndo}>
          ↶ Undo
        </button>
        <button className="btn" onClick={redo} disabled={!canRedo}>
          ↷ Redo
        </button>
        <button className="btn btn--danger" onClick={() => void handleClear()}>
          Clear
        </button>
      </div>
    </div>
  );
}
