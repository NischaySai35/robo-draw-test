/**
 * Right dock for cube-builder mode: the selected cube's move/delete
 * controls, plus a always-visible "convert to modules" summary -- feasibility
 * and branch-point warnings surfaced the same way Phase 2 reports fit issues.
 */
import { useMemo } from 'react';
import { useVoxelStore } from '../state/voxelStore';
import { FACE_DIRECTIONS, type FaceDirection } from '../types/voxel';
import { neighborCoord } from '../kinematics/voxelGraph';
import { applyVoxelConversion, computeVoxelConversionPreview, deleteVoxelWithConfirmation } from './voxelActions';
import { useUIStore } from '../state/uiStore';

const FACE_LABELS: Record<FaceDirection, string> = {
  '+x': '+X', '-x': '-X', '+y': '+Y', '-y': '-Y', '+z': '+Z', '-z': '-Z',
};

export function VoxelInspector() {
  const graph = useVoxelStore((s) => s.graph);
  const selectedVoxelId = useVoxelStore((s) => s.selectedVoxelId);
  const moveVoxel = useVoxelStore((s) => s.moveVoxel);
  const pushWarning = useUIStore((s) => s.pushWarning);

  const selected = selectedVoxelId ? graph.voxels[selectedVoxelId] : null;

  const preview = useMemo(() => computeVoxelConversionPreview(graph), [graph]);

  function handleMove(direction: FaceDirection) {
    if (!selected) return;
    const target = neighborCoord(selected.coord, direction);
    const ok = moveVoxel(selected.id, target);
    if (!ok) {
      pushWarning('Can\'t move there — cell is occupied, or the move would leave this cube disconnected.', 'error');
    }
  }

  return (
    <div className="inspector">
      <div className="panel__header">Cube inspector</div>

      {selected ? (
        <div className="inspector__section">
          <div className="inspector__id">({selected.coord.join(', ')})</div>
          <div className="inspector__section-title">Move</div>
          <div className="voxel-move-grid">
            {FACE_DIRECTIONS.map((dir) => (
              <button key={dir} className="btn" onClick={() => handleMove(dir)}>
                {FACE_LABELS[dir]}
              </button>
            ))}
          </div>
          <button className="btn btn--danger" onClick={() => void deleteVoxelWithConfirmation(selected.id)}>
            Delete cube
          </button>
        </div>
      ) : (
        <p className="inspector__hint">Select a cube to move or delete it.</p>
      )}

      <div className="inspector__section">
        <div className="inspector__section-title">Convert to modules</div>
        <div className="inspector__id">
          {preview.paths.length} chain{preview.paths.length === 1 ? '' : 's'} ·{' '}
          {preview.paths.reduce((sum, p) => sum + p.moduleCount, 0)} modules total
          {preview.branchWeldCount > 0 && (
            <>
              {' · '}
              {preview.branchWeldCount} branch weld{preview.branchWeldCount === 1 ? '' : 's'}
            </>
          )}
        </div>
        {preview.branchPointWarnings.map((msg, i) => (
          <div key={i} className="status-line status-line--warn">
            {msg}
          </div>
        ))}
        <button className="btn" disabled={preview.paths.length === 0} onClick={() => void applyVoxelConversion(preview)}>
          Apply to assembly
        </button>
      </div>
    </div>
  );
}
