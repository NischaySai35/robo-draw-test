/**
 * Feasibility + fit-quality readout for each drawn stroke, plus the actions
 * that act on them (re-fit, apply, clear, undo/redo). Mirrors Phase 1's
 * inline-warning convention: a bad fit is reported with *why*, never just
 * silently applied.
 */
import { useDrawStore } from '../state/drawStore';
import { useUIStore } from '../state/uiStore';
import { applyFitToAssembly, computeDrawPreview, persistFitForStroke } from './drawActions';

export function DrawStatusPanel() {
  const strokes = useDrawStore((s) => s.strokes);
  const feasibility = useDrawStore((s) => s.feasibility);
  const fitResults = useDrawStore((s) => s.fitResults);
  const clearStrokes = useDrawStore((s) => s.clearStrokes);
  const undo = useDrawStore((s) => s.undo);
  const redo = useDrawStore((s) => s.redo);
  const canUndo = useDrawStore((s) => s.undoStack.length > 0);
  const canRedo = useDrawStore((s) => s.redoStack.length > 0);
  const requestConfirm = useUIStore((s) => s.requestConfirm);

  async function handleClear() {
    if (strokes.length === 0) return;
    const confirmed = await requestConfirm('Clear all drawn strokes? This can be undone.', 'Clear');
    if (confirmed) clearStrokes();
  }

  return (
    <div className="draw-status">
      <div className="panel__header">Fit status</div>

      <div className="inspector__section draw-status__toolbar">
        <button className="btn" onClick={undo} disabled={!canUndo}>
          ↶ Undo
        </button>
        <button className="btn" onClick={redo} disabled={!canRedo}>
          ↷ Redo
        </button>
        <button className="btn btn--danger" onClick={() => void handleClear()} disabled={strokes.length === 0}>
          Clear
        </button>
      </div>

      {strokes.length === 0 && (
        <p className="inspector__hint">Left-drag in the viewport to draw a stroke. Right-drag orbits the camera.</p>
      )}

      {strokes.map((stroke) => {
        const report = feasibility[stroke.id];
        const fit = fitResults[stroke.id];
        return (
          <div key={stroke.id} className="inspector__section draw-status__stroke">
            <div className="inspector__id">
              {stroke.id} · {stroke.points.length} pts
            </div>

            {report && (
              <div className={`status-line status-line--${report.status === 'fits' ? 'ok' : 'warn'}`}>
                {report.status === 'fits'
                  ? `Fits with current modules (${report.modulesAvailable} assigned).`
                  : `Needs ${report.modulesNeeded} modules — ${report.modulesAvailable} assigned, ${report.deficit} short.`}
              </div>
            )}

            {fit && (
              <div className={`status-line status-line--${fit.withinTolerance ? 'ok' : 'warn'}`}>
                Residual {fit.residual.toFixed(3)} (max {fit.maxDeviation.toFixed(3)}) —{' '}
                {fit.withinTolerance ? 'within tolerance' : 'outside tolerance'}
              </div>
            )}

            {fit && fit.diagnostics.length > 0 && (
              <ul className="draw-status__diagnostics">
                {fit.diagnostics.slice(0, 3).map((d, i) => (
                  <li key={i}>{d.message}</li>
                ))}
              </ul>
            )}

            <div className="draw-status__actions">
              <button className="btn" onClick={() => persistFitForStroke(stroke)}>
                Re-fit
              </button>
              <button
                className="btn"
                onClick={() => {
                  const preview = computeDrawPreview(stroke);
                  if (preview) void applyFitToAssembly(stroke, preview);
                }}
                disabled={stroke.points.length < 2}
              >
                Apply
              </button>
            </div>
          </div>
        );
      })}
    </div>
  );
}
