/** Top-level mode switcher: Phase 1's manual editor vs Phase 2's draw-to-build window. */
import { useUIStore } from '../state/uiStore';

export function ModeTabs() {
  const mode = useUIStore((s) => s.mode);
  const setMode = useUIStore((s) => s.setMode);

  return (
    <div className="mode-tabs">
      <button className={`mode-tab${mode === 'manual' ? ' mode-tab--active' : ''}`} onClick={() => setMode('manual')}>
        Manual Edit
      </button>
      <button className={`mode-tab${mode === 'draw' ? ' mode-tab--active' : ''}`} onClick={() => setMode('draw')}>
        Draw to Build
      </button>
      <button className={`mode-tab${mode === 'cube' ? ' mode-tab--active' : ''}`} onClick={() => setMode('cube')}>
        Cube Builder
      </button>
    </div>
  );
}
