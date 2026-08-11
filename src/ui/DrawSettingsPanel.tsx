/**
 * Draw-to-build settings. Every toggle here is explicit (no hidden
 * defaults), and options that don't apply given the current selection are
 * grayed out with a tooltip rather than left pickable into an invalid
 * combination -- e.g. "plane" and "extrude" only mean something in 2D mode.
 */
import { useDrawStore } from '../state/drawStore';
import type { AutoAddMode, DrawPlane, StrokeMode } from '../types/draw';

export function DrawSettingsPanel() {
  const settings = useDrawStore((s) => s.settings);
  const updateSettings = useDrawStore((s) => s.updateSettings);
  const assignedModuleCount = useDrawStore((s) => s.assignedModuleCount);
  const setAssignedModuleCount = useDrawStore((s) => s.setAssignedModuleCount);
  const is2D = settings.dimensionality === '2d';

  return (
    <div className="draw-settings">
      <div className="panel__header">Draw settings</div>

      <div className="inspector__section">
        <div className="inspector__section-title">Sketch space</div>
        <div className="radio-row">
          <label className="radio">
            <input
              type="radio"
              checked={is2D}
              onChange={() => updateSettings({ dimensionality: '2d' })}
            />
            2D (on a plane)
          </label>
          <label className="radio">
            <input
              type="radio"
              checked={!is2D}
              onChange={() => updateSettings({ dimensionality: '3d' })}
            />
            3D (free in space)
          </label>
        </div>

        <label className="field" title={is2D ? '' : 'Only applies to 2D sketches'}>
          <span>Plane</span>
          <select
            disabled={!is2D}
            value={settings.plane}
            onChange={(e) => updateSettings({ plane: e.target.value as DrawPlane })}
          >
            <option value="XY">XY</option>
            <option value="XZ">XZ</option>
            <option value="YZ">YZ</option>
          </select>
        </label>

        <label
          className="field field--toggle"
          title={is2D ? 'Scroll after drawing to lift the sketch off the plane' : 'Extrude only applies when lifting a 2D sketch'}
        >
          <span>Extrude to 3D</span>
          <input
            type="checkbox"
            disabled={!is2D}
            checked={settings.extrude}
            onChange={(e) => updateSettings({ extrude: e.target.checked })}
          />
        </label>
      </div>

      <div className="inspector__section">
        <div className="inspector__section-title">Stroke type</div>
        <div className="radio-row">
          <label className="radio">
            <input
              type="radio"
              checked={settings.strokeMode === 'continuous'}
              onChange={() => updateSettings({ strokeMode: 'continuous' as StrokeMode })}
            />
            Continuous line
          </label>
          <label className="radio">
            <input
              type="radio"
              checked={settings.strokeMode === 'complex'}
              onChange={() => updateSettings({ strokeMode: 'complex' as StrokeMode })}
            />
            Complex diagram
          </label>
        </div>
        {settings.strokeMode === 'continuous' && (
          <p className="inspector__hint">Each new stroke replaces the previous one -- it becomes one module chain.</p>
        )}
        {settings.strokeMode === 'complex' && (
          <p className="inspector__hint">Strokes accumulate -- each becomes its own module chain.</p>
        )}
      </div>

      <div className="inspector__section">
        <div className="inspector__section-title">Auto-add modules if short</div>
        <div className="radio-row radio-row--stacked">
          {(['auto', 'ask', 'never'] as AutoAddMode[]).map((mode) => (
            <label key={mode} className="radio">
              <input
                type="radio"
                checked={settings.autoAddMode === mode}
                onChange={() => updateSettings({ autoAddMode: mode })}
              />
              {mode === 'auto' && 'Auto — add without asking'}
              {mode === 'ask' && 'Ask me — partial fit vs. add the rest'}
              {mode === 'never' && "Never — fit only what's assigned"}
            </label>
          ))}
        </div>
        <p className="inspector__hint">
          A shape doesn't have to be fully covered — whatever module count is used fits as much of the stroke as it
          can reach and reports the leftover as residual, never as a hard failure.
        </p>
      </div>

      <div className="inspector__section">
        <div className="inspector__section-title">Fit tolerance</div>
        <label className="field">
          <span>Tolerance</span>
          <input
            type="number"
            className="field__number"
            step={0.05}
            min={0.01}
            value={settings.toleranceWorldUnits}
            onChange={(e) => updateSettings({ toleranceWorldUnits: Math.max(0.01, Number(e.target.value)) })}
          />
          <span className="field__unit">units</span>
        </label>

        <label className="field" title="How many modules are already assigned to realize the stroke (feeds the feasibility check).">
          <span>Assigned</span>
          <input
            type="number"
            className="field__number"
            min={0}
            step={1}
            value={assignedModuleCount}
            onChange={(e) => setAssignedModuleCount(Number(e.target.value))}
          />
          <span className="field__unit">modules</span>
        </label>
      </div>
    </div>
  );
}
