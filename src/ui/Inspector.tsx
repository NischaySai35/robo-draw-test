/**
 * Right-dock properties inspector. Renders one of three views depending on
 * what's selected: whole module, a single rod's joint controls, or a
 * connector's lock state -- these are mutually exclusive in `selectionStore`.
 */
import { useMemo, useState } from 'react';
import { measureLoopError } from '../kinematics/loopClosure';
import { useAssemblyStore } from '../state/assemblyStore';
import { useSelectionStore } from '../state/selectionStore';
import { useUIStore } from '../state/uiStore';
import { computeWeldAnchorPose } from '../kinematics/assemblyGraph';
import type { Connector, ConnectorId, ModuleId } from '../types/module';
import { allConnectors } from '../types/module';
import { deleteModuleWithConfirmation, disconnectConnectorWithConfirmation } from './actions';
import { formatDeg, toDeg, toRad } from './format';

/**
 * Loop/DOF readout for the whole assembly.
 *
 * Renders nothing while the weld graph is a tree, which is the common case --
 * it only earns space once the structure actually has a closed loop to be
 * right or wrong about. Measuring is one forward-kinematics pass
 * (`measureLoopError`), cheap enough for every render; solving is not, so that
 * stays behind the button.
 */
function StructurePanel() {
  const assembly = useAssemblyStore((s) => s.assembly);
  const report = useAssemblyStore((s) => s.loopClosure);
  const closeLoops = useAssemblyStore((s) => s.closeLoops);
  const pushWarning = useUIStore((s) => s.pushWarning);

  const loops = useMemo(() => measureLoopError(assembly), [assembly]);
  if (loops.loopCount === 0) return null;

  // Below this the weld is closed to within a fraction of a millimetre at
  // module scale -- treating it as exact avoids nagging about solver noise.
  const closed = loops.maxPositionError < 1e-3;

  function handleClose() {
    const result = closeLoops();
    if (!result) return;
    pushWarning(
      result.converged
        ? `Loops closed — ${result.loopCount} loop${result.loopCount === 1 ? '' : 's'}, ${result.mobility} DOF remaining.`
        : `Could not close: worst weld is still ${result.maxPositionError.toFixed(3)} apart. These joint limits cannot form this loop.`,
      result.converged ? 'warning' : 'error',
    );
  }

  return (
    <div className="inspector__section">
      <div className="inspector__section-title">Structure</div>
      <div className="inspector__id">
        {loops.loopCount} closed loop{loops.loopCount === 1 ? '' : 's'} ·{' '}
        {report?.converged ? report.mobility : loops.nominalMobility} DOF
        {!report?.converged && ' (nominal)'}
      </div>
      <div className={`status-line${closed ? '' : ' status-line--warn'}`}>
        {closed
          ? 'All welds consistent.'
          : `Worst weld ${loops.maxPositionError.toFixed(3)} apart — the structure does not physically close.`}
      </div>
      {!closed && (
        <button className="btn" onClick={handleClose}>
          Close loops
        </button>
      )}
    </div>
  );
}

export function Inspector() {
  const { selectedModuleIds, selectedRod, selectedConnector } = useSelectionStore();
  const modules = useAssemblyStore((s) => s.assembly.modules);

  // Loop state describes the whole assembly, not the selection, so it sits
  // above whichever panel the current selection calls for.
  const selectedModule = selectedModuleIds.length === 1 ? modules[selectedModuleIds[0]!] : undefined;
  let panel;
  if (selectedConnector) {
    panel = <ConnectorPanel connectorId={selectedConnector} />;
  } else if (selectedRod) {
    panel = <RodPanel moduleId={selectedRod.moduleId} rodIndex={selectedRod.rodIndex} />;
  } else if (selectedModule) {
    panel = <ModulePanel moduleId={selectedModule.id} />;
  } else {
    panel = (
      <div className="inspector">
        <div className="panel__header">Inspector</div>
        <p className="inspector__hint">
          {selectedModuleIds.length > 1
            ? `${selectedModuleIds.length} modules selected.`
            : 'Select a module, rod, or connector to edit it.'}
        </p>
      </div>
    );
  }

  return (
    <>
      <StructurePanel />
      {panel}
    </>
  );
}

function ModulePanel({ moduleId }: { moduleId: ModuleId }) {
  const module = useAssemblyStore((s) => s.assembly.modules[moduleId]);
  const homeModule = useAssemblyStore((s) => s.homeModule);
  const selectRod = useSelectionStore((s) => s.selectRod);
  const selectConnector = useSelectionStore((s) => s.selectConnector);
  if (!module) return null;

  return (
    <div className="inspector">
      <div className="panel__header">Module</div>
      <div className="inspector__section">
        <div className="inspector__id">{module.id}</div>
        <button className="btn" onClick={() => homeModule(moduleId)}>
          Home whole module
        </button>
        <button className="btn btn--danger" onClick={() => void deleteModuleWithConfirmation(moduleId)}>
          Delete module
        </button>
      </div>

      <div className="inspector__section">
        <div className="inspector__section-title">Connectors</div>
        {allConnectors(module).map((connector) => (
          <button key={connector.id} className="row-btn" onClick={() => selectConnector(connector.id)}>
            {connector.locked ? '🔒' : '🔓'} Connector {connector.end}
          </button>
        ))}
      </div>

      <div className="inspector__section">
        <div className="inspector__section-title">Rods</div>
        {module.rods.map((rod, i) => (
          <button key={rod.id} className="row-btn" onClick={() => selectRod(moduleId, i)}>
            <span className={`rod-kind rod-kind--${rod.kind}`}>{rod.kind.toUpperCase()}</span>
            <span>Rod {i + 1}</span>
            <span className="row-btn__value">{formatDeg(rod.angle)}</span>
            <span>{rod.torqueEnabled ? '⚡' : '·'}</span>
          </button>
        ))}
      </div>
    </div>
  );
}

function RodPanel({ moduleId, rodIndex }: { moduleId: ModuleId; rodIndex: number }) {
  const module = useAssemblyStore((s) => s.assembly.modules[moduleId]);
  const setRodAngle = useAssemblyStore((s) => s.setRodAngle);
  const setRodTorque = useAssemblyStore((s) => s.setRodTorque);
  const setRodLimits = useAssemblyStore((s) => s.setRodLimits);
  const setRodHomeAngle = useAssemblyStore((s) => s.setRodHomeAngle);
  const homeRod = useAssemblyStore((s) => s.homeRod);
  const pushWarning = useUIStore((s) => s.pushWarning);
  const rod = module?.rods[rodIndex];
  if (!module || !rod) return null;

  const minDeg = toDeg(rod.min);
  const maxDeg = toDeg(rod.max);

  function commitLimits(nextMinDeg: number, nextMaxDeg: number) {
    if (nextMinDeg > nextMaxDeg) {
      pushWarning('Min limit cannot exceed max limit.', 'error');
      return;
    }
    setRodLimits(moduleId, rodIndex, toRad(nextMinDeg), toRad(nextMaxDeg));
  }

  function commitAngleDeg(nextDeg: number) {
    if (nextDeg < minDeg || nextDeg > maxDeg) {
      pushWarning(`Angle clamped to [${minDeg.toFixed(0)}°, ${maxDeg.toFixed(0)}°] — joint limit.`, 'warning');
    }
    setRodAngle(moduleId, rodIndex, toRad(nextDeg));
  }

  return (
    <div className="inspector">
      <div className="panel__header">
        Rod {rodIndex + 1} <span className={`rod-kind rod-kind--${rod.kind}`}>{rod.kind.toUpperCase()}</span>
      </div>

      <div className="inspector__section">
        <label className="field">
          <span>Angle</span>
          <input
            type="range"
            min={minDeg}
            max={maxDeg}
            step={0.5}
            value={toDeg(rod.angle)}
            onChange={(e) => commitAngleDeg(Number(e.target.value))}
          />
          <input
            type="number"
            className="field__number"
            value={Number(toDeg(rod.angle).toFixed(1))}
            min={minDeg}
            max={maxDeg}
            step={0.5}
            onChange={(e) => commitAngleDeg(Number(e.target.value))}
          />
          <span className="field__unit">°</span>
        </label>

        <div className="field field--pair">
          <label>
            <span>Min °</span>
            <input
              type="number"
              className="field__number"
              value={Number(minDeg.toFixed(1))}
              onChange={(e) => commitLimits(Number(e.target.value), maxDeg)}
            />
          </label>
          <label>
            <span>Max °</span>
            <input
              type="number"
              className="field__number"
              value={Number(maxDeg.toFixed(1))}
              onChange={(e) => commitLimits(minDeg, Number(e.target.value))}
            />
          </label>
        </div>

        <label className="field">
          <span>Home °</span>
          <input
            type="number"
            className="field__number"
            value={Number(toDeg(rod.home).toFixed(1))}
            min={minDeg}
            max={maxDeg}
            onChange={(e) => setRodHomeAngle(moduleId, rodIndex, toRad(Number(e.target.value)))}
          />
          <button className="btn" onClick={() => homeRod(moduleId, rodIndex)}>
            Home
          </button>
        </label>

        <label className="field field--toggle">
          <span>Torque (servo)</span>
          <button
            className={`toggle${rod.torqueEnabled ? ' toggle--on' : ''}`}
            onClick={() => setRodTorque(moduleId, rodIndex, !rod.torqueEnabled)}
          >
            {rod.torqueEnabled ? 'Energized' : 'De-energized'}
          </button>
        </label>
      </div>
    </div>
  );
}

function ConnectorPanel({ connectorId }: { connectorId: ConnectorId }) {
  const assembly = useAssemblyStore((s) => s.assembly);
  const connectConnectors = useAssemblyStore((s) => s.connectConnectors);
  const setModuleBasePose = useAssemblyStore((s) => s.setModuleBasePose);
  const pushWarning = useUIStore((s) => s.pushWarning);
  const [target, setTarget] = useState<ConnectorId>('');

  const owner = Object.values(assembly.modules).find((m) =>
    allConnectors(m).some((c) => c.id === connectorId),
  );
  const connector: Connector | undefined =
    owner && allConnectors(owner).find((c) => c.id === connectorId);
  if (!owner || !connector) return null;

  const openConnectors: Array<{ connector: Connector; moduleId: ModuleId }> = [];
  for (const m of Object.values(assembly.modules)) {
    if (m.id === owner.id) continue;
    for (const c of allConnectors(m)) {
      if (!c.locked) openConnectors.push({ connector: c, moduleId: m.id });
    }
  }

  function handleConnect() {
    if (!target) return;
    const weld = computeWeldAnchorPose(assembly, connectorId, target);
    if (!weld) return;
    setModuleBasePose(weld.anchorId, weld.anchorPose);
    connectConnectors(connectorId, target);
    pushWarning('Connector locked.', 'warning');
  }

  return (
    <div className="inspector">
      <div className="panel__header">Connector {connector.end}</div>
      <div className="inspector__section">
        <div className="inspector__id">{owner.id}</div>
        <div className="inspector__lock-state">{connector.locked ? '🔒 Locked' : '🔓 Unlocked'}</div>

        {connector.locked ? (
          <button className="btn btn--danger" onClick={() => void disconnectConnectorWithConfirmation(connectorId)}>
            Unlock
          </button>
        ) : (
          <>
            <label className="field">
              <span>Connect to</span>
              <select value={target} onChange={(e) => setTarget(e.target.value)}>
                <option value="">Select an open connector…</option>
                {openConnectors.map((c) => (
                  <option key={c.connector.id} value={c.connector.id}>
                    {c.moduleId} · {c.connector.end}
                  </option>
                ))}
              </select>
            </label>
            <button className="btn" disabled={!target} onClick={handleConnect}>
              Lock
            </button>
          </>
        )}
      </div>
    </div>
  );
}
