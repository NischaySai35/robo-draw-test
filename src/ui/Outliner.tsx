/** Hierarchy list of every module in the assembly, with lock-state glyphs and right-click delete. */
import { useState } from 'react';
import { useAssemblyStore } from '../state/assemblyStore';
import { useSelectionStore } from '../state/selectionStore';
import { ContextMenu, type ContextMenuItem } from './ContextMenu';
import { deleteModuleWithConfirmation } from './actions';

export function Outliner() {
  const modules = useAssemblyStore((s) => s.assembly.modules);
  const selectedModuleIds = useSelectionStore((s) => s.selectedModuleIds);
  const selectModule = useSelectionStore((s) => s.selectModule);
  const [menu, setMenu] = useState<{ x: number; y: number; moduleId: string } | null>(null);

  const moduleList = Object.values(modules);

  return (
    <div className="outliner">
      <div className="panel__header">Outliner ({moduleList.length})</div>
      <div className="outliner__list">
        {moduleList.length === 0 && <div className="outliner__empty">No modules yet. Use “+ Module”.</div>}
        {moduleList.map((module) => {
          const selected = selectedModuleIds.includes(module.id);
          // Only the two chain-end locks get their own glyph; showing all six
          // would crowd the row, so the four big-rod side locks collapse into a
          // count that appears only when at least one of them is engaged.
          const lockGlyph = `${module.connectorA.locked ? '🔒' : '🔓'}${module.connectorB.locked ? '🔒' : '🔓'}`;
          const lockedSides = module.sides.filter((c) => c.locked).length;
          return (
            <div
              key={module.id}
              className={`outliner__row${selected ? ' outliner__row--selected' : ''}`}
              onClick={(e) => selectModule(module.id, e.shiftKey)}
              onContextMenu={(e) => {
                e.preventDefault();
                setMenu({ x: e.clientX, y: e.clientY, moduleId: module.id });
              }}
            >
              <span className="outliner__name">{module.id}</span>
              <span
                className="outliner__locks"
                title={`Connector A / B lock state${lockedSides > 0 ? ` · ${lockedSides} side lock(s)` : ''}`}
              >
                {lockGlyph}
                {lockedSides > 0 && `·${lockedSides}`}
              </span>
            </div>
          );
        })}
      </div>
      {menu && (
        <ContextMenu
          x={menu.x}
          y={menu.y}
          onClose={() => setMenu(null)}
          items={buildMenuItems(menu.moduleId, selectModule)}
        />
      )}
    </div>
  );
}

function buildMenuItems(moduleId: string, selectModule: (id: string) => void): ContextMenuItem[] {
  return [
    { label: 'Select', onSelect: () => selectModule(moduleId) },
    { label: 'Delete', danger: true, onSelect: () => void deleteModuleWithConfirmation(moduleId) },
  ];
}
