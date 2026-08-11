import { useEffect, useState } from 'react';
import { ModelEditor } from './scene/ModelEditor';
import { DrawModeEditor } from './scene/DrawModeEditor';
import { VoxelModeEditor } from './scene/VoxelModeEditor';
import { Toolbar } from './ui/Toolbar';
import { Outliner } from './ui/Outliner';
import { Inspector } from './ui/Inspector';
import { ModeTabs } from './ui/ModeTabs';
import { DrawSettingsPanel } from './ui/DrawSettingsPanel';
import { DrawStatusPanel } from './ui/DrawStatusPanel';
import { VoxelSettingsPanel } from './ui/VoxelSettingsPanel';
import { VoxelInspector } from './ui/VoxelInspector';
import { ToastList } from './ui/ToastList';
import { ConfirmDialogHost } from './ui/ConfirmDialogHost';
import { ContextMenu, type ContextMenuItem } from './ui/ContextMenu';
import { useAssemblyStore } from './state/assemblyStore';
import { useSelectionStore } from './state/selectionStore';
import { useVoxelStore } from './state/voxelStore';
import { useUIStore } from './state/uiStore';
import { deleteModuleWithConfirmation } from './ui/actions';
import { deleteVoxelWithConfirmation } from './ui/voxelActions';

export default function App() {
  const mode = useUIStore((s) => s.mode);
  const undo = useAssemblyStore((s) => s.undo);
  const redo = useAssemblyStore((s) => s.redo);
  const [viewportMenu, setViewportMenu] = useState<{ x: number; y: number } | null>(null);

  useEffect(() => {
    function isEditingText(target: EventTarget | null) {
      return target instanceof HTMLElement && ['INPUT', 'SELECT', 'TEXTAREA'].includes(target.tagName);
    }

    function handleKeyDown(event: KeyboardEvent) {
      const currentMode = useUIStore.getState().mode;
      if (isEditingText(event.target)) return;

      if (currentMode === 'cube') {
        if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 'z') {
          event.preventDefault();
          if (event.shiftKey) useVoxelStore.getState().redo();
          else useVoxelStore.getState().undo();
          return;
        }
        if (event.key === 'Delete' || event.key === 'Backspace') {
          const { selectedVoxelId } = useVoxelStore.getState();
          if (selectedVoxelId) void deleteVoxelWithConfirmation(selectedVoxelId);
        }
        return;
      }

      // Draw mode has its own input scheme (see DrawModeController) -- nothing to bind here.
      if (currentMode !== 'manual') return;

      if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 'z') {
        event.preventDefault();
        if (event.shiftKey) redo();
        else undo();
        return;
      }
      if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 'y') {
        event.preventDefault();
        redo();
        return;
      }
      if (event.key === 'Delete' || event.key === 'Backspace') {
        const { selectedModuleIds } = useSelectionStore.getState();
        selectedModuleIds.forEach((id) => void deleteModuleWithConfirmation(id));
      }
      if (event.key.toLowerCase() === 'a' && !event.ctrlKey && !event.metaKey) {
        const count = Object.keys(useAssemblyStore.getState().assembly.modules).length;
        const id = useAssemblyStore
          .getState()
          .addModule({ position: [count * 2, 0.5, 0], quaternion: [0, 0, 0, 1] });
        useSelectionStore.getState().selectModule(id);
      }
    }

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [undo, redo]);

  return (
    <div className="app">
      <div className="topbar">
        <ModeTabs />
        {mode === 'manual' && <Toolbar />}
      </div>
      <div className="app__body">
        <aside className="dock dock--left">
          {mode === 'manual' && <Outliner />}
          {mode === 'draw' && <DrawSettingsPanel />}
          {mode === 'cube' && <VoxelSettingsPanel />}
        </aside>
        <main
          className="app__viewport"
          onContextMenu={(e) => {
            if (mode !== 'manual') return;
            e.preventDefault();
            setViewportMenu({ x: e.clientX, y: e.clientY });
          }}
        >
          {mode === 'manual' && <ModelEditor />}
          {mode === 'draw' && <DrawModeEditor />}
          {mode === 'cube' && <VoxelModeEditor />}
          {mode === 'manual' && viewportMenu && (
            <ContextMenu
              x={viewportMenu.x}
              y={viewportMenu.y}
              onClose={() => setViewportMenu(null)}
              items={buildViewportMenuItems()}
            />
          )}
        </main>
        <aside className="dock dock--right">
          {mode === 'manual' && <Inspector />}
          {mode === 'draw' && <DrawStatusPanel />}
          {mode === 'cube' && <VoxelInspector />}
        </aside>
      </div>
      <ToastList />
      <ConfirmDialogHost />
    </div>
  );
}

function buildViewportMenuItems(): ContextMenuItem[] {
  const selection = useSelectionStore.getState();
  return [
    {
      label: 'Add module',
      onSelect: () => {
        const count = Object.keys(useAssemblyStore.getState().assembly.modules).length;
        const id = useAssemblyStore
          .getState()
          .addModule({ position: [count * 2, 0.5, 0], quaternion: [0, 0, 0, 1] });
        selection.selectModule(id);
      },
    },
    {
      label: 'Delete selected',
      danger: true,
      disabled: selection.selectedModuleIds.length === 0,
      onSelect: () => selection.selectedModuleIds.forEach((id) => void deleteModuleWithConfirmation(id)),
    },
  ];
}
