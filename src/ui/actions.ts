/**
 * Shared, confirmation-gated action helpers used by the toolbar, outliner,
 * and viewport context menu alike -- keeps the "destructive action needs a
 * confirmation" rule enforced in exactly one place.
 */
import type { ConnectorId, ModuleId } from '../types/module';
import { useAssemblyStore } from '../state/assemblyStore';
import { useSelectionStore } from '../state/selectionStore';
import { useUIStore } from '../state/uiStore';

export async function deleteModuleWithConfirmation(moduleId: ModuleId): Promise<void> {
  const { moduleHasActiveLocks, deleteModule } = useAssemblyStore.getState();
  const { requestConfirm, pushWarning } = useUIStore.getState();

  if (moduleHasActiveLocks(moduleId)) {
    const confirmed = await requestConfirm(
      'This module is locked to another module. Deleting it will break that connection. Delete anyway?',
      'Delete',
    );
    if (!confirmed) return;
  }
  deleteModule(moduleId);
  useSelectionStore.getState().clearSelection();
  pushWarning('Module deleted.', 'warning');
}

export async function disconnectConnectorWithConfirmation(connectorId: ConnectorId): Promise<void> {
  const { disconnectConnector } = useAssemblyStore.getState();
  const { requestConfirm, pushWarning } = useUIStore.getState();

  const confirmed = await requestConfirm('Unlock this connector? The two modules will separate.', 'Unlock');
  if (!confirmed) return;
  disconnectConnector(connectorId);
  pushWarning('Connector unlocked.', 'warning');
}
