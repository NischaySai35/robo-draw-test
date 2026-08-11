import { beforeEach, describe, expect, it } from 'vitest';
import { useAssemblyStore } from '../state/assemblyStore';

function resetStore() {
  useAssemblyStore.setState({
    assembly: { modules: {}, edges: [] },
    undoStack: [],
    redoStack: [],
  });
}

describe('assemblyStore structural edits', () => {
  beforeEach(resetStore);

  it('adds and deletes modules', () => {
    const id = useAssemblyStore.getState().addModule();
    expect(Object.keys(useAssemblyStore.getState().assembly.modules)).toEqual([id]);
    useAssemblyStore.getState().deleteModule(id);
    expect(Object.keys(useAssemblyStore.getState().assembly.modules)).toHaveLength(0);
  });

  it('connecting two connectors locks both sides and records an edge', () => {
    const idA = useAssemblyStore.getState().addModule();
    const idB = useAssemblyStore.getState().addModule();
    const { assembly, connectConnectors } = useAssemblyStore.getState();
    const connA = assembly.modules[idA]!.connectorB;
    const connB = assembly.modules[idB]!.connectorA;

    connectConnectors(connA.id, connB.id);

    const updated = useAssemblyStore.getState().assembly;
    expect(updated.modules[idA]!.connectorB.locked).toBe(true);
    expect(updated.modules[idB]!.connectorA.locked).toBe(true);
    expect(updated.modules[idA]!.connectorB.connectedTo).toBe(connB.id);
    expect(updated.edges).toHaveLength(1);
  });

  it('deleting a module with an active lock frees its partner connector', () => {
    const idA = useAssemblyStore.getState().addModule();
    const idB = useAssemblyStore.getState().addModule();
    const { assembly, connectConnectors, deleteModule } = useAssemblyStore.getState();
    const connA = assembly.modules[idA]!.connectorB;
    const connB = assembly.modules[idB]!.connectorA;
    connectConnectors(connA.id, connB.id);

    deleteModule(idA);

    const updated = useAssemblyStore.getState().assembly;
    expect(updated.modules[idA]).toBeUndefined();
    expect(updated.modules[idB]!.connectorA.locked).toBe(false);
    expect(updated.modules[idB]!.connectorA.connectedTo).toBeNull();
    expect(updated.edges).toHaveLength(0);
  });

  it('undo reverses the last structural edit and redo restores it', () => {
    const id = useAssemblyStore.getState().addModule();
    expect(Object.keys(useAssemblyStore.getState().assembly.modules)).toHaveLength(1);

    useAssemblyStore.getState().undo();
    expect(Object.keys(useAssemblyStore.getState().assembly.modules)).toHaveLength(0);

    useAssemblyStore.getState().redo();
    expect(Object.keys(useAssemblyStore.getState().assembly.modules)).toEqual([id]);
  });

  it('clamps rod angle edits to the configured joint limits', () => {
    const id = useAssemblyStore.getState().addModule();
    const { setRodAngle } = useAssemblyStore.getState();
    // Rod 0 is TWIST with default 0..360deg range (radians internally).
    setRodAngle(id, 0, Math.PI * 10); // absurdly large -> should clamp to max
    const rod = useAssemblyStore.getState().assembly.modules[id]!.rods[0];
    expect(rod.angle).toBeCloseTo(rod.max, 5);
  });
});
