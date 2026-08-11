import { beforeEach, describe, expect, it } from 'vitest';
import { useVoxelStore } from '../state/voxelStore';
import { commitVoxelConversion, computeVoxelConversionPreview } from '../ui/voxelActions';
import { useAssemblyStore } from '../state/assemblyStore';
import { coordKey } from '../kinematics/voxelGraph';
import {
  computeAssemblyWorldTransforms,
  connectedComponents,
  connectorPose,
  findConnector,
} from '../kinematics/assemblyGraph';
import { outwardNormal, worldPosition } from '../kinematics/frame';
import { MODULE_CHAIN_LENGTH } from '../constants/geometry';

function resetVoxelStore(cellSize: number) {
  useVoxelStore.setState({
    graph: { cellSize, voxels: { '0,0,0': { id: '0,0,0', coord: [0, 0, 0] } } },
    selectedVoxelId: null,
    undoStack: [],
    redoStack: [],
  });
}

describe('computeVoxelConversionPreview', () => {
  it('collapses a straight run of cubes into one chain, not one per edge', () => {
    resetVoxelStore(2);
    useVoxelStore.getState().addVoxel([1, 0, 0]);
    useVoxelStore.getState().addVoxel([2, 0, 0]);

    const preview = computeVoxelConversionPreview();
    expect(preview.paths).toHaveLength(1);
    expect(preview.branchPointWarnings).toHaveLength(0);
  });

  it('uses more than one module when the path length exceeds one module\'s reach', () => {
    const bigCellSize = MODULE_CHAIN_LENGTH * 3;
    resetVoxelStore(bigCellSize);
    useVoxelStore.getState().addVoxel([1, 0, 0]);

    const preview = computeVoxelConversionPreview();
    expect(preview.paths).toHaveLength(1);
    expect(preview.paths[0]!.moduleCount).toBeGreaterThan(1);
  });

  it('welds every arm of a branch point onto an already-placed chain', () => {
    resetVoxelStore(2);
    useVoxelStore.getState().addVoxel([1, 0, 0]);
    useVoxelStore.getState().addVoxel([-1, 0, 0]);
    useVoxelStore.getState().addVoxel([0, 1, 0]);

    const preview = computeVoxelConversionPreview();
    expect(preview.paths).toHaveLength(3);
    // One trunk plus two arms, and each arm found a connector to weld onto --
    // this is what the 4 big-rod side connectors bought us; before them a
    // branch cube left the extra arms floating unlocked.
    expect(preview.paths.filter((p) => p.anchorWeld === null)).toHaveLength(1);
    expect(preview.branchWeldCount).toBe(2);
    expect(preview.branchPointWarnings).toHaveLength(0);
  });

  it('welds each branch arm to a distinct connector, never double-booking one', () => {
    resetVoxelStore(2);
    useVoxelStore.getState().addVoxel([1, 0, 0]);
    useVoxelStore.getState().addVoxel([-1, 0, 0]);
    useVoxelStore.getState().addVoxel([0, 1, 0]);
    useVoxelStore.getState().addVoxel([0, -1, 0]);

    const preview = computeVoxelConversionPreview();
    const welds = preview.paths.map((p) => p.anchorWeld).filter((w) => w !== null);
    const keys = welds.map((w) => `${w!.hostChainKey}:${w!.hostModuleIndex}:${w!.hostEnd}`);
    expect(new Set(keys).size).toBe(keys.length);
  });

  it('commits a branching structure as ONE locked assembly, not separate floating chains', () => {
    resetVoxelStore(2);
    useVoxelStore.getState().addVoxel([1, 0, 0]);
    useVoxelStore.getState().addVoxel([-1, 0, 0]);
    useVoxelStore.getState().addVoxel([0, 1, 0]);
    useAssemblyStore.setState({ assembly: { modules: {}, edges: [] }, undoStack: [], redoStack: [] });

    const preview = computeVoxelConversionPreview();
    const welds = commitVoxelConversion(preview);

    expect(welds).toBe(2);
    const assembly = useAssemblyStore.getState().assembly;
    expect(Object.keys(assembly.modules).length).toBeGreaterThan(1);
    // The whole point: before side connectors this came out as 3 unlocked
    // components, one per arm.
    expect(connectedComponents(assembly)).toHaveLength(1);
  });

  it('leaves the welded geometry consistent after committing -- no snapping back into place', () => {
    resetVoxelStore(2);
    useVoxelStore.getState().addVoxel([1, 0, 0]);
    useVoxelStore.getState().addVoxel([-1, 0, 0]);
    useVoxelStore.getState().addVoxel([0, 1, 0]);
    useAssemblyStore.setState({ assembly: { modules: {}, edges: [] }, undoStack: [], redoStack: [] });

    commitVoxelConversion(computeVoxelConversionPreview());
    const assembly = useAssemblyStore.getState().assembly;
    const transforms = computeAssemblyWorldTransforms(assembly);

    // Every welded pair must still resolve coincident + opposed once the graph
    // derives placements through the locks.
    for (const edge of assembly.edges) {
      const a = findConnector(assembly, edge.a)!;
      const b = findConnector(assembly, edge.b)!;
      const poseA = connectorPose(transforms.get(a.moduleId)!, a.end);
      const poseB = connectorPose(transforms.get(b.moduleId)!, b.end);
      expect(worldPosition(poseA).distanceTo(worldPosition(poseB))).toBeCloseTo(0, 4);
      expect(outwardNormal(poseA).dot(outwardNormal(poseB))).toBeCloseTo(-1, 4);
    }
  });

  it('starts a branch arm exactly at the connector it welds onto', () => {
    resetVoxelStore(2);
    useVoxelStore.getState().addVoxel([1, 0, 0]);
    useVoxelStore.getState().addVoxel([-1, 0, 0]);
    useVoxelStore.getState().addVoxel([0, 1, 0]);

    const preview = computeVoxelConversionPreview();
    const arm = preview.paths.find((p) => p.anchorWeld !== null)!;
    const host = preview.paths.find((p) => p.pathKey === arm.anchorWeld!.hostChainKey)!;

    const hostModules = Object.values(host.assembly.modules);
    const hostModule = hostModules[arm.anchorWeld!.hostModuleIndex]!;
    const hostPose = connectorPose(
      computeAssemblyWorldTransforms(host.assembly).get(hostModule.id)!,
      arm.anchorWeld!.hostEnd,
    );

    const armModules = Object.values(arm.assembly.modules);
    const armPose = computeAssemblyWorldTransforms(arm.assembly).get(armModules[0]!.id)!.connectorA;

    // Weld geometry: coincident positions, opposed outward normals. If this
    // holds the arm needs no repositioning when the lock is actually applied.
    expect(worldPosition(hostPose).distanceTo(worldPosition(armPose))).toBeCloseTo(0, 4);
    expect(outwardNormal(hostPose).dot(outwardNormal(armPose))).toBeCloseTo(-1, 4);
  });

  it('each path chain is internally locked start-to-end', () => {
    resetVoxelStore(2);
    useVoxelStore.getState().addVoxel([1, 0, 0]);
    const preview = computeVoxelConversionPreview();
    const modules = Object.values(preview.paths[0]!.assembly.modules);
    for (let i = 0; i < modules.length - 1; i += 1) {
      expect(modules[i]!.connectorB.locked).toBe(true);
      expect(modules[i + 1]!.connectorA.locked).toBe(true);
    }
  });
});

describe('voxelStore structural edits', () => {
  beforeEach(() => resetVoxelStore(2));

  it('addAdjacent spawns a row of cubes and stops at the first occupied cell', () => {
    useVoxelStore.getState().addVoxel([1, 0, 0]);
    const added = useVoxelStore.getState().addAdjacent([-2, 0, 0], '+x', 5);
    // From (-2,0,0) going +x: (-1,0,0), (0,0,0) occupied? no seed is (0,0,0) -- occupied, stop there.
    expect(added).toBe(1);
    expect(coordKey([-1, 0, 0]) in useVoxelStore.getState().graph.voxels).toBe(true);
  });

  it('moveVoxel refuses to disconnect a cube from the rest of the structure', () => {
    useVoxelStore.getState().addVoxel([1, 0, 0]);
    const ok = useVoxelStore.getState().moveVoxel(coordKey([1, 0, 0]), [10, 10, 10]);
    expect(ok).toBe(false);
  });

  it('moveVoxel allows a move that keeps at least one adjacency', () => {
    useVoxelStore.getState().addVoxel([1, 0, 0]);
    const ok = useVoxelStore.getState().moveVoxel(coordKey([1, 0, 0]), [0, 1, 0]);
    expect(ok).toBe(true);
    expect(coordKey([0, 1, 0]) in useVoxelStore.getState().graph.voxels).toBe(true);
  });

  it('undo restores the graph after a structural edit', () => {
    useVoxelStore.getState().addVoxel([1, 0, 0]);
    expect(Object.keys(useVoxelStore.getState().graph.voxels)).toHaveLength(2);
    useVoxelStore.getState().undo();
    expect(Object.keys(useVoxelStore.getState().graph.voxels)).toHaveLength(1);
  });
});
