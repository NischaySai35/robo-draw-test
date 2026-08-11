/**
 * Orchestration for the cube-builder mode: delete-with-ambiguity handling
 * and voxel-graph -> module-assembly conversion. The conversion reuses
 * Phase 2's curve fit -- each maximal *path* through the voxel grid (see
 * `voxelPaths`) becomes one "stroke" of cube centers, so a long or gently
 * turning run of cubes gets sized for its whole length, not one chain per
 * individual cube-to-cube edge. Fits are never refused for being short of
 * the "ideal" module count -- the solver already reports an honest residual
 * for a partial chain, so building as much of the shape as fits is just... applying that result.
 */
import type { VoxelCoord, VoxelId } from '../types/voxel';
import { coordKey, voxelPaths, voxelConnectedComponents, voxelWorldCenter } from '../kinematics/voxelGraph';
import { buildFittedChain, computeChainStartPoseFromStroke, computeFeasibility, strokeArcLength } from '../kinematics/curveFit';
import { anchorKeyId, findWeldAnchor, type FittedChainRef } from '../kinematics/branchAnchor';
import type { Stroke, FitResult } from '../types/draw';
import type { Assembly, ConnectorEnd, ModuleId, Pose } from '../types/module';
import { connectorByEnd } from '../types/module';
import { Vector3 } from 'three';
import { useVoxelStore } from '../state/voxelStore';
import { useAssemblyStore } from '../state/assemblyStore';
import { useUIStore } from '../state/uiStore';

const CONVERSION_TOLERANCE = 0.15;

export async function deleteVoxelWithConfirmation(id: VoxelId): Promise<void> {
  const { graph, removeVoxel, removeVoxels } = useVoxelStore.getState();
  const { pushWarning, requestChoice } = useUIStore.getState();

  if (Object.keys(graph.voxels).length <= 1) {
    pushWarning('Cannot delete the last cube -- add another first, or use Clear.', 'error');
    return;
  }

  const componentsAfter = voxelConnectedComponents(graph, id);
  if (componentsAfter.length <= 1) {
    removeVoxel(id);
    return;
  }

  // Ambiguous: removing this cube splits the structure. Ask which handling the user wants.
  const choice = await requestChoice(
    `Deleting this cube splits the structure into ${componentsAfter.length} separate pieces. Orphan keeps every piece where it is; Cascade removes every piece except the largest.`,
    [
      { label: 'Cancel', value: 'cancel' },
      { label: 'Orphan', value: 'orphan' },
      { label: 'Cascade', value: 'cascade', danger: true },
    ],
  );
  if (!choice || choice === 'cancel') return;

  if (choice === 'orphan') {
    removeVoxel(id);
    pushWarning('Cube removed -- structure split into separate pieces.', 'warning');
    return;
  }

  const sortedByLargest = [...componentsAfter].sort((a, b) => b.length - a.length);
  const toRemove = [id, ...sortedByLargest.slice(1).flat()];
  removeVoxels(toRemove);
  pushWarning(`Cascaded: removed ${toRemove.length} cube${toRemove.length === 1 ? '' : 's'}.`, 'warning');
}

/** Where a branching arm welds onto an already-placed chain. */
export interface PathAnchorWeld {
  hostChainKey: string;
  /** Index of the host module within its chain, in chain order. */
  hostModuleIndex: number;
  hostEnd: ConnectorEnd;
  /** How far the host connector sits from the branch cube it stands in for. */
  offsetFromBranch: number;
}

export interface PathChainPreview {
  pathKey: string;
  path: VoxelCoord[];
  assembly: Assembly;
  fitResult: FitResult;
  moduleCount: number;
  /** Null for the trunk chain of each component, set for every arm that found a weld. */
  anchorWeld: PathAnchorWeld | null;
}

export interface VoxelConversionPreview {
  paths: PathChainPreview[];
  /** Branch arms that found no usable connector and stay free-floating. */
  branchPointWarnings: string[];
  /** How many arms are welded onto another chain's connector. */
  branchWeldCount: number;
}

interface PlannedPath {
  /** Oriented so index 0 is the cube this arm attaches at, when it has one. */
  path: VoxelCoord[];
  anchorCube: VoxelCoord | null;
}

/**
 * Orders the raw paths so that every path except the first of its component is
 * visited only after a path it shares a cube with -- which is what lets an arm
 * be fitted outward from a connector on an already-placed chain instead of
 * being positioned blind and hoping the two line up.
 *
 * Paths only ever share an endpoint at a branch cube (the decomposition splits
 * exactly there), so the shared-endpoint map below is the full adjacency. Each
 * component is seeded with its longest path -- the most trunk-like one -- and
 * every other path in the component hangs off it, oriented to grow away from
 * wherever it attached.
 */
function planPathTraversal(paths: VoxelCoord[][]): PlannedPath[] {
  const remaining = new Set(paths.map((_, i) => i));
  const byEndpoint = new Map<VoxelId, number[]>();
  paths.forEach((path, i) => {
    const ends = new Set([coordKey(path[0]!), coordKey(path[path.length - 1]!)]);
    for (const key of ends) {
      const list = byEndpoint.get(key) ?? [];
      list.push(i);
      byEndpoint.set(key, list);
    }
  });

  const planned: PlannedPath[] = [];

  while (remaining.size > 0) {
    let seed = -1;
    for (const i of remaining) {
      if (seed < 0 || paths[i]!.length > paths[seed]!.length) seed = i;
    }
    remaining.delete(seed);
    planned.push({ path: paths[seed]!, anchorCube: null });

    const queue: VoxelCoord[][] = [paths[seed]!];
    while (queue.length > 0) {
      const current = queue.shift()!;
      for (const endCube of [current[0]!, current[current.length - 1]!]) {
        const key = coordKey(endCube);
        for (const i of byEndpoint.get(key) ?? []) {
          if (!remaining.has(i)) continue;
          remaining.delete(i);
          const raw = paths[i]!;
          const oriented = coordKey(raw[0]!) === key ? raw : [...raw].reverse();
          planned.push({ path: oriented, anchorCube: endCube });
          queue.push(oriented);
        }
      }
    }
  }

  return planned;
}

/**
 * Builds one fitted module chain per maximal path through the voxel grid
 * (see `voxelPaths`), sized for that whole path's length -- not one chain per
 * individual edge -- and welds the chains to each other where they branch.
 *
 * Chains are fitted in dependency order (see `planPathTraversal`) so that by
 * the time an arm is fitted, whatever it branches off has already been placed
 * and its connectors have real world poses. The arm then grabs the nearest
 * free connector facing its way -- usually one of the 4 riding a big rod --
 * and is fitted outward from that exact weld point, so the joint is exact by
 * construction rather than something we hope lines up afterwards. The arm's
 * stroke starts at the connector instead of the branch cube, since the chain
 * being welded onto already covers that cube.
 *
 * Takes `graph` explicitly (defaulting to the store's current one) so callers
 * memoizing on it -- e.g. `VoxelInspector`'s `useMemo` -- have a real,
 * traceable dependency.
 */
export function computeVoxelConversionPreview(
  graph = useVoxelStore.getState().graph,
): VoxelConversionPreview {
  const planned = planPathTraversal(voxelPaths(graph));

  const pathPreviews: PathChainPreview[] = [];
  const fitted: FittedChainRef[] = [];
  const consumed = new Set<string>();
  const branchPointWarnings: string[] = [];

  for (const { path, anchorCube } of planned) {
    const pathKey = path.map((c) => c.join(',')).join('_');
    const centers = path.map((coord) => voxelWorldCenter(coord, graph.cellSize));

    let points = centers;
    let chainStartPose: Pose | null = null;
    let anchorWeld: PathAnchorWeld | null = null;

    if (anchorCube) {
      const branchCenter = new Vector3(...voxelWorldCenter(anchorCube, graph.cellSize));
      const armDirection = new Vector3(...centers[1]!).sub(branchCenter);
      const anchor = findWeldAnchor(fitted, branchCenter, armDirection, consumed);

      if (anchor) {
        // `buildFittedChain` wants a chain-FORWARD frame (+Z = direction of travel)
        // and applies the weld flip itself to get connector A's pose. The host
        // connector's own pose already faces the way this arm runs, so it IS that
        // forward frame -- flipping it here first would double-flip and send the
        // arm growing back into its host.
        chainStartPose = anchor.pose;
        points = [anchor.pose.position, ...centers.slice(1)];
        anchorWeld = {
          hostChainKey: anchor.chainKey,
          hostModuleIndex: anchor.moduleIndex,
          hostEnd: anchor.end,
          offsetFromBranch: anchor.distance,
        };
        consumed.add(anchorKeyId(anchor));
        // This arm's own connector A is spent on the weld too.
        consumed.add(anchorKeyId({ chainKey: pathKey, moduleIndex: 0, end: 'A' }));
      } else {
        branchPointWarnings.push(
          `Cube (${anchorCube.join(', ')}): no free connector faces this arm, so its chain is left unlocked. Freeing up a connector nearby, or nudging the arm, usually resolves it.`,
        );
      }
    }

    const stroke: Stroke = { id: pathKey, points };
    const length = strokeArcLength(points.map((p) => new Vector3(...p)));
    const moduleCount = Math.max(1, computeFeasibility(length, 0).modulesNeeded);

    const { assembly, fitResult } = buildFittedChain(
      stroke,
      moduleCount,
      chainStartPose ?? computeChainStartPoseFromStroke(stroke),
      CONVERSION_TOLERANCE,
    );

    pathPreviews.push({ pathKey, path, assembly, fitResult, moduleCount, anchorWeld });
    fitted.push({ chainKey: pathKey, assembly });
  }

  return {
    paths: pathPreviews,
    branchPointWarnings,
    branchWeldCount: pathPreviews.filter((p) => p.anchorWeld).length,
  };
}

/** Commits every path's fitted chain into the real assembly, after a single summary confirmation. */
export async function applyVoxelConversion(preview: VoxelConversionPreview): Promise<void> {
  const { pushWarning, requestConfirm } = useUIStore.getState();
  if (preview.paths.length === 0) {
    pushWarning('No cube-to-cube connections to convert.', 'error');
    return;
  }

  const totalModules = preview.paths.reduce((sum, p) => sum + p.moduleCount, 0);
  const confirmed = await requestConfirm(
    `This will create ${totalModules} module${totalModules === 1 ? '' : 's'} across ${preview.paths.length} chain${preview.paths.length === 1 ? '' : 's'}. Continue?`,
    'Create modules',
  );
  if (!confirmed) return;

  const weldsApplied = commitVoxelConversion(preview);
  const weldNote = weldsApplied > 0 ? `, ${weldsApplied} branch weld${weldsApplied === 1 ? '' : 's'}` : '';
  pushWarning(
    `Converted: ${totalModules} modules across ${preview.paths.length} chains${weldNote}.`,
    'warning',
  );
}

/**
 * Creates the real modules for a conversion preview and locks them -- both
 * along each chain and across chains at branch points. Split out from
 * `applyVoxelConversion` so the resulting structure can be asserted on without
 * standing up the confirmation dialog. Returns the number of branch welds made.
 */
export function commitVoxelConversion(preview: VoxelConversionPreview): number {
  const { addModule, setRodAngle, setModuleBasePose, connectConnectors } = useAssemblyStore.getState();

  const realIdsByChain = new Map<string, ModuleId[]>();

  for (const pathPreview of preview.paths) {
    const previewModules = Object.values(pathPreview.assembly.modules);
    const realIds = previewModules.map((previewModule, i) => {
      const id = addModule();
      previewModule.rods.forEach((rod, rodIndex) => setRodAngle(id, rodIndex, rod.angle));
      if (i === 0) setModuleBasePose(id, previewModule.basePose);
      return id;
    });
    realIdsByChain.set(pathPreview.pathKey, realIds);
    for (let i = 0; i < realIds.length - 1; i += 1) {
      const currentModule = useAssemblyStore.getState().assembly.modules[realIds[i]!]!;
      const nextModule = useAssemblyStore.getState().assembly.modules[realIds[i + 1]!]!;
      connectConnectors(currentModule.connectorB.id, nextModule.connectorA.id);
    }
  }

  // Cross-chain branch welds go last, once every chain's modules exist. Each
  // arm was fitted starting exactly at its host connector's welded pose, so
  // locking here needs no repositioning -- the geometry already lines up.
  let weldsApplied = 0;
  for (const pathPreview of preview.paths) {
    const weld = pathPreview.anchorWeld;
    if (!weld) continue;
    const hostIds = realIdsByChain.get(weld.hostChainKey);
    const armIds = realIdsByChain.get(pathPreview.pathKey);
    const hostId = hostIds?.[weld.hostModuleIndex];
    const armId = armIds?.[0];
    if (!hostId || !armId) continue;

    const assembly = useAssemblyStore.getState().assembly;
    const hostConnector = connectorByEnd(assembly.modules[hostId]!, weld.hostEnd);
    const armConnector = assembly.modules[armId]!.connectorA;
    if (hostConnector.locked || armConnector.locked) continue;
    connectConnectors(hostConnector.id, armConnector.id);
    weldsApplied += 1;
  }

  return weldsApplied;
}
