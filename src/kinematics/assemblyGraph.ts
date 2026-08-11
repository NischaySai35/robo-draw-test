/**
 * Assembly-level graph traversal: propagates world transforms across locked
 * connector welds. Modules are peers connected via `Connector.connectedTo` --
 * there is no parent/child hierarchy, so we pick one module per connected
 * component as the traversal anchor (its `basePose` is authoritative) and
 * derive every other module's placement by walking outward through welds.
 */
import type { Assembly, Connector, ConnectorEnd, ConnectorId, ModuleId, Pose } from '../types/module';
import { allConnectors, isSideConnectorEnd, sideConnectorIndex } from '../types/module';
import { FLIP_X_180, composePoses, invertPose } from './frame';
import { computeModuleTransforms, localConnectorTransform, type ModuleWorldTransforms } from './forwardKinematics';

export function findConnector(assembly: Assembly, connectorId: ConnectorId): Connector | undefined {
  for (const module of Object.values(assembly.modules)) {
    const match = allConnectors(module).find((c) => c.id === connectorId);
    if (match) return match;
  }
  return undefined;
}

/** World pose of any one of a module's 6 connectors. */
export function connectorPose(transforms: ModuleWorldTransforms, end: ConnectorEnd): Pose {
  if (end === 'A') return transforms.connectorA;
  if (!isSideConnectorEnd(end)) return transforms.connectorB;
  return transforms.sides[sideConnectorIndex(end)];
}

/**
 * The world pose of the *weld partner* connector, given a locked connector's
 * own world pose: same position, outward normals opposed, zero gap (a 180
 * degree flip about the connector frame's local X axis, dome-to-dome flush).
 */
export function weldedPartnerPose(connectorWorldPose: Pose): Pose {
  return composePoses(connectorWorldPose, FLIP_X_180);
}

/** A weld the spanning-tree traversal could not use, because both of its modules were already placed. */
export interface CutEdge {
  a: Connector;
  b: Connector;
}

export interface AssemblyKinematics {
  transforms: Map<ModuleId, ModuleWorldTransforms>;
  /**
   * Welds left over once a spanning tree has been chosen -- each one closes a
   * kinematic loop. The tree traversal places modules through tree edges only,
   * so a cut edge's two connectors are wherever the joint angles happen to put
   * them and are NOT guaranteed to meet. Driving that gap to zero is the job of
   * `solveLoopClosure` (see kinematics/loopClosure.ts); until it runs, treat a
   * non-empty `cutEdges` as "this structure is not yet physically consistent".
   */
  cutEdges: CutEdge[];
}

/** Stable key for an undirected weld, so each one is only counted once. */
function weldKey(a: ConnectorId, b: ConnectorId): string {
  return a < b ? `${a}|${b}` : `${b}|${a}`;
}

/**
 * Computes world transforms for every module, and reports which welds the
 * traversal had to skip.
 *
 * Traversal: for each connected component, the first module encountered
 * (stable insertion order) is the anchor -- its `basePose` is used as-is.
 * Every other module in the component is reached via a BFS across locked
 * connector edges; its placement is derived by welding onto its already-
 * resolved neighbor and back-solving connector A's pose via the inverse of
 * its own local connector transform.
 *
 * The BFS visits each module once, so the welds it actually uses form a
 * spanning tree (actually a spanning forest, one tree per component). Any weld
 * whose far module was already placed is a `cutEdge` -- the loop-closing
 * constraint that this traversal, on its own, cannot satisfy.
 */
export function computeAssemblyKinematics(assembly: Assembly): AssemblyKinematics {
  const result = new Map<ModuleId, ModuleWorldTransforms>();
  const visited = new Set<ModuleId>();
  const treeWelds = new Set<string>();
  const allWelds = new Map<string, CutEdge>();

  for (const anchorId of Object.keys(assembly.modules)) {
    if (visited.has(anchorId)) continue;
    const anchor = assembly.modules[anchorId];
    visited.add(anchorId);
    result.set(anchorId, computeModuleTransforms(anchor.rods, anchor.basePose));

    const queue: ModuleId[] = [anchorId];
    while (queue.length > 0) {
      const currentId = queue.shift()!;
      const current = assembly.modules[currentId];
      const currentTransforms = result.get(currentId)!;

      for (const connector of allConnectors(current)) {
        if (!connector.locked || !connector.connectedTo) continue;
        const partner = findConnector(assembly, connector.connectedTo);
        if (!partner) continue;

        allWelds.set(weldKey(connector.id, partner.id), { a: connector, b: partner });
        if (visited.has(partner.moduleId)) continue;

        const partnerModule = assembly.modules[partner.moduleId];
        const partnerConnectorWorldPose = weldedPartnerPose(
          connectorPose(currentTransforms, connector.end),
        );

        // Back out to the partner's connector-A frame, which is what
        // `computeModuleTransforms` needs as its anchor. Which connector we
        // arrived through decides how far back that is -- see
        // `localConnectorTransform`.
        const connectorAWorldPose = composePoses(
          partnerConnectorWorldPose,
          invertPose(localConnectorTransform(partnerModule.rods, partner.end)),
        );

        treeWelds.add(weldKey(connector.id, partner.id));
        visited.add(partner.moduleId);
        result.set(partner.moduleId, computeModuleTransforms(partnerModule.rods, connectorAWorldPose));
        queue.push(partner.moduleId);
      }
    }
  }

  const cutEdges: CutEdge[] = [];
  for (const [key, edge] of allWelds) {
    if (!treeWelds.has(key)) cutEdges.push(edge);
  }

  return { transforms: result, cutEdges };
}

/**
 * World transforms only. Callers that need to know whether the result is
 * actually consistent -- i.e. whether any loop-closing weld was ignored to
 * produce it -- should use `computeAssemblyKinematics` instead.
 */
export function computeAssemblyWorldTransforms(
  assembly: Assembly,
): Map<ModuleId, ModuleWorldTransforms> {
  return computeAssemblyKinematics(assembly).transforms;
}

/**
 * Number of independent kinematic loops -- the cyclomatic number
 * `|E| - |V| + components` of the weld graph. Each one costs up to 6 degrees
 * of freedom, since a MODULINK weld is rigid (it transmits moment) rather than
 * spherical.
 */
export function loopCount(assembly: Assembly): number {
  return computeAssemblyKinematics(assembly).cutEdges.length;
}

/**
 * The traversal-anchor module id for whichever connected component
 * `moduleId` belongs to (see `computeAssemblyWorldTransforms`). Every module
 * in a rigid, locked-together sub-chain shares the same anchor, which is
 * what lets a gizmo drag move the whole sub-chain by only ever repositioning
 * the anchor's `basePose`.
 */
export function anchorModuleId(assembly: Assembly, moduleId: ModuleId): ModuleId {
  const component = connectedComponents(assembly).find((c) => c.includes(moduleId));
  return component?.[0] ?? moduleId;
}

/**
 * Computes the anchor `basePose` a module's rigid sub-chain would need so
 * that `connectorId` ends up welded flush onto `targetConnectorId` -- the
 * same rigid-body math the viewport's drag-to-snap uses, exposed for the
 * inspector's explicit "Connect" control (Phase 1 requires locking to be
 * reachable without drag-and-drop too).
 */
export function computeWeldAnchorPose(
  assembly: Assembly,
  connectorId: ConnectorId,
  targetConnectorId: ConnectorId,
): { anchorId: ModuleId; anchorPose: Pose } | null {
  const connector = findConnector(assembly, connectorId);
  const target = findConnector(assembly, targetConnectorId);
  if (!connector || !target) return null;

  const transforms = computeAssemblyWorldTransforms(assembly);
  const targetPose = connectorPose(transforms.get(target.moduleId)!, target.end);
  const desiredPose = weldedPartnerPose(targetPose);

  const anchorId = anchorModuleId(assembly, connector.moduleId);
  const anchorPoseOld = transforms.get(anchorId)!.connectorA;
  const selectedPoseOld = connectorPose(transforms.get(connector.moduleId)!, connector.end);
  const rigidOffset = composePoses(invertPose(anchorPoseOld), selectedPoseOld);

  return { anchorId, anchorPose: composePoses(desiredPose, invertPose(rigidOffset)) };
}

/** Connected-component grouping of module ids, following locked connector edges. */
export function connectedComponents(assembly: Assembly): ModuleId[][] {
  const visited = new Set<ModuleId>();
  const components: ModuleId[][] = [];

  for (const startId of Object.keys(assembly.modules)) {
    if (visited.has(startId)) continue;
    const component: ModuleId[] = [];
    const queue: ModuleId[] = [startId];
    visited.add(startId);

    while (queue.length > 0) {
      const currentId = queue.shift()!;
      component.push(currentId);
      const current = assembly.modules[currentId];
      for (const connector of allConnectors(current)) {
        if (!connector.locked || !connector.connectedTo) continue;
        const partner = findConnector(assembly, connector.connectedTo);
        if (partner && !visited.has(partner.moduleId)) {
          visited.add(partner.moduleId);
          queue.push(partner.moduleId);
        }
      }
    }
    components.push(component);
  }
  return components;
}
