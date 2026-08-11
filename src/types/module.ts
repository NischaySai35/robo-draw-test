/**
 * Core data model for MODULINK.
 *
 * A "module" is a fixed 8-segment chain: hemisphere A -> 6 rods -> hemisphere B.
 * The assembly is a graph of modules connected peer-to-peer at their hemisphere
 * connectors -- there is no parent/child hierarchy. This file only defines the
 * shape of that data; forward kinematics and graph traversal live in
 * `src/kinematics/`, kept free of Three.js and React so they stay unit-testable.
 */

export type RodKind = 'twist' | 'bend';

/**
 * The 4 connectors riding the middle of the big spine rod, pointing radially
 * outward from it -- named for the rod-local direction each one faces. Order
 * is significant: `Module.sides` and `ModuleWorldTransforms.sides` are both
 * indexed by this array's order.
 */
export const SIDE_CONNECTOR_ENDS = ['UP', 'RIGHT', 'DOWN', 'LEFT'] as const;
export type SideConnectorEnd = (typeof SIDE_CONNECTOR_ENDS)[number];

/**
 * Every lock point on a module: the two chain ends plus the four side
 * connectors -- the 6 faces of a cube, metaphorically.
 */
export type ConnectorEnd = 'A' | 'B' | SideConnectorEnd;

export const CONNECTOR_ENDS: readonly ConnectorEnd[] = ['A', 'B', ...SIDE_CONNECTOR_ENDS];

export function isSideConnectorEnd(end: ConnectorEnd): end is SideConnectorEnd {
  return end !== 'A' && end !== 'B';
}

/** Index into `Module.sides` / `ModuleWorldTransforms.sides` for a side end. */
export function sideConnectorIndex(end: SideConnectorEnd): number {
  return SIDE_CONNECTOR_ENDS.indexOf(end);
}

export type RodId = string;
export type ConnectorId = string;
export type ModuleId = string;

/** A single revolute joint segment of the chain. */
export interface Rod {
  id: RodId;
  kind: RodKind;
  /** Current joint angle, radians. */
  angle: number;
  /** Lower joint limit, radians. */
  min: number;
  /** Upper joint limit, radians. */
  max: number;
  /** Angle the "Home" action drives this joint to, radians. */
  home: number;
  /** When true, the joint actively holds/drives to its commanded angle. */
  torqueEnabled: boolean;
}

/** A hemisphere connector -- a lock point, not a joint. */
export interface Connector {
  id: ConnectorId;
  moduleId: ModuleId;
  end: ConnectorEnd;
  locked: boolean;
  /** The connector this one is welded to, or null if free. */
  connectedTo: ConnectorId | null;
}

/** A rigid position + orientation. Plain data so kinematics stays pure. */
export interface Pose {
  position: [number, number, number];
  /** Quaternion, [x, y, z, w]. */
  quaternion: [number, number, number, number];
}

/**
 * One module: the fixed 8-segment chain.
 *
 * `basePose` is the world pose of connector A's frame and is only meaningful
 * when this module is the traversal anchor for its connected component (see
 * `computeAssemblyWorldTransforms`) -- for a free-floating module it's simply
 * where the module sits in the scene; for a module reached through a locked
 * connector, its world placement is derived instead and `basePose` is ignored.
 */
export interface Module {
  id: ModuleId;
  basePose: Pose;
  /** Ordered rods 1-6: TWIST, BEND, BEND, TWIST, BEND, TWIST. */
  rods: [Rod, Rod, Rod, Rod, Rod, Rod];
  connectorA: Connector;
  connectorB: Connector;
  /**
   * The 4 side connectors on the big spine rod, indexed in
   * `SIDE_CONNECTOR_ENDS` order. Same geometry and lock behaviour as A/B --
   * a module can be welded onto one of these to branch off sideways.
   */
  sides: [Connector, Connector, Connector, Connector];
}

/** Every connector on a module, chain ends first. Order matches `CONNECTOR_ENDS`. */
export function allConnectors(module: Module): Connector[] {
  return [module.connectorA, module.connectorB, ...module.sides];
}

export function connectorByEnd(module: Module, end: ConnectorEnd): Connector {
  if (end === 'A') return module.connectorA;
  if (end === 'B') return module.connectorB;
  return module.sides[sideConnectorIndex(end)];
}

/** An edge in the connector graph: two locked connectors welded together. */
export interface ConnectorEdge {
  a: ConnectorId;
  b: ConnectorId;
}

/** The whole scene: a graph of modules connected at their connectors. */
export interface Assembly {
  modules: Record<ModuleId, Module>;
  edges: ConnectorEdge[];
}

export const ROD_KIND_SEQUENCE: RodKind[] = ['twist', 'bend', 'bend', 'twist', 'bend', 'twist'];
