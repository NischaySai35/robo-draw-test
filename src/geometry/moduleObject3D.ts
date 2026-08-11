/**
 * Builds and updates the Three.js representation of one module.
 *
 * Every part is positioned directly in world space (position + quaternion
 * taken straight from forward kinematics) rather than nested under a single
 * rigid parent transform -- a module's segments are NOT rigidly related to
 * each other once joints move, so each mesh owns its own world pose. The
 * `THREE.Group` returned here is purely an organizational container (kept at
 * the identity transform) so the whole module can be added/removed from the
 * scene and picked/raycast against as one unit.
 */
import { Group, Mesh, Quaternion, Vector3 } from 'three';
import type { ConnectorEnd, Module, Pose } from '../types/module';
import { CONNECTOR_ENDS, connectorByEnd } from '../types/module';
import type { ModuleWorldTransforms } from '../kinematics/forwardKinematics';
import { connectorPose } from '../kinematics/assemblyGraph';
import { BIG_ROD_INDEX, HEMISPHERE_RADIUS } from '../constants/geometry';
import { DOCK_NUB_PLACEMENTS, geometries, rodGeometryFor, stripeGeometryFor } from './primitives';
import { materials } from './materials';

/**
 * The padlock badge sits to the *side* of the connector (a blend of its local
 * X and Y axes), never along its local Z/outward-normal axis -- offsetting
 * along that axis used to shove the badge into/through whatever sat on the
 * other side of a locked joint.
 *
 * This must stay GREATER than `HEMISPHERE_RADIUS` (plus roughly the badge's
 * own half-width) so the badge clears the connector's rim and reads as a lug
 * mounted on the side. At a smaller radius it lands inside the rim, and since
 * the badge straddles the z=0 plane -- which is exactly where the connector's
 * flat mounting face sits -- it ends up half-sunk into that face, showing as a
 * stray square embedded in the disc.
 */
const PADLOCK_TANGENT_RADIUS = HEMISPHERE_RADIUS * 1.07;

interface RodRefs {
  mesh: Mesh;
  stripe: Mesh | null;
  knuckle: Mesh | null;
}

/** The meshes making up one of a module's 6 connectors. */
export interface ConnectorRefs {
  end: ConnectorEnd;
  mesh: Mesh;
  dockNubs: Mesh[];
  padlock: Group;
}

export interface ModuleObject3D {
  group: Group;
  /** All 6 connectors, keyed by end -- chain ends A/B plus the 4 big-rod sides. */
  connectors: Record<ConnectorEnd, ConnectorRefs>;
  rods: RodRefs[];
}

function buildPadlock(): Group {
  const group = new Group();
  const body = new Mesh(geometries.padlockBody, materials.padlockUnlocked);
  const shackle = new Mesh(geometries.padlockShackle, materials.padlockUnlocked);
  body.name = 'padlockBody';
  shackle.name = 'padlockShackle';
  group.add(body, shackle);
  return group;
}

/**
 * 4 small "lock nub" hemispheres keying the dock face, added as children of
 * a connector's hemisphere mesh so they automatically inherit its world
 * transform -- no separate per-frame positioning needed, just a material
 * sync to keep them matching the parent's locked/unlocked color. Each nub
 * sits ON the dome's surface with its own outward-radial orientation (see
 * `DOCK_NUB_PLACEMENTS`), so it visibly pokes out as a bump.
 */
function buildDockNubs(parent: Mesh): Mesh[] {
  return DOCK_NUB_PLACEMENTS.map((placement, i) => {
    const nub = new Mesh(geometries.dockNub, materials.hemisphereUnlocked);
    nub.position.copy(placement.position);
    nub.quaternion.copy(placement.quaternion);
    nub.name = `dockNub:${i}`;
    parent.add(nub);
    return nub;
  });
}

function buildConnector(group: Group, end: ConnectorEnd): ConnectorRefs {
  const mesh = new Mesh(geometries.hemisphere, materials.hemisphereUnlocked);
  mesh.name = `connector${end}`;
  const dockNubs = buildDockNubs(mesh);
  const padlock = buildPadlock();
  group.add(mesh, padlock);
  return { end, mesh, dockNubs, padlock };
}

export function createModuleObject3D(module: Module): ModuleObject3D {
  const group = new Group();
  group.name = `module:${module.id}`;

  const connectors = Object.fromEntries(
    CONNECTOR_ENDS.map((end) => [end, buildConnector(group, end)]),
  ) as Record<ConnectorEnd, ConnectorRefs>;

  const rods: RodRefs[] = module.rods.map((rod, i) => {
    const isBig = i === BIG_ROD_INDEX;
    const mesh = new Mesh(rodGeometryFor(rod.kind, isBig), materials.rod);
    mesh.name = `rod:${i}`;
    let stripe: Mesh | null = null;
    let knuckle: Mesh | null = null;
    if (rod.kind === 'twist') {
      stripe = new Mesh(stripeGeometryFor(isBig), materials.stripeDeenergized);
      mesh.add(stripe);
    } else {
      knuckle = new Mesh(geometries.knuckle, materials.knuckleDeenergized);
      knuckle.name = `knuckle:${i}`;
      group.add(knuckle);
    }
    group.add(mesh);
    return { mesh, stripe, knuckle };
  });

  return { group, connectors, rods };
}

const tmpTangent = new Vector3();
const tmpX = new Vector3();
const tmpY = new Vector3();

function placePadlock(padlock: Group, pose: Pose, locked: boolean) {
  const q = new Quaternion(...pose.quaternion);
  tmpX.set(1, 0, 0).applyQuaternion(q);
  tmpY.set(0, 1, 0).applyQuaternion(q);
  tmpTangent.copy(tmpX).add(tmpY).normalize().multiplyScalar(PADLOCK_TANGENT_RADIUS);
  padlock.position.set(
    pose.position[0] + tmpTangent.x,
    pose.position[1] + tmpTangent.y,
    pose.position[2] + tmpTangent.z,
  );
  padlock.quaternion.copy(q);
  const material = locked ? materials.padlockLocked : materials.padlockUnlocked;
  (padlock.children as Mesh[]).forEach((child) => {
    child.material = material;
  });
}

/** Applies fresh world transforms + visual state (locked/torque) to an existing module's meshes. */
export function updateModuleObject3D(
  refs: ModuleObject3D,
  module: Module,
  transforms: ModuleWorldTransforms,
): void {
  for (const end of CONNECTOR_ENDS) {
    const connectorRefs = refs.connectors[end];
    const connector = connectorByEnd(module, end);
    const pose = connectorPose(transforms, end);

    connectorRefs.mesh.position.set(...pose.position);
    connectorRefs.mesh.quaternion.set(...pose.quaternion);
    connectorRefs.mesh.material = connector.locked
      ? materials.hemisphereLocked
      : materials.hemisphereUnlocked;
    connectorRefs.dockNubs.forEach((nub) => { nub.material = connectorRefs.mesh.material; });

    placePadlock(connectorRefs.padlock, pose, connector.locked);
  }

  module.rods.forEach((rod, i) => {
    const pose = transforms.rods[i];
    const rodRefs = refs.rods[i];
    rodRefs.mesh.position.set(...pose.position);
    rodRefs.mesh.quaternion.set(...pose.quaternion);

    if (rodRefs.stripe) {
      rodRefs.stripe.material = rod.torqueEnabled ? materials.stripeEnergized : materials.stripeDeenergized;
    }
    if (rodRefs.knuckle) {
      rodRefs.knuckle.position.set(...pose.position);
      rodRefs.knuckle.quaternion.set(...pose.quaternion);
      rodRefs.knuckle.material = rod.torqueEnabled ? materials.knuckleEnergized : materials.knuckleDeenergized;
    }
  });
}

/**
 * Removes a module's group from the scene. Geometries/materials are shared
 * singletons owned by `primitives.ts`/`materials.ts`, so there's nothing
 * instance-specific left to dispose once the group is detached.
 */
export function disposeModuleObject3D(refs: ModuleObject3D): void {
  refs.group.removeFromParent();
}
