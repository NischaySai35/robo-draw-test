/**
 * Pose <-> matrix helpers shared by forward kinematics and geometry building.
 *
 * We use Three.js's math classes (Vector3/Quaternion/Matrix4) here because
 * they're a solid, well-tested affine-transform library -- not because this
 * module touches rendering. Nothing here creates a scene, mesh, or material,
 * so it stays pure and unit-testable without a browser.
 */
import { Matrix4, Quaternion, Vector3 } from 'three';
import type { Pose } from '../types/module';

export const IDENTITY_POSE: Pose = {
  position: [0, 0, 0],
  quaternion: [0, 0, 0, 1],
};

export function poseToMatrix(pose: Pose): Matrix4 {
  const m = new Matrix4();
  m.compose(
    new Vector3(...pose.position),
    new Quaternion(...pose.quaternion),
    new Vector3(1, 1, 1),
  );
  return m;
}

export function matrixToPose(m: Matrix4): Pose {
  const position = new Vector3();
  const quaternion = new Quaternion();
  const scale = new Vector3();
  m.decompose(position, quaternion, scale);
  return {
    position: [position.x, position.y, position.z],
    quaternion: [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
  };
}

/** Compose `a` then `b` in local space: result = a * b (b applied in a's frame). */
export function composePoses(a: Pose, b: Pose): Pose {
  const result = new Matrix4().multiplyMatrices(poseToMatrix(a), poseToMatrix(b));
  return matrixToPose(result);
}

export function invertPose(pose: Pose): Pose {
  return matrixToPose(poseToMatrix(pose).clone().invert());
}

/** Local transform: translate along +Z by `distance`, no rotation. */
export function translateZ(distance: number): Pose {
  return { position: [0, 0, distance], quaternion: [0, 0, 0, 1] };
}

/** Local transform: rotate about the local X axis by `angle` radians. */
export function rotateX(angle: number): Pose {
  const q = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), angle);
  return { position: [0, 0, 0], quaternion: [q.x, q.y, q.z, q.w] };
}

/** Local transform: rotate about the local Y axis by `angle` radians. */
export function rotateY(angle: number): Pose {
  const q = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), angle);
  return { position: [0, 0, 0], quaternion: [q.x, q.y, q.z, q.w] };
}

/** Local transform: rotate about the local Z axis by `angle` radians. */
export function rotateZ(angle: number): Pose {
  const q = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), angle);
  return { position: [0, 0, 0], quaternion: [q.x, q.y, q.z, q.w] };
}

/** 180-degree flip about local X -- used to weld two connector frames dome-to-dome. */
export const FLIP_X_180: Pose = rotateX(Math.PI);

export function worldPosition(pose: Pose): Vector3 {
  return new Vector3(...pose.position);
}

/** The outward-facing normal of a connector frame is its local +Z axis in world space. */
export function outwardNormal(pose: Pose): Vector3 {
  return new Vector3(0, 0, 1).applyQuaternion(new Quaternion(...pose.quaternion)).normalize();
}
