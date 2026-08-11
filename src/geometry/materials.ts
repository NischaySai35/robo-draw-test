/**
 * Shared materials, indexed by visual state. Modules swap `mesh.material` to
 * one of these singletons rather than cloning per-instance -- color only
 * depends on state (locked/energized), never on module identity, so sharing
 * is both cheaper and simpler than per-instance materials. Per-selection
 * highlighting is layered on top separately (see EditModeController), not
 * baked into these.
 *
 * Every solid material is `side: DoubleSide` -- Three.js defaults to
 * FrontSide, which culls backfaces entirely, so orbiting or zooming the
 * camera *inside* any mesh (easy to do at this model's scale) made it go
 * invisible. Blender's viewport doesn't do that; ours shouldn't either.
 */
import { BackSide, DoubleSide, MeshStandardMaterial } from 'three';
import { COLORS } from '../constants/geometry';

export const materials = {
  hemisphereLocked: new MeshStandardMaterial({
    color: COLORS.hemisphereLocked,
    roughness: 0.4,
    metalness: 0.15,
    side: DoubleSide,
  }),
  hemisphereUnlocked: new MeshStandardMaterial({
    color: COLORS.hemisphereUnlocked,
    roughness: 0.6,
    metalness: 0.05,
    side: DoubleSide,
  }),
  rod: new MeshStandardMaterial({
    color: COLORS.rodTwist,
    roughness: 0.55,
    metalness: 0.1,
    side: DoubleSide,
  }),
  stripeEnergized: new MeshStandardMaterial({
    color: COLORS.stripeEnergized,
    roughness: 0.35,
    emissive: COLORS.stripeEnergized,
    emissiveIntensity: 0.3,
    side: DoubleSide,
  }),
  stripeDeenergized: new MeshStandardMaterial({
    color: COLORS.stripeDeenergized,
    roughness: 0.7,
    side: DoubleSide,
  }),
  knuckleEnergized: new MeshStandardMaterial({
    color: COLORS.knuckleEnergized,
    roughness: 0.35,
    emissive: COLORS.knuckleEnergized,
    emissiveIntensity: 0.3,
    side: DoubleSide,
  }),
  knuckleDeenergized: new MeshStandardMaterial({
    color: COLORS.knuckleDeenergized,
    roughness: 0.7,
    side: DoubleSide,
  }),
  padlockLocked: new MeshStandardMaterial({
    color: COLORS.padlockLocked,
    roughness: 0.3,
    emissive: COLORS.padlockLocked,
    emissiveIntensity: 0.45,
    side: DoubleSide,
  }),
  padlockUnlocked: new MeshStandardMaterial({
    color: COLORS.padlockUnlocked,
    roughness: 0.6,
    side: DoubleSide,
  }),
  ghostSnap: new MeshStandardMaterial({
    color: COLORS.ghostSnap,
    transparent: true,
    opacity: 0.35,
    depthWrite: false,
    side: DoubleSide,
  }),
  /**
   * The selection outline is a deliberately inside-out rim: it's a slightly
   * enlarged clone of the selected mesh, and we want to see the *inside* of
   * that enlarged shell (so it reads as a halo around the real mesh) while
   * the real mesh occludes its front -- so this one intentionally stays
   * BackSide, not DoubleSide.
   */
  selectionOutline: new MeshStandardMaterial({
    color: COLORS.selection,
    emissive: COLORS.selection,
    emissiveIntensity: 0.8,
    side: BackSide,
  }),
  /** Translucent "ghost" chain used by draw-to-build to preview a fit before it's applied. */
  drawPreview: new MeshStandardMaterial({
    color: COLORS.ghostSnap,
    transparent: true,
    opacity: 0.55,
    depthWrite: false,
    roughness: 0.4,
    side: DoubleSide,
  }),
} as const;
