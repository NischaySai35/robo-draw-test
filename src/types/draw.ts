/**
 * Types for the Phase 2 "draw-to-build" mode. Kept separate from the core
 * module data model (`types/module.ts`) since a stroke is raw user input,
 * not part of the assembly graph, until it's fitted and applied.
 */
import type { ModuleId } from './module';

export type DrawDimensionality = '2d' | '3d';
export type DrawPlane = 'XY' | 'XZ' | 'YZ';
export type StrokeMode = 'continuous' | 'complex';
export type AutoAddMode = 'auto' | 'ask' | 'never';

export interface DrawSettings {
  dimensionality: DrawDimensionality;
  /** Which plane a 2D sketch is drawn on. Ignored in 3D mode. */
  plane: DrawPlane;
  /** Lift a 2D sketch into 3D by dragging along the plane normal after drawing. 2D-only. */
  extrude: boolean;
  strokeMode: StrokeMode;
  autoAddMode: AutoAddMode;
  /** Max acceptable average distance (world units) between stroke and fitted chain. */
  toleranceWorldUnits: number;
}

export const DEFAULT_DRAW_SETTINGS: DrawSettings = {
  dimensionality: '2d',
  plane: 'XY',
  extrude: false,
  strokeMode: 'continuous',
  autoAddMode: 'ask',
  toleranceWorldUnits: 0.25,
};

/** A single freehand stroke: an ordered polyline of world-space points. */
export interface Stroke {
  id: string;
  points: [number, number, number][];
}

export interface FitDiagnostic {
  moduleId: ModuleId;
  rodIndex: number;
  message: string;
}

/** Result of fitting one stroke onto one chain of modules. */
export interface FitResult {
  strokeId: string;
  /** Solved joint angles per module, in chain order, one entry per rod (radians). */
  moduleAngles: Record<ModuleId, [number, number, number, number, number, number]>;
  /** Average distance between the stroke and the resulting chain's sampled path (world units). */
  residual: number;
  /** Worst single-point deviation (world units). */
  maxDeviation: number;
  diagnostics: FitDiagnostic[];
  withinTolerance: boolean;
}

export interface FeasibilityReport {
  strokeLength: number;
  /** Total straight-line reach of the modules currently assigned to this stroke. */
  capacityLength: number;
  modulesAvailable: number;
  modulesNeeded: number;
  deficit: number;
  status: 'fits' | 'needs-more-modules';
}
