/**
 * Orchestration between draw-mode state, the pure curve-fit solver, and the
 * real assembly store. Nothing in `kinematics/curveFit.ts` touches Zustand
 * or shows a confirm dialog -- that wiring lives here so the solver itself
 * stays unit-testable in isolation.
 *
 * Fitting a shape is never an all-or-nothing affair: it's not always
 * possible (or required) to realize the whole drawn stroke, so the app
 * always fits and applies *as many modules as the current policy allows*,
 * reporting residual/deviation honestly instead of refusing outright.
 */
import {
  buildFittedChain,
  computeChainStartPoseFromStroke,
  computeFeasibility,
  strokeArcLength,
} from '../kinematics/curveFit';
import { Vector3 } from 'three';
import type { Assembly, Pose } from '../types/module';
import type { DrawPlane, FitResult, Stroke } from '../types/draw';
import { useDrawStore } from '../state/drawStore';
import { useAssemblyStore } from '../state/assemblyStore';
import { useUIStore } from '../state/uiStore';

/** World-space normal of a 2D draw plane -- matches `DrawModeController.currentPlane`. */
function planeNormalFor(plane: DrawPlane): Vector3 {
  if (plane === 'XZ') return new Vector3(0, 1, 0);
  if (plane === 'YZ') return new Vector3(1, 0, 0);
  return new Vector3(0, 0, 1); // XY
}

export interface DrawPreview {
  assembly: Assembly;
  fitResult: FitResult;
  chainStartPose: Pose;
  /** How many modules this preview was actually fitted with. */
  moduleCount: number;
  /** How many modules a *full* fit of the whole stroke would ideally need. */
  idealModuleCount: number;
}

/**
 * Runs feasibility + the curve-fit solver for one stroke and returns a
 * throwaway preview assembly for rendering. Pure with respect to `drawStore`
 * -- it only *reads* current settings, never writes -- so it's safe to call
 * from a reactive store subscription (see `DrawModeController`) without
 * risking a write-triggers-itself loop. Use `persistFitForStroke` when you
 * actually want the result recorded for the status panel.
 *
 * `moduleCountOverride` lets a caller force a specific module count (used by
 * the "partial fit" choice in `applyFitToAssembly`); otherwise the count
 * used follows `autoAddMode`: `auto`/`ask` fit the full/ideal shape, `never`
 * never fits with more than what's already assigned.
 *
 * `refine` (default true) controls whether the solver's coordinate-descent
 * refinement sweeps run -- see `fitChainToStroke`'s doc comment. Pass
 * `false` from a reactive, still-being-dragged render path where speed
 * matters more than the last bit of quality (see `DrawModeController`);
 * leave it on for the "stroke just finished" / "Re-fit" / "Apply" recompute,
 * which is precisely where refinement's whole benefit -- letting more of the
 * curve retroactively pull earlier joints into a better shape -- applies.
 */
export function computeDrawPreview(stroke: Stroke, moduleCountOverride?: number, refine = true): DrawPreview | null {
  if (stroke.points.length < 2) return null;

  const { settings, assignedModuleCount } = useDrawStore.getState();
  const strokePoints = stroke.points.map((p) => new Vector3(...p));
  const length = strokeArcLength(strokePoints);
  const feasibility = computeFeasibility(length, assignedModuleCount);
  const idealModuleCount = Math.max(1, feasibility.modulesNeeded);

  const policyModuleCount =
    settings.autoAddMode === 'never' ? Math.max(1, Math.min(idealModuleCount, assignedModuleCount || 1)) : idealModuleCount;
  const moduleCount = moduleCountOverride ?? policyModuleCount;

  // A 2D-mode stroke is confined to a known plane -- pin the fit's frame to that
  // plane's exact normal so BEND (which only ever rotates about that axis) can't
  // drift the chain off the plane the user actually drew on.
  const planeHint = settings.dimensionality === '2d' ? planeNormalFor(settings.plane) : undefined;
  const chainStartPose = computeChainStartPoseFromStroke(stroke, planeHint);
  const { assembly, fitResult } = buildFittedChain(stroke, moduleCount, chainStartPose, settings.toleranceWorldUnits, refine);

  return { assembly, fitResult, chainStartPose, moduleCount, idealModuleCount };
}

/**
 * `computeDrawPreview` plus recording the feasibility/fit result into
 * `drawStore` for the status panel. Call this from discrete events (stroke
 * finished, "Re-fit" button) -- never from a `drawStore` subscription, since
 * every write here would re-trigger that subscription. Always runs full
 * refinement (this is exactly the "settled" moment it's for).
 */
export function persistFitForStroke(stroke: Stroke): DrawPreview | null {
  const preview = computeDrawPreview(stroke);
  if (!preview) return null;
  const { assignedModuleCount, setFeasibility, setFitResult } = useDrawStore.getState();
  const strokePoints = stroke.points.map((p) => new Vector3(...p));
  setFeasibility(stroke.id, computeFeasibility(strokeArcLength(strokePoints), assignedModuleCount));
  setFitResult(stroke.id, preview.fitResult);
  return preview;
}

function commitPreview(preview: DrawPreview): number {
  const previewModules = Object.values(preview.assembly.modules);
  const { addModule, setRodAngle, setModuleBasePose, connectConnectors } = useAssemblyStore.getState();

  const realIds = previewModules.map((previewModule, i) => {
    const id = addModule();
    previewModule.rods.forEach((rod, rodIndex) => setRodAngle(id, rodIndex, rod.angle));
    if (i === 0) setModuleBasePose(id, previewModule.basePose);
    return id;
  });

  for (let i = 0; i < realIds.length - 1; i += 1) {
    const currentModule = useAssemblyStore.getState().assembly.modules[realIds[i]!]!;
    const nextModule = useAssemblyStore.getState().assembly.modules[realIds[i + 1]!]!;
    connectConnectors(currentModule.connectorB.id, nextModule.connectorA.id);
  }
  return realIds.length;
}

/**
 * Commits a fitted preview into the real assembly: creates the fitted
 * modules, applies their solved rod angles, and chain-locks them in order.
 * Never refuses to build just because the stroke isn't fully covered --
 * `autoAddMode` only controls *how many* modules get used, never whether
 * to apply at all:
 *  - `auto`: uses the full/ideal module count without asking.
 *  - `ask`: if that's more than what's already assigned, offers a genuine
 *    choice between a partial fit (assigned count) and a full fit (adds
 *    the rest) -- Cancel is always an option too.
 *  - `never`: the preview was already fitted at the assigned count, so this
 *    just applies it -- always a partial (or exact) fit, never a refusal.
 */
export async function applyFitToAssembly(stroke: Stroke, preview: DrawPreview): Promise<void> {
  const { settings, assignedModuleCount } = useDrawStore.getState();
  const { pushWarning, requestChoice } = useUIStore.getState();

  let finalPreview = preview;

  if (settings.autoAddMode === 'ask' && preview.idealModuleCount > assignedModuleCount) {
    const partialCount = Math.max(1, assignedModuleCount);
    const choice = await requestChoice(
      `Full coverage of this stroke needs ${preview.idealModuleCount} modules, but only ${assignedModuleCount} are assigned. Fit with just what's assigned, or add the rest for a full fit?`,
      [
        { label: 'Cancel', value: 'cancel' },
        { label: `Partial fit (${partialCount})`, value: 'partial' },
        { label: `Full fit (add to ${preview.idealModuleCount})`, value: 'full' },
      ],
    );
    if (!choice || choice === 'cancel') return;
    if (choice === 'partial' && partialCount !== preview.moduleCount) {
      const recomputed = computeDrawPreview(stroke, partialCount);
      if (recomputed) finalPreview = recomputed;
    }
  }

  const created = commitPreview(finalPreview);
  pushWarning(
    `Applied: ${created} module${created === 1 ? '' : 's'} fitted (residual ${finalPreview.fitResult.residual.toFixed(3)})${
      finalPreview.moduleCount < finalPreview.idealModuleCount ? ' -- partial fit, shape continues beyond this chain.' : '.'
    }`,
    finalPreview.fitResult.withinTolerance ? 'warning' : 'error',
  );
  // The stroke has now been realized as real modules -- drop it from the sketch so
  // the draw canvas doesn't keep showing an already-applied stroke as still pending.
  useDrawStore.getState().removeStroke(stroke.id);
}
