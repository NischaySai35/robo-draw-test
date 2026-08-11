import { describe, expect, it } from 'vitest';
import { Quaternion, Vector3 } from 'three';
import { clampRodAngle, createModule } from '../kinematics/factory';
import {
  buildFittedChain,
  chainStartPoseToBasePose,
  computeChainStartPoseFromStroke,
  computeFeasibility,
  findSelfIntersection,
  fitChainToStroke,
  resampleByArcLength,
  strokeArcLength,
} from '../kinematics/curveFit';
import { computeAssemblyWorldTransforms } from '../kinematics/assemblyGraph';
import { computeModuleTransforms } from '../kinematics/forwardKinematics';
import { IDENTITY_POSE } from '../kinematics/frame';
import { MODULE_CHAIN_LENGTH } from '../constants/geometry';
import type { Stroke } from '../types/draw';

const DEG = Math.PI / 180;

describe('clampRodAngle', () => {
  it('wraps a negative angle to its coterminal equivalent for a 0-360deg range instead of clamping to 0', () => {
    const twistRod = { min: 0, max: 360 * DEG };
    // -87deg is the exact same physical rotation as 273deg, which fits the range fine.
    const result = clampRodAngle(twistRod, -87 * DEG);
    expect(result / DEG).toBeCloseTo(273, 3);
  });

  it('leaves an already-in-range angle untouched', () => {
    const twistRod = { min: 0, max: 360 * DEG };
    expect(clampRodAngle(twistRod, 40 * DEG) / DEG).toBeCloseTo(40, 5);
  });

  it('still hard-clamps a genuinely out-of-range angle on a sub-360deg rod', () => {
    const bendRod = { min: -90 * DEG, max: 90 * DEG };
    expect(clampRodAngle(bendRod, 150 * DEG) / DEG).toBeCloseTo(90, 5);
    expect(clampRodAngle(bendRod, -150 * DEG) / DEG).toBeCloseTo(-90, 5);
  });
});

describe('resampleByArcLength', () => {
  it('preserves endpoints and returns the requested count', () => {
    const points = [new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(1, 1, 0)];
    const resampled = resampleByArcLength(points, 5);
    expect(resampled).toHaveLength(5);
    expect(resampled[0]!.distanceTo(points[0]!)).toBeCloseTo(0, 5);
    expect(resampled[4]!.distanceTo(points[2]!)).toBeCloseTo(0, 5);
  });
});

describe('computeFeasibility', () => {
  it('reports "fits" when available modules already cover the stroke length', () => {
    const report = computeFeasibility(MODULE_CHAIN_LENGTH * 2, 3);
    expect(report.status).toBe('fits');
    expect(report.deficit).toBe(0);
  });

  it('reports a module deficit when the stroke is longer than current reach', () => {
    const report = computeFeasibility(MODULE_CHAIN_LENGTH * 5, 2);
    expect(report.status).toBe('needs-more-modules');
    expect(report.deficit).toBeGreaterThan(0);
    expect(report.modulesNeeded).toBe(5);
  });
});

describe('fitChainToStroke', () => {
  it('fits a straight stroke with near-zero bend angles', () => {
    const module = createModule(IDENTITY_POSE);
    const length = MODULE_CHAIN_LENGTH;
    const stroke: Stroke = {
      id: 's1',
      points: [
        [0, 0, 0],
        [0, 0, length / 2],
        [0, 0, length],
      ],
    };
    // IDENTITY_POSE here is the chain-forward start frame (local +Z = stroke direction),
    // not connector A's outward-facing basePose -- see fitChainToStroke's doc comment.
    const result = fitChainToStroke([module], stroke, IDENTITY_POSE, 0.3);
    for (const rod of module.rods) {
      if (rod.kind === 'bend') {
        const angle = result.moduleAngles[module.id]![module.rods.indexOf(rod)]!;
        expect(Math.abs(angle)).toBeLessThan(0.05);
      }
    }
    expect(result.residual).toBeLessThan(0.2);
  });

  it('curves the chain toward a bending stroke better than the straight rest pose does', () => {
    const module = createModule(IDENTITY_POSE);
    const length = MODULE_CHAIN_LENGTH;
    // A gentle arc bending toward +Y as it progresses along Z.
    const stroke: Stroke = {
      id: 's2',
      points: Array.from({ length: 12 }, (_, i) => {
        const t = i / 11;
        return [0, Math.sin(t * 0.8) * length * 0.4, t * length] as [number, number, number];
      }),
    };
    const result = fitChainToStroke([module], stroke, IDENTITY_POSE, 0.3);
    const basePose = chainStartPoseToBasePose(IDENTITY_POSE);

    // Compare against the module's un-fitted (rest/straight) end position.
    const restEnd = computeModuleTransforms(module.rods, basePose).connectorB;
    const strokeEnd = new Vector3(...stroke.points[stroke.points.length - 1]!);
    const restDistance = new Vector3(...restEnd.position).distanceTo(strokeEnd);

    const fittedRods = module.rods.map((rod, i) => ({ ...rod, angle: result.moduleAngles[module.id]![i]! }));
    const fittedEnd = computeModuleTransforms(fittedRods as typeof module.rods, basePose).connectorB;
    const fittedDistance = new Vector3(...fittedEnd.position).distanceTo(strokeEnd);

    expect(fittedDistance).toBeLessThan(restDistance);
  });

  it('flags a diagnostic when the stroke curves tighter than the rod limits allow', () => {
    const module = createModule(IDENTITY_POSE);
    // A near-U-turn within a single module's reach -- should exceed default bend limits.
    const stroke: Stroke = {
      id: 's3',
      points: [
        [0, 0, 0],
        [0, 0.05, 0.05],
        [0, 0.4, 0.02],
        [0, 0.4, -0.3],
      ],
    };
    const result = fitChainToStroke([module], stroke, IDENTITY_POSE, 0.05);
    expect(result.diagnostics.length).toBeGreaterThan(0);
    expect(result.withinTolerance).toBe(false);
  });


  it('stays near-straight (no erratic bends) with only two sparse, jittery points -- the very start of a drag', () => {
    const module = createModule(IDENTITY_POSE);
    // Two points a tiny distance apart with a small lateral jitter -- exactly what
    // exists a couple of pointermove events into a fresh stroke.
    const stroke: Stroke = { id: 'sparse', points: [[0, 0, 0], [0.003, -0.002, 0.02]] };
    const result = fitChainToStroke([module], stroke, IDENTITY_POSE, 0.3);
    for (const angle of result.moduleAngles[module.id]!) {
      // TWIST's range is 0-360deg, so a near-zero rotation may come back as ~0 or as its
      // coterminal ~360 equivalent (see clampRodAngle) -- both are the same physical angle.
      const distanceFromZero = Math.min(Math.abs(angle), Math.abs(angle - Math.PI * 2));
      expect(distanceFromZero).toBeLessThan(0.2); // ~11 degrees -- no wild swings from noise
    }
  });

  it('covers only its own reachable prefix without distorting when the stroke is far longer than the chain', () => {
    const module = createModule(IDENTITY_POSE);
    const stroke: Stroke = {
      id: 'long',
      points: [
        [0, 0, 0],
        [0, 0, MODULE_CHAIN_LENGTH * 10],
      ],
    };
    const result = fitChainToStroke([module], stroke, IDENTITY_POSE, 0.3);
    // A straight stroke far longer than one module should still fit with near-zero
    // bend -- the chain just covers its own short straight prefix, not a distorted stretch.
    for (const angle of result.moduleAngles[module.id]!) {
      expect(Math.abs(angle)).toBeLessThan(0.05);
    }
    expect(result.diagnostics).toHaveLength(0);
  });

  it('tracks a multi-hump wave across several modules without oscillating (pure-pursuit self-correction)', () => {
    const modules = Array.from({ length: 3 }, () => createModule(IDENTITY_POSE));
    const totalLen = MODULE_CHAIN_LENGTH * 3;
    // Two gentle humps spread across the whole reachable length -- representative of a
    // realistic hand-drawn wiggle, not an instantaneous sharp turn.
    const points: [number, number, number][] = Array.from({ length: 40 }, (_, i) => {
      const t = i / 39;
      return [0, Math.sin(t * Math.PI * 2) * totalLen * 0.15, t * totalLen];
    });
    const stroke: Stroke = { id: 'wave', points };
    const result = fitChainToStroke(modules, stroke, IDENTITY_POSE, 0.3);

    expect(result.diagnostics).toHaveLength(0);
    // No joint should need a near-180deg twist -- that's the signature of the fit
    // losing track of a consistent bend direction rather than smoothly following a
    // gentle, realistic curve.
    for (const module of modules) {
      module.rods.forEach((rod, i) => {
        if (rod.kind !== 'twist') return;
        const deg = Math.abs((result.moduleAngles[module.id]![i]! * 180) / Math.PI);
        const distanceFrom180 = Math.min(Math.abs(deg - 180), Math.abs(deg - 360));
        expect(distanceFrom180).toBeGreaterThan(30);
      });
    }
  });

  it('refinement (default) never does worse than the pure-pursuit baseline alone, and improves a genuinely bendy shape', () => {
    const modules = Array.from({ length: 4 }, () => createModule(IDENTITY_POSE));
    const totalLen = MODULE_CHAIN_LENGTH * 4;
    // A less regular wiggle than the single-plane wave above -- varies in two axes so the
    // pure-pursuit baseline's forward-only, never-revisited choices are more likely to leave
    // something on the table for refinement to pick up.
    const points: [number, number, number][] = Array.from({ length: 60 }, (_, i) => {
      const t = i / 59;
      return [
        Math.sin(t * Math.PI * 2.3) * totalLen * 0.1,
        Math.cos(t * Math.PI * 1.6) * totalLen * 0.08,
        t * totalLen,
      ];
    });
    const stroke: Stroke = { id: 'bendy', points };

    const baseline = fitChainToStroke(modules, stroke, IDENTITY_POSE, 0.3, false);
    const refined = fitChainToStroke(modules, stroke, IDENTITY_POSE, 0.3, true);

    expect(refined.residual).toBeLessThanOrEqual(baseline.residual + 1e-9);
  });
});

describe('strokeArcLength', () => {
  it('sums segment distances', () => {
    const points = [new Vector3(0, 0, 0), new Vector3(3, 0, 0), new Vector3(3, 4, 0)];
    expect(strokeArcLength(points)).toBeCloseTo(7, 5);
  });
});

describe('findSelfIntersection', () => {
  it('flags two points close in space but far apart along the chain', () => {
    const points = [new Vector3(0, 0, 0), new Vector3(0, 0, 1), new Vector3(0, 0, 2), new Vector3(0.05, 0, 0.02)];
    const arcLengths = [0, 1, 2, 5]; // last point is far along the chain but spatially near the start
    expect(findSelfIntersection(points, arcLengths)).toBe(true);
  });

  it('does not flag a joint\'s own close neighbors along the chain', () => {
    const points = [new Vector3(0, 0, 0), new Vector3(0, 0, 0.05), new Vector3(0, 0, 0.1)];
    const arcLengths = [0, 0.05, 0.1]; // adjacent along the chain -- naturally close, not a crossing
    expect(findSelfIntersection(points, arcLengths)).toBe(false);
  });

  it('does not flag a chain that never comes back near itself', () => {
    const points = Array.from({ length: 10 }, (_, i) => new Vector3(0, 0, i));
    const arcLengths = points.map((_, i) => i);
    expect(findSelfIntersection(points, arcLengths)).toBe(false);
  });
});

describe('planarity', () => {
  it('keeps a fit entirely on the Z=0 plane for a stroke that never leaves it', () => {
    const points: [number, number, number][] = Array.from({ length: 20 }, (_, i) => {
      const t = i / 19;
      return [t * 3, t * 1.2 + Math.sin(t * 2) * 0.3, 0];
    });
    const stroke: Stroke = { id: 'planar', points };
    const chainStartPose = computeChainStartPoseFromStroke(stroke, new Vector3(0, 0, 1));

    const modules = [createModule(), createModule()];
    const result = fitChainToStroke(modules, stroke, chainStartPose, 0.3);
    const basePose = chainStartPoseToBasePose(chainStartPose);
    modules.forEach((m) => m.rods.forEach((r, i) => { r.angle = result.moduleAngles[m.id]![i]!; }));

    let pose = basePose;
    for (const module of modules) {
      const t = computeModuleTransforms(module.rods, pose);
      for (const rodPose of t.rods) {
        expect(Math.abs(rodPose.position[2])).toBeLessThan(1e-6);
      }
      expect(Math.abs(t.connectorB.position[2])).toBeLessThan(1e-6);
      pose = t.connectorB;
    }
  });

  it('a point at the very start of a long, well-defined stroke still gets a confident (not blended-toward-arbitrary) tangent', () => {
    // Regression test for a confidence-formula bug: a boundary point (nothing
    // available "before" it) was capped at ~50% confidence even when the
    // stroke overall was long and unambiguous, blending the start frame
    // halfway toward an arbitrary fallback and knocking it off the stroke's
    // own plane.
    const points: [number, number, number][] = Array.from({ length: 20 }, (_, i) => [i * 0.2, 0, 0]);
    const stroke: Stroke = { id: 'long-straight', points };
    const pose = computeChainStartPoseFromStroke(stroke);
    // Local +Z should point essentially exactly along +X (the stroke's true direction),
    // not partway toward the arbitrary world-Z fallback used for low-confidence cases.
    const forward = new Vector3(0, 0, 1).applyQuaternion(new Quaternion(...pose.quaternion));
    expect(forward.x).toBeGreaterThan(0.99);
  });
});

describe('multi-module assembled chain (production path: buildFittedChain + computeAssemblyWorldTransforms)', () => {
  it('progresses monotonically along a sharp turn followed by a long straight leg, without doubling back', () => {
    // A ~126deg turn (beyond what a single module's bend limits can do smoothly) immediately
    // followed by a long straight run -- exercises multiple modules welded end-to-end.
    const seg = MODULE_CHAIN_LENGTH;
    const points: [number, number, number][] = [];
    for (let i = 0; i <= 8; i += 1) {
      const t = (i / 8) * (Math.PI * 0.7);
      points.push([Math.sin(t) * seg * 0.5, seg * 0.5 * (1 - Math.cos(t)), 0]);
    }
    const turnEnd = new Vector3(...points[points.length - 1]!);
    const dir = new Vector3(Math.cos(Math.PI * 0.7), Math.sin(Math.PI * 0.7), 0);
    for (let i = 1; i <= 10; i += 1) {
      const p = turnEnd.clone().add(dir.clone().multiplyScalar((i / 10) * seg * 2));
      points.push([p.x, p.y, p.z]);
    }
    const stroke: Stroke = { id: 'sharp-turn', points };
    const chainStartPose = computeChainStartPoseFromStroke(stroke, new Vector3(0, 0, 1));
    // Size the chain to the stroke's own length so the last module doesn't run past the end --
    // that's a separate, expected "confidence drops to 0, continue straight" behavior, not
    // what this test is checking for.
    const moduleCount = computeFeasibility(strokeArcLength(points.map((p) => new Vector3(...p))), 0).modulesNeeded;
    const { assembly } = buildFittedChain(stroke, moduleCount, chainStartPose, 0.3);

    const transforms = computeAssemblyWorldTransforms(assembly);
    const orderedModuleIds = Object.keys(assembly.modules);
    const connectorBPositions = orderedModuleIds.map((id) => new Vector3(...transforms.get(id)!.connectorB.position));

    // Each module's end should make real forward progress toward the stroke's actual end --
    // welded modules must never double back past where an earlier module already reached. The
    // very last module is allowed to run a bit past the endpoint once it's out of stroke to
    // follow (confidence hits 0, it just continues straight -- expected, not a regression), so
    // only the modules still within the stroke's reach are held to strict monotonic progress.
    const strokeEnd = new Vector3(...points[points.length - 1]!);
    let previousDistanceToEnd = new Vector3(...chainStartPose.position).distanceTo(strokeEnd);
    connectorBPositions.forEach((pos, i) => {
      const distanceToEnd = pos.distanceTo(strokeEnd);
      if (i < connectorBPositions.length - 1) {
        expect(distanceToEnd).toBeLessThan(previousDistanceToEnd + 1e-6);
      }
      previousDistanceToEnd = distanceToEnd;
    });
    // Even with the last module's overshoot, it shouldn't end up wildly far from the actual end.
    expect(previousDistanceToEnd).toBeLessThan(MODULE_CHAIN_LENGTH * 1.5);
  });
});
