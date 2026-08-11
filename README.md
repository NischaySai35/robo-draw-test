# MODULINK

Primitive procedural modular robot designer.

- Phase 1: core module engine + manual-control GUI.
- Phase 2: draw-to-build mode (sketch a curve, fit a module chain to it).
- Phase 3: cube-builder mode (spawn a voxel graph, convert it to module chains).

See `src/` for the full breakdown:

- `src/types/module.ts`, `src/types/draw.ts`, `src/types/voxel.ts` — core data models
- `src/kinematics/` — pure forward-kinematics, connector-graph, curve-fit, and voxel-graph logic (no Three.js-rendering/React coupling, unit-tested)
- `src/geometry/` — procedural Three.js primitives for one module
- `src/scene/` — mount-once viewports (`ModelEditor`, `DrawModeEditor`, `VoxelModeEditor`) + their imperative controllers
- `src/state/` — Zustand stores: `assemblyStore` (structural undo/redo), `selectionStore`, `drawStore`, `voxelStore`, `uiStore`
- `src/ui/` — React dock panels (Toolbar, Outliner, Inspector, draw settings/status, voxel settings/inspector, toasts, choice dialog, context menu)

## Run it

```
npm install
npm run dev
```

Use the **Manual Edit / Draw to Build / Cube Builder** tabs at the top to switch modes:

- **Manual Edit**: left dock is the module outliner, center is the 3D
  viewport, right dock is the properties inspector.
- **Draw to Build**: left dock is sketch settings, center is the draw
  viewport (left-drag draws, right-drag orbits, scroll extrudes/adjusts
  sketch depth), right dock is the fit/feasibility status panel with
  Re-fit/Apply per stroke.
- **Cube Builder**: left dock is instructions + undo/redo/clear, center is
  the voxel viewport (click/drag a face's ghost hotspot to spawn adjacent
  cubes, click a cube body to select it, right-drag orbits), right dock is
  the selected cube's move/delete controls plus a "Convert to modules"
  summary with a single Apply.

## Other commands

```
npm run typecheck   # tsc -b --noEmit, strict mode
npm test            # vitest — kinematics/graph unit tests
npm run build       # production build
```
