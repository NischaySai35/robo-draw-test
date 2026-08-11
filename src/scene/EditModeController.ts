/**
 * Imperative Three.js scene controller for Phase 1's manual-control edit mode.
 *
 * Mirrors the `ModelEditor` + `EditModeController` split used by the sibling
 * TETROBOT project: `ModelEditor` (see scene/ModelEditor.tsx) mounts once and
 * owns the DOM/canvas lifecycle, while this class owns the live Three.js
 * scene graph, picking, gizmo, and snap-preview logic. It is NOT re-created
 * on React re-renders -- it subscribes to the Zustand stores directly and
 * patches the scene imperatively, since hot-patching a live Three.js scene
 * from React re-renders is exactly what we want to avoid.
 */
import {
  AmbientLight,
  Color,
  DirectionalLight,
  Group,
  GridHelper,
  Mesh,
  PerspectiveCamera,
  Raycaster,
  Scene,
  Vector2,
  WebGLRenderer,
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import type { Assembly, Connector, ConnectorId, ModuleId, Pose } from '../types/module';
import { allConnectors } from '../types/module';
import { anchorModuleId, computeAssemblyWorldTransforms, connectorPose, findConnector } from '../kinematics/assemblyGraph';
import { composePoses, invertPose, matrixToPose } from '../kinematics/frame';
import { findSnapCandidates, type SnapCandidate } from '../kinematics/snap';
import { solveConstrained } from '../kinematics/loopClosure';
import { createModuleObject3D, disposeModuleObject3D, updateModuleObject3D, type ModuleObject3D } from '../geometry/moduleObject3D';
import { materials } from '../geometry/materials';
import { useAssemblyStore } from '../state/assemblyStore';
import { useSelectionStore } from '../state/selectionStore';
import { useUIStore } from '../state/uiStore';

interface DragContext {
  moduleId: ModuleId;
  anchorId: ModuleId;
  /** Rigid offset from the anchor module's connector-A frame to the dragged module's, captured at drag start. */
  rigidOffset: Pose;
}

export class EditModeController {
  private readonly scene = new Scene();
  private readonly camera: PerspectiveCamera;
  private readonly renderer: WebGLRenderer;
  private readonly orbitControls: OrbitControls;
  private readonly transformControls: TransformControls;
  private readonly raycaster = new Raycaster();
  private readonly pointerNdc = new Vector2();

  private readonly moduleObjects = new Map<ModuleId, ModuleObject3D>();
  /** Invisible proxy the gizmo actually manipulates -- see drag math in `onGizmoChange`. */
  private readonly gizmoHandle = new Group();
  private dragContext: DragContext | null = null;
  /** True between the gizmo's drag-start and drag-end, for either drag mode. */
  private gizmoDragging = false;
  private pendingSnap: SnapCandidate | null = null;
  private lastTransforms = computeAssemblyWorldTransforms({ modules: {}, edges: [] });
  private disposed = false;
  private resizeObserver: ResizeObserver | null = null;
  private unsubscribes: Array<() => void> = [];
  private readonly container: HTMLElement;

  constructor(container: HTMLElement) {
    this.container = container;
    this.camera = new PerspectiveCamera(50, 1, 0.05, 500);
    this.camera.position.set(4, 3.2, 6);

    this.renderer = new WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(this.renderer.domElement);

    this.scene.background = new Color(0x1b1f27);
    // Ambient + a single key light left large parts of correctly-shaped geometry (e.g. the far
    // side of a hemisphere dome) in near-total shadow from many camera angles, blending into the
    // dark background and reading as broken/missing geometry rather than just unlit. A dimmer
    // fill light from roughly the opposite direction, plus a bit more ambient, keeps every part
    // of the model visibly lit no matter which way the orbit camera is facing.
    this.scene.add(new AmbientLight(0xffffff, 0.85));
    const key = new DirectionalLight(0xffffff, 1.0);
    key.position.set(5, 8, 4);
    this.scene.add(key);
    const fill = new DirectionalLight(0xaec6ff, 0.45);
    fill.position.set(-6, -3, -5);
    this.scene.add(fill);
    const grid = new GridHelper(20, 20, 0x3a4150, 0x2a2f3a);
    this.scene.add(grid);

    this.orbitControls = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbitControls.enableDamping = true;
    this.orbitControls.target.set(0, 0.5, 0);

    this.transformControls = new TransformControls(this.camera, this.renderer.domElement);
    this.transformControls.setMode('translate');
    this.transformControls.addEventListener('dragging-changed', (event) => {
      this.orbitControls.enabled = !event.value;
      this.gizmoDragging = Boolean(event.value);
      if (!event.value) this.onDragEnd();
    });
    this.transformControls.addEventListener('objectChange', () => this.onGizmoChange());
    this.scene.add(this.transformControls.getHelper());
    this.gizmoHandle.visible = false;
    this.scene.add(this.gizmoHandle);

    this.renderer.domElement.addEventListener('pointerdown', this.onPointerDown);
    window.addEventListener('keydown', this.onGizmoModeKey);

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();

    this.unsubscribes.push(
      useAssemblyStore.subscribe((state) => this.syncAssembly(state.assembly)),
      useSelectionStore.subscribe(() => this.syncSelectionHighlight()),
    );
    this.syncAssembly(useAssemblyStore.getState().assembly);
    this.tick();
  }

  private resize() {
    const { clientWidth, clientHeight } = this.container;
    if (clientWidth === 0 || clientHeight === 0) return;
    this.camera.aspect = clientWidth / clientHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(clientWidth, clientHeight);
  }

  private tick = () => {
    if (this.disposed) return;
    this.orbitControls.update();
    this.renderer.render(this.scene, this.camera);
    requestAnimationFrame(this.tick);
  };

  // ---------------------------------------------------------------------
  // Assembly sync: create/update/remove module Object3Ds from store state
  // ---------------------------------------------------------------------

  private syncAssembly(assembly: Assembly) {
    const transforms = computeAssemblyWorldTransforms(assembly);
    this.lastTransforms = transforms;

    for (const id of Array.from(this.moduleObjects.keys())) {
      if (!assembly.modules[id]) {
        const refs = this.moduleObjects.get(id);
        if (refs) disposeModuleObject3D(refs);
        this.moduleObjects.delete(id);
      }
    }

    for (const module of Object.values(assembly.modules)) {
      let refs = this.moduleObjects.get(module.id);
      if (!refs) {
        refs = createModuleObject3D(module);
        this.scene.add(refs.group);
        this.moduleObjects.set(module.id, refs);
      }
      const moduleTransforms = transforms.get(module.id);
      if (moduleTransforms) updateModuleObject3D(refs, module, moduleTransforms);
    }

    this.syncSelectionHighlight();
    this.syncGizmoTarget();
  }

  // ---------------------------------------------------------------------
  // Selection highlighting (shared materials mean we can't tint per-instance,
  // so selected modules get emissive outline meshes layered on their parts).
  // ---------------------------------------------------------------------

  private syncSelectionHighlight() {
    const { selectedModuleIds } = useSelectionStore.getState();
    for (const [id, refs] of this.moduleObjects) {
      const selected = selectedModuleIds.includes(id);
      Object.values(refs.connectors).forEach((c) => this.setOutline(c.mesh, selected));
      refs.rods.forEach((r) => this.setOutline(r.mesh, selected));
    }
  }

  private setOutline(mesh: Mesh, on: boolean) {
    mesh.userData['outlineOn'] = on;
    // Cheap approach: nudge emissive via renderOrder-based outline is overkill for
    // Phase 1; we instead scale a translucent backside clone in `updateOutlineMesh`.
    this.updateOutlineMesh(mesh, on);
  }

  private updateOutlineMesh(mesh: Mesh, on: boolean) {
    const existing = mesh.getObjectByName('__outline') as Mesh | undefined;
    if (!on) {
      if (existing) mesh.remove(existing);
      return;
    }
    if (existing) return;
    const outline = new Mesh(mesh.geometry, materials.selectionOutline);
    outline.name = '__outline';
    outline.scale.setScalar(1.08);
    mesh.add(outline);
  }

  // ---------------------------------------------------------------------
  // Gizmo: moves the whole rigid connected-component the selected module
  // belongs to (locked sub-chains always move together -- see module docs).
  // ---------------------------------------------------------------------

  /**
   * Places the gizmo handle. A selected CONNECTOR puts it on that connector and
   * switches the drag to inverse kinematics (see `onGizmoChange`); otherwise it
   * sits on the module's connector A and drags the whole rigid component.
   *
   * Skipped entirely mid-drag: applying solved angles re-runs `syncAssembly`,
   * which lands back here, and re-seating the handle on the connector's new
   * pose would fight the pointer.
   */
  private syncGizmoTarget() {
    if (this.gizmoDragging) return;

    const { selectedModuleIds, selectedConnector } = useSelectionStore.getState();
    if (selectedConnector) {
      const connector = findConnector(useAssemblyStore.getState().assembly, selectedConnector);
      const transforms = connector && this.lastTransforms.get(connector.moduleId);
      if (connector && transforms) {
        const pose = connectorPose(transforms, connector.end);
        this.gizmoHandle.position.set(...pose.position);
        this.gizmoHandle.quaternion.set(...pose.quaternion);
        this.gizmoHandle.visible = true;
        if (this.transformControls.object !== this.gizmoHandle) {
          this.transformControls.attach(this.gizmoHandle);
        }
        return;
      }
    }

    const moduleId = selectedModuleIds[0];
    if (!moduleId || this.dragContext) return;
    const transforms = this.lastTransforms.get(moduleId);
    if (!transforms) {
      this.transformControls.detach();
      this.gizmoHandle.visible = false;
      return;
    }
    this.gizmoHandle.position.set(...transforms.connectorA.position);
    this.gizmoHandle.quaternion.set(...transforms.connectorA.quaternion);
    this.gizmoHandle.visible = true;
    if (this.transformControls.object !== this.gizmoHandle) {
      this.transformControls.attach(this.gizmoHandle);
    }
  }

  /**
   * Bends the structure so the selected connector follows the gizmo, holding
   * every weld closed and every joint in range.
   *
   * Deliberately cheap per event: no restarts and a low iteration cap, because
   * each solve is seeded from the current angles and so warm-starts off the
   * previous frame -- the drag converges across frames rather than within one.
   * Restarts would be actively wrong here; they jump to a different solution
   * branch and the structure visibly teleports mid-drag.
   */
  private dragConnectorToGizmo(connectorId: ConnectorId): void {
    const { assembly } = useAssemblyStore.getState();
    const connector = findConnector(assembly, connectorId);
    if (!connector) return;

    const { angles } = solveConstrained(
      assembly,
      [{
        moduleId: connector.moduleId,
        end: connector.end,
        target: matrixToPose(this.gizmoHandle.matrixWorld),
        // A translate gizmo says nothing about orientation, so demanding one
        // would only make the goal harder to reach for no benefit.
        positionOnly: this.transformControls.getMode() === 'translate',
      }],
      { restarts: 0, maxIterations: 12 },
    );
    useAssemblyStore.getState().setSolvedAngles(angles);
  }

  private onGizmoChange() {
    const { selectedModuleIds, selectedConnector } = useSelectionStore.getState();
    if (selectedConnector) {
      this.dragConnectorToGizmo(selectedConnector);
      return;
    }

    const moduleId = selectedModuleIds[0];
    if (!moduleId) return;
    const { assembly } = useAssemblyStore.getState();

    if (!this.dragContext) {
      const anchorId = anchorModuleId(assembly, moduleId);
      const anchorPose = this.lastTransforms.get(anchorId)?.connectorA;
      const selectedPose = this.lastTransforms.get(moduleId)?.connectorA;
      if (!anchorPose || !selectedPose) return;
      const rigidOffset = composePoses(invertPose(anchorPose), selectedPose);
      this.dragContext = { moduleId, anchorId, rigidOffset };
    }

    const handleWorldPose = matrixToPose(this.gizmoHandle.matrixWorld);
    const anchorPoseNew = composePoses(handleWorldPose, invertPose(this.dragContext.rigidOffset));
    useAssemblyStore.getState().setModuleBasePose(this.dragContext.anchorId, anchorPoseNew);
    this.updateSnapPreview();
  }

  private onDragEnd() {
    if (this.pendingSnap) {
      useAssemblyStore
        .getState()
        .connectConnectors(this.pendingSnap.connectorA.id, this.pendingSnap.connectorB.id);
      useUIStore.getState().pushWarning('Connectors locked.', 'warning');
      this.clearSnapPreview();
    }
    this.dragContext = null;
  }

  // ---------------------------------------------------------------------
  // Snap preview: while dragging, look for nearby facing open connectors
  // belonging to a *different* connected component and ghost-highlight them.
  // ---------------------------------------------------------------------

  private updateSnapPreview() {
    if (!this.dragContext) return;
    const { assembly } = useAssemblyStore.getState();
    const draggedAnchor = this.dragContext.anchorId;
    const open: Array<{ connector: Connector; pose: Pose }> = [];

    for (const module of Object.values(assembly.modules)) {
      const anchorForModule = anchorModuleId(assembly, module.id);
      if (anchorForModule === draggedAnchor) continue; // skip the module's own rigid component
      const transforms = this.lastTransforms.get(module.id);
      if (!transforms) continue;
      for (const connector of allConnectors(module)) {
        if (!connector.locked) open.push({ connector, pose: connectorPose(transforms, connector.end) });
      }
    }
    // Also include the dragged module's own open connectors as candidates.
    const draggedModule = assembly.modules[this.dragContext.moduleId];
    const draggedTransforms = this.lastTransforms.get(this.dragContext.moduleId);
    if (draggedModule && draggedTransforms) {
      for (const connector of allConnectors(draggedModule)) {
        if (!connector.locked) {
          open.push({ connector, pose: connectorPose(draggedTransforms, connector.end) });
        }
      }
    }

    const candidates = findSnapCandidates(open).filter(
      (c) => c.connectorA.moduleId === this.dragContext!.moduleId || c.connectorB.moduleId === this.dragContext!.moduleId,
    );

    this.clearSnapPreview();
    const best = candidates[0];
    if (best) {
      this.pendingSnap = best;
      this.ghostConnector(best.connectorA.id, true);
      this.ghostConnector(best.connectorB.id, true);
    }
  }

  private clearSnapPreview() {
    if (this.pendingSnap) {
      this.ghostConnector(this.pendingSnap.connectorA.id, false);
      this.ghostConnector(this.pendingSnap.connectorB.id, false);
    }
    this.pendingSnap = null;
  }

  /**
   * Ghost-tints a connector's hemisphere to preview a pending snap. The mesh's
   * material right before ghosting is stashed in `userData` and restored
   * exactly when the ghost is cleared, since the "real" material otherwise
   * depends on live locked state and shouldn't be guessed at restore time.
   */
  private ghostConnector(connectorId: ConnectorId, on: boolean) {
    const mesh = this.findConnectorMesh(connectorId);
    if (!mesh) return;
    if (on) {
      mesh.userData['preGhostMaterial'] = mesh.material;
      mesh.material = materials.ghostSnap;
    } else {
      const prev = mesh.userData['preGhostMaterial'] as Mesh['material'] | undefined;
      if (prev) mesh.material = prev;
      delete mesh.userData['preGhostMaterial'];
    }
  }

  private findConnectorMesh(connectorId: ConnectorId): Mesh | undefined {
    const { assembly } = useAssemblyStore.getState();
    const connector = findConnector(assembly, connectorId);
    if (!connector) return undefined;
    const refs = this.moduleObjects.get(connector.moduleId);
    if (!refs) return undefined;
    return refs.connectors[connector.end].mesh;
  }

  /** G = translate, R = rotate -- standard DCC-tool shortcuts for the gizmo mode. */
  private onGizmoModeKey = (event: KeyboardEvent) => {
    if (event.target instanceof HTMLElement && ['INPUT', 'SELECT', 'TEXTAREA'].includes(event.target.tagName)) return;
    if (event.key.toLowerCase() === 'g') this.transformControls.setMode('translate');
    if (event.key.toLowerCase() === 'r') this.transformControls.setMode('rotate');
  };

  // ---------------------------------------------------------------------
  // Picking
  // ---------------------------------------------------------------------

  private onPointerDown = (event: PointerEvent) => {
    if (event.target !== this.renderer.domElement) return;
    if (this.transformControls.dragging) return;
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointerNdc.set(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    this.raycaster.setFromCamera(this.pointerNdc, this.camera);

    const pickable: Mesh[] = [];
    for (const refs of this.moduleObjects.values()) {
      pickable.push(
        ...Object.values(refs.connectors).map((c) => c.mesh),
        ...refs.rods.map((r) => r.mesh),
      );
    }
    const hits = this.raycaster.intersectObjects(pickable, false);
    const additive = event.shiftKey;
    const selection = useSelectionStore.getState();

    if (hits.length === 0) {
      if (!additive) selection.clearSelection();
      return;
    }

    const hitMesh = hits[0]!.object as Mesh;
    const moduleEntry = Array.from(this.moduleObjects.entries()).find(
      ([, refs]) =>
        Object.values(refs.connectors).some((c) => c.mesh === hitMesh) ||
        refs.rods.some((r) => r.mesh === hitMesh),
    );
    if (!moduleEntry) return;
    const [moduleId, refs] = moduleEntry;

    const hitConnector = Object.values(refs.connectors).find((c) => c.mesh === hitMesh);
    if (hitConnector) {
      const { assembly } = useAssemblyStore.getState();
      const module = assembly.modules[moduleId];
      const connector = module && allConnectors(module).find((c) => c.end === hitConnector.end);
      if (connector) selection.selectConnector(connector.id);
      selection.selectModule(moduleId, additive);
      return;
    }

    const rodIndex = refs.rods.findIndex((r) => r.mesh === hitMesh);
    if (rodIndex >= 0) selection.selectRod(moduleId, rodIndex);
  };

  dispose() {
    this.disposed = true;
    this.resizeObserver?.disconnect();
    this.unsubscribes.forEach((fn) => fn());
    this.renderer.domElement.removeEventListener('pointerdown', this.onPointerDown);
    window.removeEventListener('keydown', this.onGizmoModeKey);
    this.transformControls.dispose();
    this.orbitControls.dispose();
    this.renderer.dispose();
    this.container.removeChild(this.renderer.domElement);
  }
}
