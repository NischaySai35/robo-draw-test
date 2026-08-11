/**
 * Imperative Three.js scene controller for Phase 2's draw-to-build mode.
 * Same mount-once split as `EditModeController`: this class owns the live
 * scene, stroke capture, and preview rendering; `DrawModeEditor.tsx` only
 * mounts it once and hands off the container element.
 *
 * Input scheme: left-drag draws a stroke, right-drag orbits (OrbitControls'
 * left button is intentionally left unbound), matching common sketch-tool
 * conventions and avoiding a left-drag conflict between "orbit" and "draw".
 */
import {
  AmbientLight,
  BufferGeometry,
  Color,
  DirectionalLight,
  DoubleSide,
  GridHelper,
  Group,
  Line,
  LineBasicMaterial,
  Mesh,
  MeshBasicMaterial,
  MOUSE,
  PerspectiveCamera,
  Plane,
  PlaneGeometry,
  Raycaster,
  Scene,
  Vector2,
  Vector3,
  WebGLRenderer,
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { Stroke } from '../types/draw';
import { computeAssemblyWorldTransforms } from '../kinematics/assemblyGraph';
import { BIG_ROD_INDEX } from '../constants/geometry';
import { geometries, rodGeometryFor } from '../geometry/primitives';
import { materials } from '../geometry/materials';
import { useDrawStore } from '../state/drawStore';
import { computeDrawPreview, persistFitForStroke, type DrawPreview } from '../ui/drawActions';

/** Minimum world-space distance between consecutive captured points -- keeps strokes light. */
const MIN_POINT_SPACING = 0.06;
const STROKE_COLOR = 0xffb703;

export class DrawModeController {
  private readonly scene = new Scene();
  private readonly camera: PerspectiveCamera;
  private readonly renderer: WebGLRenderer;
  private readonly orbitControls: OrbitControls;
  private readonly raycaster = new Raycaster();
  private readonly pointerNdc = new Vector2();

  private readonly planeMesh: Mesh;
  private readonly strokeLines = new Map<string, Line>();
  private readonly previewGroups = new Map<string, Group>();

  private isPointerDrawing = false;
  private sketchDepth = 4;
  private pendingExtrudeStrokeId: string | null = null;
  private disposed = false;
  private resizeObserver: ResizeObserver | null = null;
  private readonly unsubscribes: Array<() => void> = [];
  private readonly container: HTMLElement;

  constructor(container: HTMLElement) {
    this.container = container;
    this.camera = new PerspectiveCamera(50, 1, 0.05, 500);
    this.camera.position.set(3, 3, 6);

    this.renderer = new WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(this.renderer.domElement);

    this.scene.background = new Color(0x181c24);
    // See EditModeController for why there's a fill light too -- a single key light left
    // correctly-shaped geometry looking broken from many angles, purely because its unlit side
    // blended into the dark background.
    this.scene.add(new AmbientLight(0xffffff, 0.85));
    const key = new DirectionalLight(0xffffff, 1.0);
    key.position.set(5, 8, 4);
    this.scene.add(key);
    const fill = new DirectionalLight(0xaec6ff, 0.45);
    fill.position.set(-6, -3, -5);
    this.scene.add(fill);
    this.scene.add(new GridHelper(20, 20, 0x3a4150, 0x272c38));

    this.planeMesh = new Mesh(
      new PlaneGeometry(12, 12),
      new MeshBasicMaterial({ color: 0x2c3a55, transparent: true, opacity: 0.12, side: DoubleSide }),
    );
    this.scene.add(this.planeMesh);

    this.orbitControls = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbitControls.enableDamping = true;
    this.orbitControls.target.set(0, 0.5, 0);
    // Left button intentionally unbound -- it's reserved for drawing strokes.
    this.orbitControls.mouseButtons = { LEFT: null, MIDDLE: MOUSE.DOLLY, RIGHT: MOUSE.ROTATE };

    const dom = this.renderer.domElement;
    dom.addEventListener('pointerdown', this.onPointerDown);
    dom.addEventListener('pointermove', this.onPointerMove);
    window.addEventListener('pointerup', this.onPointerUp);
    dom.addEventListener('wheel', this.onWheel, { passive: false });
    dom.addEventListener('contextmenu', (e) => e.preventDefault());

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();

    this.unsubscribes.push(
      useDrawStore.subscribe(() => this.rebuildAll()),
    );
    this.rebuildAll();
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
  // Draw plane / ray picking
  // ---------------------------------------------------------------------

  private currentPlane(): Plane {
    const { plane } = useDrawStore.getState().settings;
    if (plane === 'XZ') return new Plane(new Vector3(0, 1, 0), 0);
    if (plane === 'YZ') return new Plane(new Vector3(1, 0, 0), 0);
    return new Plane(new Vector3(0, 0, 1), 0); // XY
  }

  private updatePlaneVisual() {
    const { dimensionality, plane } = useDrawStore.getState().settings;
    this.planeMesh.visible = dimensionality === '2d';
    this.planeMesh.rotation.set(0, 0, 0);
    if (plane === 'XZ') this.planeMesh.rotation.x = Math.PI / 2;
    else if (plane === 'YZ') this.planeMesh.rotation.y = Math.PI / 2;
  }

  private pickWorldPoint(event: PointerEvent): Vector3 | null {
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointerNdc.set(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    this.raycaster.setFromCamera(this.pointerNdc, this.camera);

    const { dimensionality } = useDrawStore.getState().settings;
    const target = new Vector3();
    if (dimensionality === '2d') {
      const hit = this.raycaster.ray.intersectPlane(this.currentPlane(), target);
      return hit ? target : null;
    }
    // 3D freehand: intersect a camera-facing plane at `sketchDepth` -- scroll while
    // drawing pushes that plane deeper/shallower, approximating "drag through space".
    const camForward = new Vector3();
    this.camera.getWorldDirection(camForward);
    const planePoint = this.camera.position.clone().add(camForward.clone().multiplyScalar(this.sketchDepth));
    const billboard = new Plane().setFromNormalAndCoplanarPoint(camForward, planePoint);
    const hit = this.raycaster.ray.intersectPlane(billboard, target);
    return hit ? target : null;
  }

  // ---------------------------------------------------------------------
  // Pointer / wheel handlers
  // ---------------------------------------------------------------------

  private onPointerDown = (event: PointerEvent) => {
    if (event.button !== 0) return; // only the left button draws
    if (event.target !== this.renderer.domElement) return;
    const point = this.pickWorldPoint(event);
    if (!point) return;

    this.pendingExtrudeStrokeId = null;
    this.isPointerDrawing = true;
    this.orbitControls.enabled = false;
    useDrawStore.getState().startStroke();
    useDrawStore.getState().appendPoint([point.x, point.y, point.z]);
  };

  private onPointerMove = (event: PointerEvent) => {
    if (!this.isPointerDrawing) return;
    const point = this.pickWorldPoint(event);
    if (!point) return;
    const { strokes, activeStrokeId } = useDrawStore.getState();
    const active = strokes.find((s) => s.id === activeStrokeId);
    const last = active?.points[active.points.length - 1];
    if (last && point.distanceTo(new Vector3(...last)) < MIN_POINT_SPACING) return;
    useDrawStore.getState().appendPoint([point.x, point.y, point.z]);
  };

  private onPointerUp = () => {
    if (!this.isPointerDrawing) return;
    this.isPointerDrawing = false;
    this.orbitControls.enabled = true;

    const { strokes, activeStrokeId, settings, endStroke } = useDrawStore.getState();
    const stroke = strokes.find((s) => s.id === activeStrokeId);
    endStroke();
    if (!stroke || stroke.points.length < 2) return;

    persistFitForStroke(stroke);
    if (settings.dimensionality === '2d' && settings.extrude) {
      this.pendingExtrudeStrokeId = stroke.id;
    }
  };

  private onWheel = (event: WheelEvent) => {
    if (this.pendingExtrudeStrokeId) {
      event.preventDefault();
      const normal = this.currentPlane().normal;
      const delta = normal.clone().multiplyScalar(-event.deltaY * 0.003);
      const strokeId = this.pendingExtrudeStrokeId;
      useDrawStore.getState().offsetStrokePoints(strokeId, [delta.x, delta.y, delta.z]);
      const stroke = useDrawStore.getState().strokes.find((s) => s.id === strokeId);
      if (stroke) persistFitForStroke(stroke);
      return;
    }
    if (this.isPointerDrawing && useDrawStore.getState().settings.dimensionality === '3d') {
      event.preventDefault();
      this.sketchDepth = Math.min(20, Math.max(0.5, this.sketchDepth - event.deltaY * 0.01));
    }
  };

  // ---------------------------------------------------------------------
  // Rendering: raw strokes + fitted preview chains
  // ---------------------------------------------------------------------

  private rebuildAll() {
    this.updatePlaneVisual();
    const { strokes } = useDrawStore.getState();
    const liveIds = new Set(strokes.map((s) => s.id));

    for (const [id, line] of this.strokeLines) {
      if (!liveIds.has(id)) {
        this.scene.remove(line);
        line.geometry.dispose();
        this.strokeLines.delete(id);
      }
    }
    for (const [id, group] of this.previewGroups) {
      if (!liveIds.has(id)) {
        this.scene.remove(group);
        this.previewGroups.delete(id);
      }
    }

    for (const stroke of strokes) {
      this.updateStrokeLine(stroke);
      this.updatePreview(stroke);
    }
  }

  private updateStrokeLine(stroke: Stroke) {
    let line = this.strokeLines.get(stroke.id);
    const points = stroke.points.map((p) => new Vector3(...p));
    if (points.length < 2) {
      if (line) {
        this.scene.remove(line);
        line.geometry.dispose();
        this.strokeLines.delete(stroke.id);
      }
      return;
    }
    const geometry = new BufferGeometry().setFromPoints(points);
    if (!line) {
      line = new Line(geometry, new LineBasicMaterial({ color: STROKE_COLOR, linewidth: 2 }));
      this.scene.add(line);
      this.strokeLines.set(stroke.id, line);
    } else {
      line.geometry.dispose();
      line.geometry = geometry;
    }
  }

  private updatePreview(stroke: Stroke) {
    // Full coordinate-descent refinement (see fitChainToStroke) is too slow to run on every
    // single pointermove of an active drag -- use the fast pursuit-only baseline for the live
    // preview while still dragging, and the full-quality fit once the stroke has settled.
    const preview: DrawPreview | null = computeDrawPreview(stroke, undefined, !this.isPointerDrawing);
    let group = this.previewGroups.get(stroke.id);
    if (!preview) {
      if (group) group.visible = false;
      return;
    }
    if (!group) {
      group = new Group();
      this.scene.add(group);
      this.previewGroups.set(stroke.id, group);
    }
    group.visible = true;
    group.clear();

    const transforms = computeAssemblyWorldTransforms(preview.assembly);
    for (const module of Object.values(preview.assembly.modules)) {
      const t = transforms.get(module.id);
      if (!t) continue;
      // All 6 connectors -- the two chain ends plus the 4 riding the big rod.
      for (const pose of [t.connectorA, t.connectorB, ...t.sides]) {
        const hemi = new Mesh(geometries.hemisphere, materials.drawPreview);
        hemi.position.set(...pose.position);
        hemi.quaternion.set(...pose.quaternion);
        group.add(hemi);
      }

      module.rods.forEach((rod, i) => {
        const pose = t.rods[i];
        if (!pose) return;
        // Must match moduleObject3D.ts's geometry choice exactly -- the FK positions below
        // were computed using each rod's REAL physical length (rodLength() already special-cases
        // the oversized spine rod), so rendering it with the small geometry here would leave a
        // visible gap between this mesh and the next segment's actual position.
        const rodGeometry = rodGeometryFor(rod.kind, i === BIG_ROD_INDEX);
        const mesh = new Mesh(rodGeometry, materials.drawPreview);
        mesh.position.set(...pose.position);
        mesh.quaternion.set(...pose.quaternion);
        group!.add(mesh);
      });
    }
  }

  dispose() {
    this.disposed = true;
    this.resizeObserver?.disconnect();
    this.unsubscribes.forEach((fn) => fn());
    const dom = this.renderer.domElement;
    dom.removeEventListener('pointerdown', this.onPointerDown);
    dom.removeEventListener('pointermove', this.onPointerMove);
    window.removeEventListener('pointerup', this.onPointerUp);
    dom.removeEventListener('wheel', this.onWheel);
    this.orbitControls.dispose();
    this.renderer.dispose();
    this.container.removeChild(this.renderer.domElement);
  }
}
