/**
 * Imperative Three.js scene controller for Phase 3's cube-builder mode.
 * Same mount-once split as the other two modes' controllers.
 *
 * Input scheme: left-click a cube body selects it; left-click (or
 * left-drag) an open face's hotspot spawns one or more adjacent cubes in
 * that direction; right-drag orbits. Orbit's own left-button rotate is only
 * suppressed for the duration of a hotspot drag (see `onPointerDown`), so a
 * left-drag anywhere else still orbits normally.
 */
import {
  AmbientLight,
  BoxGeometry,
  Color,
  DirectionalLight,
  DoubleSide,
  GridHelper,
  Group,
  Mesh,
  MeshStandardMaterial,
  PerspectiveCamera,
  Raycaster,
  Scene,
  Vector2,
  WebGLRenderer,
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { FaceDirection, VoxelCoord, VoxelId } from '../types/voxel';
import { FACE_DIRECTIONS, FACE_NORMALS } from '../types/voxel';
import { isOccupied, neighborCoord, voxelWorldCenter } from '../kinematics/voxelGraph';
import { useVoxelStore } from '../state/voxelStore';

const PIXELS_PER_CUBE = 70;
const MAX_DRAG_CUBES = 8;

// side: DoubleSide throughout -- orbiting/zooming inside a cube shouldn't make it vanish.
const materials = {
  cube: new MeshStandardMaterial({ color: 0x4a5568, roughness: 0.6, metalness: 0.05, transparent: true, opacity: 0.85, side: DoubleSide }),
  cubeSelected: new MeshStandardMaterial({ color: 0xffb703, roughness: 0.5, metalness: 0.1, transparent: true, opacity: 0.9, side: DoubleSide }),
  hotspot: new MeshStandardMaterial({ color: 0x8894a3, roughness: 0.8, transparent: true, opacity: 0.18, side: DoubleSide }),
  hotspotHover: new MeshStandardMaterial({ color: 0x33d17a, roughness: 0.4, emissive: 0x33d17a, emissiveIntensity: 0.5, transparent: true, opacity: 0.55, side: DoubleSide }),
  ghost: new MeshStandardMaterial({ color: 0x33d17a, roughness: 0.4, transparent: true, opacity: 0.35, depthWrite: false, side: DoubleSide }),
};

interface HotspotEntry {
  voxelId: VoxelId;
  coord: VoxelCoord;
  direction: FaceDirection;
}

interface DragState {
  voxelId: VoxelId;
  coord: VoxelCoord;
  direction: FaceDirection;
  screenStart: Vector2;
}

export class VoxelModeController {
  private readonly scene = new Scene();
  private readonly camera: PerspectiveCamera;
  private readonly renderer: WebGLRenderer;
  private readonly orbitControls: OrbitControls;
  private readonly raycaster = new Raycaster();
  private readonly pointerNdc = new Vector2();

  private readonly cubeMeshes = new Map<VoxelId, Mesh>();
  private readonly hotspotMeshes = new Map<Mesh, HotspotEntry>();
  private readonly hotspotGroup = new Group();
  private readonly ghostGroup = new Group();

  private hoveredHotspot: Mesh | null = null;
  private drag: DragState | null = null;
  private disposed = false;
  private resizeObserver: ResizeObserver | null = null;
  private readonly unsubscribes: Array<() => void> = [];
  private readonly container: HTMLElement;

  constructor(container: HTMLElement) {
    this.container = container;
    this.camera = new PerspectiveCamera(50, 1, 0.05, 500);
    this.camera.position.set(5, 4, 7);

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
    this.scene.add(new GridHelper(24, 24, 0x3a4150, 0x272c38));
    this.scene.add(this.hotspotGroup, this.ghostGroup);

    this.orbitControls = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbitControls.enableDamping = true;
    this.orbitControls.target.set(0, 0.5, 0);

    const dom = this.renderer.domElement;
    dom.addEventListener('pointerdown', this.onPointerDown);
    dom.addEventListener('pointermove', this.onPointerMove);
    window.addEventListener('pointerup', this.onPointerUp);

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();

    this.unsubscribes.push(useVoxelStore.subscribe(() => this.rebuildAll()));
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
  // Rendering: cube bodies + open-face hotspots
  // ---------------------------------------------------------------------

  private rebuildAll() {
    const { graph, selectedVoxelId } = useVoxelStore.getState();
    const liveIds = new Set(Object.keys(graph.voxels));

    for (const [id, mesh] of this.cubeMeshes) {
      if (!liveIds.has(id)) {
        this.scene.remove(mesh);
        this.cubeMeshes.delete(id);
      }
    }

    for (const voxel of Object.values(graph.voxels)) {
      let mesh = this.cubeMeshes.get(voxel.id);
      if (!mesh) {
        mesh = new Mesh(new BoxGeometry(graph.cellSize * 0.86, graph.cellSize * 0.86, graph.cellSize * 0.86), materials.cube);
        mesh.userData['voxelId'] = voxel.id;
        this.scene.add(mesh);
        this.cubeMeshes.set(voxel.id, mesh);
      }
      mesh.position.set(...voxelWorldCenter(voxel.coord, graph.cellSize));
      mesh.material = voxel.id === selectedVoxelId ? materials.cubeSelected : materials.cube;
    }

    this.rebuildHotspots(graph);
  }

  private rebuildHotspots(graph: ReturnType<typeof useVoxelStore.getState>['graph']) {
    this.hotspotGroup.clear();
    this.hotspotMeshes.clear();

    for (const voxel of Object.values(graph.voxels)) {
      for (const dir of FACE_DIRECTIONS) {
        const neighbor = neighborCoord(voxel.coord, dir);
        if (isOccupied(graph, neighbor)) continue; // only open faces get a hotspot

        const normal = FACE_NORMALS[dir];
        const size = graph.cellSize * 0.6;
        const thickness = graph.cellSize * 0.04;
        const geometry = new BoxGeometry(
          normal[0] !== 0 ? thickness : size,
          normal[1] !== 0 ? thickness : size,
          normal[2] !== 0 ? thickness : size,
        );
        const mesh = new Mesh(geometry, materials.hotspot);
        const center = voxelWorldCenter(voxel.coord, graph.cellSize);
        const offset = graph.cellSize * 0.46;
        mesh.position.set(center[0] + normal[0] * offset, center[1] + normal[1] * offset, center[2] + normal[2] * offset);
        this.hotspotGroup.add(mesh);
        this.hotspotMeshes.set(mesh, { voxelId: voxel.id, coord: voxel.coord, direction: dir });
      }
    }
  }

  private updateGhostPreview(state: DragState, count: number) {
    this.ghostGroup.clear();
    const { graph } = useVoxelStore.getState();
    let cursor = state.coord;
    for (let i = 0; i < count; i += 1) {
      cursor = neighborCoord(cursor, state.direction);
      if (isOccupied(graph, cursor)) break;
      const mesh = new Mesh(new BoxGeometry(graph.cellSize * 0.8, graph.cellSize * 0.8, graph.cellSize * 0.8), materials.ghost);
      mesh.position.set(...voxelWorldCenter(cursor, graph.cellSize));
      this.ghostGroup.add(mesh);
    }
  }

  // ---------------------------------------------------------------------
  // Picking / interaction
  // ---------------------------------------------------------------------

  private raycastFromEvent(event: PointerEvent) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointerNdc.set(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    this.raycaster.setFromCamera(this.pointerNdc, this.camera);
  }

  private onPointerDown = (event: PointerEvent) => {
    if (event.button !== 0) return;
    if (event.target !== this.renderer.domElement) return;
    this.raycastFromEvent(event);

    const hotspotHits = this.raycaster.intersectObjects(Array.from(this.hotspotMeshes.keys()), false);
    if (hotspotHits.length > 0) {
      const entry = this.hotspotMeshes.get(hotspotHits[0]!.object as Mesh)!;
      this.drag = { ...entry, screenStart: new Vector2(event.clientX, event.clientY) };
      this.orbitControls.enabled = false;
      this.updateGhostPreview(this.drag, 1);
      return;
    }

    const cubeHits = this.raycaster.intersectObjects(Array.from(this.cubeMeshes.values()), false);
    if (cubeHits.length > 0) {
      const voxelId = (cubeHits[0]!.object as Mesh).userData['voxelId'] as VoxelId;
      useVoxelStore.getState().selectVoxel(voxelId);
    } else {
      useVoxelStore.getState().selectVoxel(null);
    }
  };

  private onPointerMove = (event: PointerEvent) => {
    if (this.drag) {
      const dx = event.clientX - this.drag.screenStart.x;
      const dy = event.clientY - this.drag.screenStart.y;
      const pixelDistance = Math.sqrt(dx * dx + dy * dy);
      const count = Math.min(MAX_DRAG_CUBES, 1 + Math.floor(pixelDistance / PIXELS_PER_CUBE));
      this.updateGhostPreview(this.drag, count);
      return;
    }

    this.raycastFromEvent(event);
    const hits = this.raycaster.intersectObjects(Array.from(this.hotspotMeshes.keys()), false);
    const hit = (hits[0]?.object as Mesh | undefined) ?? null;
    if (hit !== this.hoveredHotspot) {
      if (this.hoveredHotspot) this.hoveredHotspot.material = materials.hotspot;
      if (hit) hit.material = materials.hotspotHover;
      this.hoveredHotspot = hit;
    }
  };

  private onPointerUp = (event: PointerEvent) => {
    if (!this.drag) return;
    const dx = event.clientX - this.drag.screenStart.x;
    const dy = event.clientY - this.drag.screenStart.y;
    const pixelDistance = Math.sqrt(dx * dx + dy * dy);
    const count = Math.min(MAX_DRAG_CUBES, 1 + Math.floor(pixelDistance / PIXELS_PER_CUBE));

    useVoxelStore.getState().addAdjacent(this.drag.coord, this.drag.direction, count);
    this.ghostGroup.clear();
    this.drag = null;
    this.orbitControls.enabled = true;
  };

  dispose() {
    this.disposed = true;
    this.resizeObserver?.disconnect();
    this.unsubscribes.forEach((fn) => fn());
    const dom = this.renderer.domElement;
    dom.removeEventListener('pointerdown', this.onPointerDown);
    dom.removeEventListener('pointermove', this.onPointerMove);
    window.removeEventListener('pointerup', this.onPointerUp);
    this.orbitControls.dispose();
    this.renderer.dispose();
    this.container.removeChild(this.renderer.domElement);
  }
}
