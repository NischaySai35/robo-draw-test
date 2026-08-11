/**
 * Viewport for the prompt-to-build tab: the model's intended skeleton, and the
 * modules that were actually fitted to it, drawn in the same space.
 *
 * Drawing both at once is the whole diagnostic. A structure that looks wrong
 * has two possible causes that a picture of the modules alone cannot separate:
 * the model designed a bad shape, or the solver fitted a good shape badly.
 * With the skeleton showing through, they are obvious -- a wonky skeleton is
 * the model's fault, and modules straying off a clean skeleton are the fitter's.
 *
 * Same mount-once contract as the other controllers: React owns the DOM node,
 * this owns the scene and subscribes to the store directly.
 */
import {
  AmbientLight,
  BufferGeometry,
  Color,
  DirectionalLight,
  Float32BufferAttribute,
  GridHelper,
  Group,
  LineBasicMaterial,
  LineSegments,
  Mesh,
  MeshBasicMaterial,
  PerspectiveCamera,
  Scene,
  SphereGeometry,
  Vector3,
  WebGLRenderer,
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { computeAssemblyWorldTransforms } from '../kinematics/assemblyGraph';
import { createModuleObject3D, disposeModuleObject3D, updateModuleObject3D, type ModuleObject3D } from '../geometry/moduleObject3D';
import { useGenerateStore } from '../state/generateStore';
import type { ShapeSpec } from '../kinematics/skeletonFit';

const JOINT_RADIUS = 0.16;

const skeletonLineMaterial = new LineBasicMaterial({ color: 0xffb703, transparent: true, opacity: 0.9 });
const jointMaterial = new MeshBasicMaterial({ color: 0xffd166 });
const jointGeometry = new SphereGeometry(JOINT_RADIUS, 12, 8);

export class GenerateModeController {
  private readonly scene = new Scene();
  private readonly camera: PerspectiveCamera;
  private readonly renderer: WebGLRenderer;
  private readonly controls: OrbitControls;
  private readonly skeletonGroup = new Group();
  private readonly moduleGroup = new Group();
  private readonly moduleObjects = new Map<string, ModuleObject3D>();
  private readonly resizeObserver: ResizeObserver;
  private readonly unsubscribe: () => void;
  private frameHandle = 0;
  private lastRevision = -1;
  /** Re-frame the camera only when a NEW structure arrives, never on a redraw. */
  private lastFramedRevision = -1;

  private readonly container: HTMLElement;

  constructor(container: HTMLElement) {
    this.container = container;
    this.renderer = new WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(this.renderer.domElement);

    this.scene.background = new Color(0x11151c);
    this.camera = new PerspectiveCamera(50, container.clientWidth / Math.max(1, container.clientHeight), 0.1, 500);
    this.camera.position.set(9, 7, 11);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;

    const grid = new GridHelper(40, 40, 0x2a3340, 0x1b222c);
    this.scene.add(grid, this.skeletonGroup, this.moduleGroup);
    this.scene.add(new AmbientLight(0xffffff, 0.75));
    const key = new DirectionalLight(0xffffff, 1.1);
    key.position.set(6, 12, 8);
    this.scene.add(key);

    this.resizeObserver = new ResizeObserver(() => this.handleResize());
    this.resizeObserver.observe(container);

    this.unsubscribe = useGenerateStore.subscribe((state) => {
      if (state.revision !== this.lastRevision) this.sync();
    });
    this.sync();
    this.animate();
  }

  private handleResize(): void {
    const width = this.container.clientWidth;
    const height = Math.max(1, this.container.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  private sync(): void {
    const state = useGenerateStore.getState();
    this.lastRevision = state.revision;
    this.drawSkeleton(state.spec);
    this.drawModules(state.showModules ? state.fit?.assembly ?? null : null);

    if (state.spec && state.revision !== this.lastFramedRevision && state.fit) {
      this.frameCamera(state.spec);
      this.lastFramedRevision = state.revision;
    }
  }

  private drawSkeleton(spec: ShapeSpec | null): void {
    for (const child of [...this.skeletonGroup.children]) {
      this.skeletonGroup.remove(child);
      if (child instanceof LineSegments) child.geometry.dispose();
    }
    if (!spec) return;

    const positions = new Map(spec.nodes.map((n) => [n.id, n.position]));
    const points: number[] = [];
    for (const edge of spec.edges) {
      const a = positions.get(edge.from);
      const b = positions.get(edge.to);
      if (!a || !b) continue;
      points.push(a[0], a[1], a[2], b[0], b[1], b[2]);
    }
    const geometry = new BufferGeometry();
    geometry.setAttribute('position', new Float32BufferAttribute(points, 3));
    this.skeletonGroup.add(new LineSegments(geometry, skeletonLineMaterial));

    for (const node of spec.nodes) {
      const joint = new Mesh(jointGeometry, jointMaterial);
      joint.position.set(node.position[0], node.position[1], node.position[2]);
      this.skeletonGroup.add(joint);
    }
  }

  private drawModules(assembly: import('../types/module').Assembly | null): void {
    const wanted = new Set(assembly ? Object.keys(assembly.modules) : []);
    for (const [id, refs] of [...this.moduleObjects]) {
      if (wanted.has(id)) continue;
      this.moduleGroup.remove(refs.group);
      disposeModuleObject3D(refs);
      this.moduleObjects.delete(id);
    }
    if (!assembly) return;

    const transforms = computeAssemblyWorldTransforms(assembly);
    for (const module of Object.values(assembly.modules)) {
      let refs = this.moduleObjects.get(module.id);
      if (!refs) {
        refs = createModuleObject3D(module);
        this.moduleObjects.set(module.id, refs);
        this.moduleGroup.add(refs.group);
      }
      const transform = transforms.get(module.id);
      if (transform) updateModuleObject3D(refs, module, transform);
    }
  }

  /** Pull the camera back to fit the structure, so a big result isn't off-screen. */
  private frameCamera(spec: ShapeSpec): void {
    const centre = new Vector3();
    let radius = 1;
    if (spec.nodes.length > 0) {
      for (const node of spec.nodes) centre.add(new Vector3(...node.position));
      centre.divideScalar(spec.nodes.length);
      for (const node of spec.nodes) radius = Math.max(radius, centre.distanceTo(new Vector3(...node.position)));
    }
    const distance = radius * 3.2 + 4;
    this.controls.target.copy(centre);
    this.camera.position.set(centre.x + distance * 0.6, centre.y + distance * 0.5, centre.z + distance * 0.7);
    this.controls.update();
  }

  private animate = (): void => {
    this.frameHandle = requestAnimationFrame(this.animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };

  dispose(): void {
    cancelAnimationFrame(this.frameHandle);
    this.unsubscribe();
    this.resizeObserver.disconnect();
    this.controls.dispose();
    for (const refs of this.moduleObjects.values()) disposeModuleObject3D(refs);
    this.moduleObjects.clear();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}
