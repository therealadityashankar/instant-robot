// three.js renderer for a MuJoCo model. Builds one three.js mesh per MuJoCo geom
// from the *compiled* model geometry (primitives from geom_size; meshes from the
// model's mesh_vert/mesh_face arrays — no STL re-parsing needed), then each frame
// positions them from data.geom_xpos / geom_xmat. MuJoCo is z-up, so the camera
// up-vector is set to +Z.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import type { Mj, MjModel, MjData } from './mujocoSession';

// mjtGeom
const PLANE = 0;
const SPHERE = 2;
const CAPSULE = 3;
const ELLIPSOID = 4;
const CYLINDER = 5;
const BOX = 6;
const MESH = 7;

export function geomGeometry(
  model: MjModel,
  type: number,
  size: [number, number, number],
  dataid: number,
): THREE.BufferGeometry | null {
  switch (type) {
    case PLANE: {
      const s = size[0] > 0 ? size[0] * 2 : 4;
      return new THREE.PlaneGeometry(s, s); // normal is +Z, matching MuJoCo
    }
    case SPHERE:
      return new THREE.SphereGeometry(size[0], 24, 16);
    case ELLIPSOID: {
      const g = new THREE.SphereGeometry(1, 24, 16);
      g.scale(size[0], size[1], size[2]);
      return g;
    }
    case CAPSULE: {
      // MuJoCo: size[0]=radius, size[1]=half-length, axis = local Z.
      const g = new THREE.CapsuleGeometry(size[0], size[1] * 2, 8, 16);
      g.rotateX(Math.PI / 2); // three capsule is Y-axis → make it Z
      return g;
    }
    case CYLINDER: {
      const g = new THREE.CylinderGeometry(size[0], size[0], size[1] * 2, 24);
      g.rotateX(Math.PI / 2);
      return g;
    }
    case BOX:
      return new THREE.BoxGeometry(size[0] * 2, size[1] * 2, size[2] * 2);
    case MESH:
      return meshGeometry(model, dataid);
    default:
      return null;
  }
}

function meshGeometry(model: MjModel, meshId: number): THREE.BufferGeometry {
  const vertAdr = model.mesh_vertadr[meshId];
  const vertNum = model.mesh_vertnum[meshId];
  const faceAdr = model.mesh_faceadr[meshId];
  const faceNum = model.mesh_facenum[meshId];
  const verts = model.mesh_vert as Float32Array; // float3
  const faces = model.mesh_face as Int32Array; // int3, indices are mesh-local

  const positions = new Float32Array(faceNum * 9);
  for (let f = 0; f < faceNum; f++) {
    for (let k = 0; k < 3; k++) {
      const vi = faces[(faceAdr + f) * 3 + k]; // local vertex index
      const src = (vertAdr + vi) * 3;
      const dst = f * 9 + k * 3;
      positions[dst] = verts[src];
      positions[dst + 1] = verts[src + 1];
      positions[dst + 2] = verts[src + 2];
    }
  }
  void vertNum;
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  g.computeVertexNormals();
  return g;
}

/**
 * A second window onto an existing scene, from an explicit camera pose — the
 * robot's-eye views.
 *
 * It borrows the scene rather than building one: a MujocoRenderer constructs a
 * mesh per geom, and the arm alone is ~374k triangles, so standing up extra
 * renderers with their own copies is what made the app crawl. Sharing means the
 * geometry is built and animated once no matter how many views look at it.
 */
/**
 * A wooden floor: nicer to look at than a grid, and the grain gives the eye a
 * sense of scale and motion that flat colour doesn't. Drawn procedurally at
 * 256×256 and tiled, so it costs a quarter of a megabyte rather than an asset.
 */
function woodenFloor(): THREE.Mesh {
  const N = 256;
  const c = document.createElement('canvas');
  c.width = N;
  c.height = N;
  const ctx = c.getContext('2d')!;
  ctx.fillStyle = '#8a5a33';
  ctx.fillRect(0, 0, N, N);
  // Grain: many faint darker streaks along the plank, plus plank seams.
  for (let i = 0; i < 220; i++) {
    const y = Math.random() * N;
    ctx.strokeStyle = `rgba(60, 34, 16, ${0.04 + Math.random() * 0.1})`;
    ctx.lineWidth = 0.5 + Math.random() * 1.5;
    ctx.beginPath();
    ctx.moveTo(0, y);
    for (let x = 0; x <= N; x += 16) ctx.lineTo(x, y + Math.sin(x / 26 + i) * 1.6);
    ctx.stroke();
  }
  ctx.strokeStyle = 'rgba(45, 25, 12, 0.55)';
  ctx.lineWidth = 1.5;
  for (const y of [0, N / 4, N / 2, (3 * N) / 4]) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(N, y);
    ctx.stroke();
  }

  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(6, 6);
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(8, 8),
    new THREE.MeshLambertMaterial({ map: tex }),
  );
  floor.position.z = -0.002; // just under the model's own floor plane, if any
  return floor;
}

export class PovView {
  private renderer: THREE.WebGLRenderer;
  private camera: THREE.PerspectiveCamera;
  private scene: THREE.Scene;

  constructor(canvas: HTMLCanvasElement, scene: THREE.Scene, fovDeg = 46) {
    // preserveDrawingBuffer: these frames are read back and decoded, so the
    // buffer has to survive past the render.
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(1); // detection wants exact pixels, not device scaling
    this.scene = scene;
    this.camera = new THREE.PerspectiveCamera(fovDeg, 4 / 3, 0.01, 100);
    this.camera.up.set(0, 0, 1);
  }

  get fovDeg(): number {
    return this.camera.fov;
  }

  /** Aim using an OpenCV pose (x right, y down, z forward). */
  setPose(R: number[], t: [number, number, number]) {
    // three.js looks down −Z with +Y up, so the basis is x, −y, −z of that one.
    const m = new THREE.Matrix4();
    m.set(
      R[0], -R[1], -R[2], 0,
      R[3], -R[4], -R[5], 0,
      R[6], -R[7], -R[8], 0,
      0, 0, 0, 1,
    );
    this.camera.quaternion.setFromRotationMatrix(m);
    this.camera.position.set(t[0], t[1], t[2]);
  }

  render(w: number, h: number) {
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.render(this.scene, this.camera);
  }

  dispose() {
    this.renderer.dispose();
  }
}

export class MujocoRenderer {
  private renderer: THREE.WebGLRenderer;
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;
  private meshes: Array<{ mesh: THREE.Mesh; geomId: number }> = [];
  private model: MjModel;
  private data: MjData;
  private targetMarker: THREE.Mesh;
  private targetSphere!: THREE.Mesh;

  private pov: boolean;

  /**
   * `opts.pov` renders the scene from an explicit camera pose (see setPovPose)
   * instead of an orbiting one — used to show what the robot's onboard camera
   * would see when no real camera is attached.
   */
  constructor(
    canvas: HTMLCanvasElement,
    _mj: Mj,
    model: MjModel,
    data: MjData,
    opts: { pov?: boolean } = {},
  ) {
    this.pov = !!opts.pov;
    this.model = model;
    this.data = data;

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x14161a);

    // 46° vertical ≈ a 60° horizontal view at 4:3 — a plausible webcam, and the
    // same cone simDetect uses to decide what the robot can see.
    this.camera = new THREE.PerspectiveCamera(this.pov ? 46 : 45, 1, 0.01, 100);
    this.camera.up.set(0, 0, 1); // MuJoCo z-up
    // Viewpoint chosen from the running app with "Copy view" — keeps the robot,
    // the board and the station all in frame.
    this.camera.position.set(1.745, -0.277, 1.234);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(0.0, -0.1, 0.15);
    this.controls.enableDamping = true;
    this.controls.enabled = !this.pov;

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.75));
    const dir = new THREE.DirectionalLight(0xffffff, 1.4);
    dir.position.set(0.5, -0.6, 1.2);
    this.scene.add(dir);
    this.scene.add(woodenFloor());

    this.buildGeoms();

    // Target marker: a cone whose tip is the goal point and whose axis is the
    // gripper's approach direction, with a square base showing the wrist roll.
    const mat = new THREE.MeshBasicMaterial({ color: 0x22c55e, wireframe: true });
    const cone = new THREE.ConeGeometry(0.014, 0.045, 4); // 4 sides → square base
    // Cone points +Y with tip at +h/2; re-aim to +X with tip at the origin.
    cone.rotateZ(-Math.PI / 2);
    cone.translate(-0.0225, 0, 0);
    this.targetMarker = new THREE.Mesh(cone, mat);
    this.scene.add(this.targetMarker);
    // Sphere marker, shown instead of the cone when the target has no orientation
    // (e.g. the "None" grasp — approach direction unspecified).
    this.targetSphere = new THREE.Mesh(new THREE.SphereGeometry(0.012, 16, 12), mat);
    this.targetSphere.visible = false;
    this.scene.add(this.targetSphere);

    this.update();
  }

  private buildGeoms() {
    const { model } = this;
    const type = model.geom_type as Int32Array;
    const size = model.geom_size as Float32Array;
    const rgba = model.geom_rgba as Float32Array;
    const dataid = model.geom_dataid as Int32Array;
    const group = model.geom_group as Int32Array;

    for (let i = 0; i < model.ngeom; i++) {
      // Skip fully-transparent geoms and collision-only groups (group 3+).
      if (rgba[i * 4 + 3] <= 0) continue;
      if (group[i] >= 3) continue;
      const g = geomGeometry(
        model,
        type[i],
        [size[i * 3], size[i * 3 + 1], size[i * 3 + 2]],
        dataid[i],
      );
      if (!g) continue;
      const mat = new THREE.MeshStandardMaterial({
        color: new THREE.Color(rgba[i * 4], rgba[i * 4 + 1], rgba[i * 4 + 2]),
        metalness: 0.1,
        roughness: 0.8,
      });
      const mesh = new THREE.Mesh(g, mat);
      this.scene.add(mesh);
      this.meshes.push({ mesh, geomId: i });
    }
  }

  /**
   * Position + orient the IK target marker. `rot` is the desired gripper world
   * rotation (row-major 3×3, columns = local x/y/z axes); its local X is the
   * approach the cone points along. Omit to keep the marker axis-aligned.
   */
  setTarget(pos: [number, number, number], rot?: number[]) {
    // With an orientation → oriented cone; without → sphere (direction-free).
    this.targetMarker.visible = !!rot;
    this.targetSphere.visible = !rot;
    const marker = rot ? this.targetMarker : this.targetSphere;
    marker.position.set(pos[0], pos[1], pos[2]);
    if (rot) {
      // three.js Matrix4.set takes row-major; basis columns are our local axes.
      const m = new THREE.Matrix4();
      m.set(rot[0], rot[1], rot[2], 0, rot[3], rot[4], rot[5], 0, rot[6], rot[7], rot[8], 0, 0, 0, 0, 1);
      this.targetMarker.quaternion.setFromRotationMatrix(m);
    }
  }

  /**
   * Paint a texture onto specific geoms — used to put real ArUco markers on the
   * tag geoms so the rendered image contains findable fiducials. Unlit
   * (MeshBasicMaterial) on purpose: shading a fiducial only makes it harder to
   * threshold, exactly as a glossy print does in real life.
   */
  setGeomTextures(textures: Map<number, HTMLCanvasElement>) {
    for (const { mesh, geomId } of this.meshes) {
      const canvas = textures.get(geomId);
      if (!canvas) continue;
      const tex = new THREE.CanvasTexture(canvas);
      tex.colorSpace = THREE.SRGBColorSpace;
      tex.magFilter = THREE.LinearFilter;
      tex.minFilter = THREE.LinearMipmapLinearFilter;
      tex.anisotropy = 8;
      mesh.material = new THREE.MeshBasicMaterial({ map: tex });
    }
  }

  /** Vertical field of view (degrees) — needed to synthesise sim intrinsics. */
  get fovDeg(): number {
    return this.camera.fov;
  }

  /** The scene itself, so extra views can render it without rebuilding it. */
  get sceneRef(): THREE.Scene {
    return this.scene;
  }

  /** Current orbit camera placement, for copying a good viewpoint out of the UI. */
  cameraState(): { pos: [number, number, number]; target: [number, number, number] } {
    const p = this.camera.position;
    const t = this.controls.target;
    return { pos: [p.x, p.y, p.z], target: [t.x, t.y, t.z] };
  }

  /**
   * Point the camera using an OpenCV-convention pose (R row-major camera→world,
   * x right / y down / z forward). three.js cameras look down −Z with +Y up, so
   * the basis is x, −y, −z of the given one.
   */
  setPovPose(R: number[], t: [number, number, number]) {
    const m = new THREE.Matrix4();
    m.set(
      R[0], -R[1], -R[2], 0,
      R[3], -R[4], -R[5], 0,
      R[6], -R[7], -R[8], 0,
      0, 0, 0, 1,
    );
    this.camera.quaternion.setFromRotationMatrix(m);
    this.camera.position.set(t[0], t[1], t[2]);
  }

  /** Pull the latest geom transforms from data and render one frame. */
  update() {
    const xpos = this.data.geom_xpos as Float64Array;
    const xmat = this.data.geom_xmat as Float64Array;
    const m = new THREE.Matrix4();
    for (const { mesh, geomId } of this.meshes) {
      const p = geomId * 3;
      const r = geomId * 9;
      mesh.position.set(xpos[p], xpos[p + 1], xpos[p + 2]);
      // geom_xmat is row-major 3×3.
      m.set(
        xmat[r], xmat[r + 1], xmat[r + 2], 0,
        xmat[r + 3], xmat[r + 4], xmat[r + 5], 0,
        xmat[r + 6], xmat[r + 7], xmat[r + 8], 0,
        0, 0, 0, 1,
      );
      mesh.quaternion.setFromRotationMatrix(m);
    }
    if (!this.pov) this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  resize(width: number, height: number) {
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  dispose() {
    this.controls.dispose();
    this.renderer.dispose();
    for (const { mesh } of this.meshes) mesh.geometry.dispose();
  }
}
