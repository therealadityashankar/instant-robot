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
// ── Procedural Textures & Materials for Items, Props & Stations ───────────────

const TEXTURE_CACHE = new Map<string, THREE.CanvasTexture>();

function getOrCreateTexture(
  key: string,
  drawFn: (ctx: CanvasRenderingContext2D, w: number, h: number) => void,
  w = 256,
  h = 256,
): THREE.CanvasTexture {
  if (TEXTURE_CACHE.has(key)) return TEXTURE_CACHE.get(key)!;
  const c = document.createElement('canvas');
  c.width = w;
  c.height = h;
  const ctx = c.getContext('2d')!;
  drawFn(ctx, w, h);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  TEXTURE_CACHE.set(key, tex);
  return tex;
}

/** Realistic natural wood grain texture with growth rings and fibers. */
function getWoodBlockTexture(): THREE.CanvasTexture {
  return getOrCreateTexture('wood_block', (ctx, w, h) => {
    const grad = ctx.createLinearGradient(0, 0, w, 0);
    grad.addColorStop(0, '#d97706');
    grad.addColorStop(0.3, '#b45309');
    grad.addColorStop(0.7, '#d97706');
    grad.addColorStop(1, '#92400e');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, w, h);

    // Fine wood grain rings & long fibers
    for (let i = 0; i < 300; i++) {
      const y = Math.random() * h;
      ctx.strokeStyle = `rgba(120, 53, 15, ${0.08 + Math.random() * 0.18})`;
      ctx.lineWidth = 0.6 + Math.random() * 1.8;
      ctx.beginPath();
      ctx.moveTo(0, y);
      for (let x = 0; x <= w; x += 16) {
        ctx.lineTo(x, y + Math.sin(x / 20 + i * 0.3) * 2.5 + Math.sin(x / 40) * 1.5);
      }
      ctx.stroke();
    }
    // Growth ring subtle accents
    for (let i = 0; i < 15; i++) {
      ctx.strokeStyle = 'rgba(67, 20, 7, 0.25)';
      ctx.lineWidth = 1.2;
      const y = (i / 15) * h;
      ctx.beginPath();
      ctx.moveTo(0, y);
      for (let x = 0; x <= w; x += 32) {
        ctx.lineTo(x, y + Math.cos(x / 30) * 3);
      }
      ctx.stroke();
    }
  });
}

/** Glossy crimson apple skin with stippling and golden undertones. */
function getAppleTexture(): THREE.CanvasTexture {
  return getOrCreateTexture('apple_skin', (ctx, w, h) => {
    const grad = ctx.createRadialGradient(w * 0.4, h * 0.3, 10, w * 0.5, h * 0.5, w * 0.6);
    grad.addColorStop(0, '#ef4444');
    grad.addColorStop(0.4, '#dc2626');
    grad.addColorStop(0.8, '#991b1b');
    grad.addColorStop(1, '#7f1d1d');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, w, h);

    // Natural golden undertone streaks
    for (let i = 0; i < 80; i++) {
      const x = Math.random() * w;
      ctx.strokeStyle = `rgba(251, 191, 36, ${0.08 + Math.random() * 0.12})`;
      ctx.lineWidth = 1 + Math.random() * 2;
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x + (Math.random() - 0.5) * 20, h);
      ctx.stroke();
    }
    // Micro-stippling (apple lenticels)
    for (let i = 0; i < 400; i++) {
      ctx.fillStyle = `rgba(254, 240, 138, ${0.15 + Math.random() * 0.35})`;
      ctx.beginPath();
      ctx.arc(Math.random() * w, Math.random() * h, 0.6 + Math.random() * 0.8, 0, Math.PI * 2);
      ctx.fill();
    }
  });
}

/** Dimpled citrus orange peel with tactile pores. */
function getOrangeTexture(): THREE.CanvasTexture {
  return getOrCreateTexture('orange_peel', (ctx, w, h) => {
    const grad = ctx.createRadialGradient(w * 0.4, h * 0.4, 20, w * 0.5, h * 0.5, w * 0.6);
    grad.addColorStop(0, '#fb923c');
    grad.addColorStop(0.6, '#f97316');
    grad.addColorStop(1, '#c2410c');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, w, h);

    // Dimpled relief pores
    for (let i = 0; i < 1200; i++) {
      const x = Math.random() * w;
      const y = Math.random() * h;
      const r = 0.8 + Math.random() * 1.2;
      ctx.fillStyle = 'rgba(154, 52, 18, 0.45)';
      ctx.beginPath();
      ctx.arc(x, y, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = 'rgba(254, 215, 170, 0.35)';
      ctx.beginPath();
      ctx.arc(x - 0.5, y - 0.5, r * 0.7, 0, Math.PI * 2);
      ctx.fill();
    }
  });
}

/** Banana peel with longitudinal facet shading and ripening speckles. */
function getBananaTexture(): THREE.CanvasTexture {
  return getOrCreateTexture('banana_peel', (ctx, w, h) => {
    const grad = ctx.createLinearGradient(0, 0, 0, h);
    grad.addColorStop(0, '#84cc16'); // green stem end
    grad.addColorStop(0.12, '#eab308');
    grad.addColorStop(0.85, '#facc15');
    grad.addColorStop(1, '#65a30d'); // green tip
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, w, h);

    // Longitudinal peel ribs
    for (let i = 1; i < 5; i++) {
      const x = (i / 5) * w;
      ctx.strokeStyle = 'rgba(161, 98, 7, 0.35)';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    // Natural sugar speckles
    for (let i = 0; i < 120; i++) {
      ctx.fillStyle = `rgba(113, 63, 18, ${0.4 + Math.random() * 0.4})`;
      ctx.beginPath();
      ctx.arc(Math.random() * w, Math.random() * h, 0.8 + Math.random() * 1.4, 0, Math.PI * 2);
      ctx.fill();
    }
  });
}

/** Woven cane / wicker basket weave texture. */
function getWickerTexture(): THREE.CanvasTexture {
  return getOrCreateTexture('wicker_basket', (ctx, w, h) => {
    ctx.fillStyle = '#78350f';
    ctx.fillRect(0, 0, w, h);

    const step = 16;
    for (let x = 0; x < w; x += step) {
      for (let y = 0; y < h; y += step) {
        const isHorizontal = ((x / step + y / step) % 2) === 0;
        ctx.fillStyle = isHorizontal ? '#b45309' : '#92400e';
        ctx.fillRect(x + 1, y + 1, step - 2, step - 2);

        // Shadow crevices
        ctx.fillStyle = 'rgba(41, 15, 3, 0.6)';
        ctx.fillRect(x, y, step, 1.5);
        ctx.fillRect(x, y, 1.5, step);

        // Highlight sheen
        ctx.fillStyle = 'rgba(251, 191, 36, 0.25)';
        if (isHorizontal) {
          ctx.fillRect(x + 2, y + step / 2 - 1, step - 4, 2);
        } else {
          ctx.fillRect(x + step / 2 - 1, y + 2, 2, step - 4);
        }
      }
    }
  });
}

/** Textured industrial matte pedestal / dark slate granite. */
function getPedestalTexture(): THREE.CanvasTexture {
  return getOrCreateTexture('pedestal_stone', (ctx, w, h) => {
    ctx.fillStyle = '#292d39';
    ctx.fillRect(0, 0, w, h);

    for (let i = 0; i < 1500; i++) {
      const x = Math.random() * w;
      const y = Math.random() * h;
      const bright = Math.random() > 0.5;
      ctx.fillStyle = bright
        ? `rgba(255, 255, 255, ${0.04 + Math.random() * 0.08})`
        : `rgba(0, 0, 0, ${0.1 + Math.random() * 0.15})`;
      ctx.fillRect(x, y, 1.2, 1.2);
    }
  });
}

/** Birch plywood / clean board veneer texture. */
function getBoardTexture(): THREE.CanvasTexture {
  return getOrCreateTexture('board_birch', (ctx, w, h) => {
    ctx.fillStyle = '#ebdccb';
    ctx.fillRect(0, 0, w, h);

    for (let i = 0; i < 150; i++) {
      const y = Math.random() * h;
      ctx.strokeStyle = `rgba(180, 140, 100, ${0.05 + Math.random() * 0.1})`;
      ctx.lineWidth = 0.8 + Math.random() * 1.5;
      ctx.beginPath();
      ctx.moveTo(0, y);
      for (let x = 0; x <= w; x += 20) {
        ctx.lineTo(x, y + Math.sin(x / 30 + i) * 1.5);
      }
      ctx.stroke();
    }
  });
}

/** 3D printed stand texture with 0.2mm layer lines. */
function get3DPrintedTexture(): THREE.CanvasTexture {
  return getOrCreateTexture('stand_printed', (ctx, w, h) => {
    ctx.fillStyle = '#334155';
    ctx.fillRect(0, 0, w, h);

    for (let y = 0; y < h; y += 3) {
      ctx.fillStyle = (y % 6 === 0) ? 'rgba(255, 255, 255, 0.05)' : 'rgba(0, 0, 0, 0.12)';
      ctx.fillRect(0, y, w, 1.5);
    }
  });
}

/**
 * Assign rich textured Three.js materials to MuJoCo geoms based on their semantic identity.
 */
function createMaterialForGeom(geomName: string, r: number, g: number, b: number, a: number): THREE.Material {
  const baseColor = new THREE.Color(r, g, b);

  // 1. Wooden Jenga Block
  if (geomName.startsWith('block_geom') || geomName === 'block') {
    const tex = getWoodBlockTexture();
    return new THREE.MeshStandardMaterial({
      map: tex,
      color: new THREE.Color(0xffffff),
      roughness: 0.55,
      metalness: 0.05,
    });
  }

  // 2. Apple
  if (geomName.startsWith('apple_') && !geomName.includes('stalk')) {
    const tex = getAppleTexture();
    return new THREE.MeshStandardMaterial({
      map: tex,
      color: new THREE.Color(0xffffff),
      roughness: 0.22,
      metalness: 0.08,
    });
  }

  // 3. Orange
  if (geomName.startsWith('orange_') && !geomName.includes('stalk')) {
    const tex = getOrangeTexture();
    return new THREE.MeshStandardMaterial({
      map: tex,
      color: new THREE.Color(0xffffff),
      roughness: 0.45,
      metalness: 0.04,
    });
  }

  // 4. Banana
  if (geomName.startsWith('banana_')) {
    const tex = getBananaTexture();
    return new THREE.MeshStandardMaterial({
      map: tex,
      color: new THREE.Color(0xffffff),
      roughness: 0.35,
      metalness: 0.05,
    });
  }

  // 5. Basket (Woven Wicker)
  if (geomName.startsWith('basket_')) {
    const tex = getWickerTexture();
    tex.repeat.set(2, 2);
    return new THREE.MeshStandardMaterial({
      map: tex,
      color: new THREE.Color(0xffffff),
      roughness: 0.65,
      metalness: 0.05,
    });
  }

  // 6. Bottle (Translucent Glass & Cap)
  if (geomName.startsWith('bottle_') && !geomName.includes('cap')) {
    return new THREE.MeshPhysicalMaterial({
      color: new THREE.Color(0x38bdf8),
      transmission: 0.75,
      opacity: 0.85,
      transparent: true,
      roughness: 0.1,
      metalness: 0.1,
      ior: 1.5,
    });
  }
  if (geomName.startsWith('bottle_cap')) {
    return new THREE.MeshStandardMaterial({
      color: new THREE.Color(0xe2e8f0),
      metalness: 0.85,
      roughness: 0.25,
    });
  }

  // 7. Stalk / Stems
  if (geomName.includes('stalk')) {
    return new THREE.MeshStandardMaterial({
      color: new THREE.Color(0x451a03),
      roughness: 0.85,
      metalness: 0.05,
    });
  }

  // 8. Pedestals
  if (geomName.startsWith('station_pedestal')) {
    const tex = getPedestalTexture();
    tex.repeat.set(2, 2);
    return new THREE.MeshStandardMaterial({
      map: tex,
      color: new THREE.Color(0xffffff),
      roughness: 0.75,
      metalness: 0.15,
    });
  }

  // 9. Boards (Station & Calibration)
  if (geomName.startsWith('station_board') || geomName.startsWith('board_surface') || geomName.startsWith('board_interior')) {
    const tex = getBoardTexture();
    return new THREE.MeshStandardMaterial({
      map: tex,
      color: new THREE.Color(0xffffff),
      roughness: 0.45,
      metalness: 0.05,
    });
  }

  // 10. Boundary Stand 3D Mesh
  if (geomName.startsWith('boundary_stand')) {
    const tex = get3DPrintedTexture();
    tex.repeat.set(3, 3);
    return new THREE.MeshStandardMaterial({
      map: tex,
      color: new THREE.Color(0xffffff),
      roughness: 0.6,
      metalness: 0.1,
    });
  }

  // Fallback default material with slight specular enhancement
  return new THREE.MeshStandardMaterial({
    color: baseColor,
    metalness: 0.1,
    roughness: 0.65,
  });
}

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
  tex.repeat.set(18, 18);
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(28, 28),
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
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(1);
    this.scene = scene;
    this.camera = new THREE.PerspectiveCamera(fovDeg, 4 / 3, 0.01, 100);
    this.camera.up.set(0, 0, 1);
  }

  get fovDeg(): number {
    return this.camera.fov;
  }

  /** Aim using an OpenCV pose (x right, y down, z forward). */
  setPose(R: number[], t: [number, number, number]) {
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
  private mj?: Mj;
  private model: MjModel;
  private data: MjData;
  private targetMarker: THREE.Mesh;
  private targetSphere!: THREE.Mesh;

  private pov: boolean;

  constructor(
    canvas: HTMLCanvasElement,
    mj: Mj,
    model: MjModel,
    data: MjData,
    opts: { pov?: boolean } = {},
  ) {
    this.pov = !!opts.pov;
    this.mj = mj;
    this.model = model;
    this.data = data;

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x14161a);

    this.camera = new THREE.PerspectiveCamera(this.pov ? 46 : 45, 1, 0.01, 100);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(-0.570, 0.189, 1.524);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(0.309, 0.153, 0.514);
    this.controls.enableDamping = true;
    this.controls.enabled = !this.pov;
    this.controls.maxDistance = 6.25; // max zoomout derived from cam (-4.311, -0.262, 4.510) and target (0.454, -0.145, 0.468)

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.75));
    const dir = new THREE.DirectionalLight(0xffffff, 1.4);
    dir.position.set(0.5, -0.6, 1.2);
    this.scene.add(dir);
    this.scene.add(woodenFloor());

    this.buildGeoms();

    const mat = new THREE.MeshBasicMaterial({ color: 0x22c55e, wireframe: true });
    const cone = new THREE.ConeGeometry(0.014, 0.045, 4);
    cone.rotateZ(-Math.PI / 2);
    cone.translate(-0.0225, 0, 0);
    this.targetMarker = new THREE.Mesh(cone, mat);
    this.scene.add(this.targetMarker);

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
      if (rgba[i * 4 + 3] <= 0) continue;
      if (group[i] >= 3) continue;
      const g = geomGeometry(
        model,
        type[i],
        [size[i * 3], size[i * 3 + 1], size[i * 3 + 2]],
        dataid[i],
      );
      if (!g) continue;

      const geomName = this.mj ? (this.mj.mj_id2name(model, 5 /* mjOBJ_GEOM */, i) || '') : '';
      const mat = createMaterialForGeom(
        geomName,
        rgba[i * 4],
        rgba[i * 4 + 1],
        rgba[i * 4 + 2],
        rgba[i * 4 + 3],
      );

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
