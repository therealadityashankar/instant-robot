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

function geomGeometry(
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

export class MujocoRenderer {
  private renderer: THREE.WebGLRenderer;
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;
  private meshes: Array<{ mesh: THREE.Mesh; geomId: number }> = [];
  private model: MjModel;
  private data: MjData;
  private targetMarker: THREE.Mesh;

  constructor(canvas: HTMLCanvasElement, _mj: Mj, model: MjModel, data: MjData) {
    this.model = model;
    this.data = data;

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x14161a);

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
    this.camera.up.set(0, 0, 1); // MuJoCo z-up
    this.camera.position.set(0.55, -0.55, 0.5);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(0.25, 0, 0.2);
    this.controls.enableDamping = true;

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.75));
    const dir = new THREE.DirectionalLight(0xffffff, 1.4);
    dir.position.set(0.5, -0.6, 1.2);
    this.scene.add(dir);
    const grid = new THREE.GridHelper(1, 20, 0x334155, 0x233044);
    grid.rotation.x = Math.PI / 2; // grid in XY plane (z-up)
    this.scene.add(grid);

    this.buildGeoms();

    // Target marker for the IK goal.
    this.targetMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.008, 16, 12),
      new THREE.MeshBasicMaterial({ color: 0x22c55e, wireframe: true }),
    );
    this.scene.add(this.targetMarker);

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

  /** Position the IK target marker (world x,y,z). */
  setTarget(pos: [number, number, number]) {
    this.targetMarker.position.set(pos[0], pos[1], pos[2]);
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
    this.controls.update();
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
