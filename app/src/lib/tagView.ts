// A minimal three.js scene showing just two things: the ROBOT and the detected
// ArUco TAGS, in a common world frame — so you can see where the robot is relative
// to the tags it sees. The robot is rendered from its MuJoCo geoms (arm + base
// only, not the sim's board/block/shelves); the tags are placed at world poses the
// caller derives from the onboard-camera detections.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { geomGeometry } from './mujocoRender';
import type { MjModel, MjData } from './mujocoSession';

export interface WorldTag {
  id: number;
  R: number[]; // row-major 3×3, tag orientation in world
  p: [number, number, number]; // world position (m)
  sizeMm: number;
}

function hueColor(id: number): THREE.Color {
  return new THREE.Color().setHSL(((id * 47) % 360) / 360, 0.6, 0.55);
}

function labelSprite(text: string, color: THREE.Color): THREE.Sprite {
  const c = document.createElement('canvas');
  c.width = 128;
  c.height = 64;
  const ctx = c.getContext('2d')!;
  ctx.font = 'bold 44px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.lineWidth = 6;
  ctx.strokeStyle = 'rgba(0,0,0,0.85)';
  ctx.strokeText(text, 64, 32);
  ctx.fillStyle = '#' + color.getHexString();
  ctx.fillText(text, 64, 32);
  const tex = new THREE.CanvasTexture(c);
  tex.anisotropy = 4;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  s.scale.set(0.07, 0.035, 1);
  return s;
}

export class TagView {
  private renderer: THREE.WebGLRenderer;
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;
  private robotGroup: THREE.Group;
  private tagGroup: THREE.Group;
  private robotMeshes: Array<{ mesh: THREE.Mesh; geomId: number }> = [];
  private model: MjModel | null = null;
  private data: MjData | null = null;

  constructor(canvas: HTMLCanvasElement) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0e1013);

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
    this.camera.up.set(0, 0, 1); // MuJoCo world is z-up
    this.camera.position.set(1.745, -0.277, 1.234);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(0.0, -0.1, 0.15);
    this.controls.enableDamping = true;

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.85));
    const dir = new THREE.DirectionalLight(0xffffff, 1.0);
    dir.position.set(0.5, -0.6, 1.2);
    this.scene.add(dir);
    const grid = new THREE.GridHelper(4, 40, 0x334155, 0x233044);
    grid.rotation.x = Math.PI / 2; // XY plane (z-up)
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(0.1)); // world origin

    this.robotGroup = new THREE.Group();
    this.tagGroup = new THREE.Group();
    this.scene.add(this.robotGroup, this.tagGroup);
  }

  /** Build robot meshes once for the given geom ids (arm + base only). */
  setRobot(model: MjModel, data: MjData, geomIds: number[]) {
    this.model = model;
    this.data = data;
    for (const { mesh } of this.robotMeshes) mesh.geometry.dispose();
    this.robotGroup.clear();
    this.robotMeshes = [];
    const type = model.geom_type as Int32Array;
    const size = model.geom_size as Float32Array;
    const rgba = model.geom_rgba as Float32Array;
    const dataid = model.geom_dataid as Int32Array;
    for (const i of geomIds) {
      const g = geomGeometry(model, type[i], [size[i * 3], size[i * 3 + 1], size[i * 3 + 2]], dataid[i]);
      if (!g) continue;
      const mesh = new THREE.Mesh(
        g,
        new THREE.MeshStandardMaterial({
          color: new THREE.Color(rgba[i * 4], rgba[i * 4 + 1], rgba[i * 4 + 2]),
          metalness: 0.1,
          roughness: 0.8,
        }),
      );
      this.robotGroup.add(mesh);
      this.robotMeshes.push({ mesh, geomId: i });
    }
  }

  /** Re-place the robot meshes from the live world transforms. */
  private updateRobot() {
    if (!this.data) return;
    const xpos = this.data.geom_xpos as Float64Array;
    const xmat = this.data.geom_xmat as Float64Array;
    const m = new THREE.Matrix4();
    for (const { mesh, geomId } of this.robotMeshes) {
      const p = geomId * 3;
      const r = geomId * 9;
      mesh.position.set(xpos[p], xpos[p + 1], xpos[p + 2]);
      m.set(
        xmat[r], xmat[r + 1], xmat[r + 2], 0,
        xmat[r + 3], xmat[r + 4], xmat[r + 5], 0,
        xmat[r + 6], xmat[r + 7], xmat[r + 8], 0,
        0, 0, 0, 1,
      );
      mesh.quaternion.setFromRotationMatrix(m);
    }
  }

  /** Replace the shown tags (already in world coordinates). */
  setTags(tags: WorldTag[]) {
    for (const child of this.tagGroup.children) {
      const c = child as THREE.Mesh & THREE.Sprite;
      c.geometry?.dispose?.();
      const mat = c.material as THREE.Material & { map?: THREE.Texture };
      mat?.map?.dispose?.();
      mat?.dispose?.();
    }
    this.tagGroup.clear();

    const m4 = new THREE.Matrix4();
    for (const tag of tags) {
      const color = hueColor(tag.id);
      const s = tag.sizeMm / 1000;
      const plane = new THREE.Mesh(
        new THREE.PlaneGeometry(s, s),
        new THREE.MeshStandardMaterial({ color, side: THREE.DoubleSide, transparent: true, opacity: 0.8, roughness: 0.9 }),
      );
      const R = tag.R;
      m4.set(R[0], R[1], R[2], 0, R[3], R[4], R[5], 0, R[6], R[7], R[8], 0, 0, 0, 0, 1);
      plane.quaternion.setFromRotationMatrix(m4);
      plane.position.set(tag.p[0], tag.p[1], tag.p[2]);
      this.tagGroup.add(plane);

      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(plane.geometry),
        new THREE.LineBasicMaterial({ color }),
      );
      edges.quaternion.copy(plane.quaternion);
      edges.position.copy(plane.position);
      this.tagGroup.add(edges);

      const label = labelSprite(String(tag.id), color);
      label.position.set(tag.p[0], tag.p[1], tag.p[2] + s * 0.75);
      this.tagGroup.add(label);
    }
  }

  render() {
    this.updateRobot();
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
    this.setTags([]);
    for (const { mesh } of this.robotMeshes) mesh.geometry.dispose();
    this.renderer.dispose();
  }
}
