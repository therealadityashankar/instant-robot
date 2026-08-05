// Robot registry: which arm model to load and, for mobile robots, the static
// base geometry to mount underneath it. Both robots share the same SO-101 arm
// (and therefore the same IK); LeKiwi adds a decorative mobile base whose
// placement relative to the arm is user-configurable (see Simulator).

export type RobotId = 'so101' | 'lekiwi';

export interface RobotBase {
  dir: string; // public/models/<dir>
  manifest: string; // JSON manifest filename under that dir
  /** Default base position relative to the arm mount (m), [x, y, z]. */
  defaultOffset: [number, number, number];
}

export interface RobotDef {
  id: RobotId;
  label: string;
  base?: RobotBase;
}

export const ROBOTS: RobotDef[] = [
  { id: 'so101', label: 'SO-101 (fixed arm)' },
  {
    id: 'lekiwi',
    label: 'LeKiwi (mobile base)',
    base: { dir: 'lekiwi', manifest: 'base.json', defaultOffset: [0, 0, 0] },
  },
];

export function robotById(id: RobotId): RobotDef {
  return ROBOTS.find((r) => r.id === id) ?? ROBOTS[0];
}

// ── Base geometry manifest (produced by tags-and-borders/lekiwi_base_from_urdf.py) ──

export interface BaseMesh {
  name: string;
  file: string;
  scale: [number, number, number];
}
export interface BaseGeom {
  mesh: string;
  pos: [number, number, number];
  quat: [number, number, number, number]; // w, x, y, z
}
export interface BaseManifest {
  /** Distance from the arm mount to the lowest base point (m) — how far to lift
   *  the arm so the wheels rest on the floor. */
  baseDrop: number;
  meshes: BaseMesh[];
  geoms: BaseGeom[];
}

/**
 * Inject a mobile base into an already-composed scene XML: mesh assets before
 * `</asset>` and a single mocap body (so its offset can be tuned live via
 * `mocap_pos`) before `</worldbody>`. Must be called AFTER the block mocap body
 * is injected so the base takes mocap index 1 and the block stays index 0.
 */
export function injectBase(xml: string, manifest: BaseManifest): string {
  const f = (n: number) => n.toFixed(6);
  const meshes = manifest.meshes
    .map((m) => `<mesh name="${m.name}" file="${m.file}" scale="${m.scale.map(f).join(' ')}"/>`)
    .join('\n    ');
  const geoms = manifest.geoms
    .map(
      (g) =>
        `<geom type="mesh" mesh="${g.mesh}" pos="${g.pos.map(f).join(' ')}" ` +
        `quat="${g.quat.map(f).join(' ')}" rgba="0.32 0.33 0.37 1" ` +
        `contype="0" conaffinity="0" group="2"/>`,
    )
    .join('\n      ');
  const body = `<body name="lekiwi_base" mocap="true" pos="0 0 0">\n      ${geoms}\n    </body>`;

  let out = xml;
  const assetIdx = out.lastIndexOf('</asset>');
  if (assetIdx >= 0) out = out.slice(0, assetIdx) + `  ${meshes}\n  ` + out.slice(assetIdx);
  const wbIdx = out.lastIndexOf('</worldbody>');
  if (wbIdx >= 0) out = out.slice(0, wbIdx) + `  ${body}\n  ` + out.slice(wbIdx);
  return out;
}
