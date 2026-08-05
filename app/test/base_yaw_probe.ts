import { readFileSync, readdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { loadMujocoModule, mountModel, type ModelFiles } from '../src/lib/mujocoSession';
import { buildBoardSceneXml } from '../src/lib/boardSim';
import { injectBase, type BaseManifest } from '../src/lib/robots';

const here = dirname(fileURLToPath(import.meta.url));
const SO101 = join(here, '..', 'public', 'models', 'so101');
const LEKIWI = join(here, '..', 'public', 'models', 'lekiwi');
const mj = await loadMujocoModule();
let armXml = readFileSync(join(SO101, 'so101_new_calib.xml'), 'utf8');
armXml = armXml.replace('<body name="base" pos="0 0 0"', '<body name="base" mocap="true" pos="0 0 0"');
const manifest = JSON.parse(readFileSync(join(LEKIWI, 'base.json'), 'utf8')) as BaseManifest;
let scene = buildBoardSceneXml(armXml, { physics: true });
scene = injectBase(scene, manifest);
const files: ModelFiles = { 'scene.xml': scene };
for (const f of readdirSync(join(SO101, 'assets'))) if (f.endsWith('.stl')) files[`assets/${f}`] = new Uint8Array(readFileSync(join(SO101, 'assets', f)));
for (const m of manifest.meshes) files[`assets/${m.file}`] = new Uint8Array(readFileSync(join(LEKIWI, 'assets', m.file)));
const { model, data } = mountModel(mj, files, 'scene.xml');

const bid = mj.mj_name2id(model, 1, 'lekiwi_base');
const mocapId = (model.body_mocapid as Int32Array)[bid];
// pick a base geom far from the axis (a wheel) and watch it rotate about Z.
const gid = mj.mj_name2id(model, 5, 'g0'); // geoms are auto-named; fall back to any
function geomXY() {
  const gx = data.geom_xpos as Float64Array;
  // find the base geom with max |y| to track rotation
  let best = -1, by = 0;
  for (let g = 0; g < model.ngeom; g++) {
    const y = Math.abs(gx[g * 3 + 1]);
    if (y > by) { by = y; best = g; }
  }
  return [gx[best * 3], gx[best * 3 + 1]];
}
const mq = data.mocap_quat as Float64Array;
const set = (w: number, x: number, y: number, z: number) => { mq[mocapId * 4] = w; mq[mocapId * 4 + 1] = x; mq[mocapId * 4 + 2] = y; mq[mocapId * 4 + 3] = z; mj.mj_forward(model, data); };
// flip only (yaw 0): [0,1,0,0]
set(0, 1, 0, 0);
const a = geomXY();
// yaw 90 ∘ flip = [0, cos45, sin45, 0]
set(0, Math.cos(Math.PI / 4), Math.sin(Math.PI / 4), 0);
const b = geomXY();
console.log('base extreme geom xy @yaw0 :', a.map((v) => v.toFixed(3)));
console.log('base extreme geom xy @yaw90:', b.map((v) => v.toFixed(3)));
console.log('changed:', Math.hypot(a[0] - b[0], a[1] - b[1]).toFixed(3));
void gid;
