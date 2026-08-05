// The full LeKiwi sim scene — mocap arm + physics free-body block + collidable
// floor + sliding shelves + injected base — must compile and step stably.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, readdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { loadMujocoModule, mountModel, type ModelFiles } from '../src/lib/mujocoSession';
import { buildBoardSceneXml } from '../src/lib/boardSim';
import { injectBase, type BaseManifest } from '../src/lib/robots';
import { injectShelves, shelfJointName } from '../src/lib/shelves';

const here = dirname(fileURLToPath(import.meta.url));
const SO101 = join(here, '..', 'public', 'models', 'so101');
const LEKIWI = join(here, '..', 'public', 'models', 'lekiwi');
const COUNT = 3;

test('LeKiwi sim scene (arm+block+shelves) compiles and steps', async () => {
  const mj = await loadMujocoModule();
  let armXml = readFileSync(join(SO101, 'so101_new_calib.xml'), 'utf8');
  armXml = armXml.replace('<body name="base" pos="0 0 0"', '<body name="base" mocap="true" pos="0 0 0"');
  const manifest = JSON.parse(readFileSync(join(LEKIWI, 'base.json'), 'utf8')) as BaseManifest;
  let scene = buildBoardSceneXml(armXml, { physics: true });
  scene = injectBase(scene, manifest);
  scene = injectShelves(scene, COUNT);

  const files: ModelFiles = { 'scene.xml': scene };
  for (const f of readdirSync(join(SO101, 'assets'))) if (f.endsWith('.stl')) files[`assets/${f}`] = new Uint8Array(readFileSync(join(SO101, 'assets', f)));
  for (const m of manifest.meshes) files[`assets/${m.file}`] = new Uint8Array(readFileSync(join(LEKIWI, 'assets', m.file)));

  const { model, data } = mountModel(mj, files, 'scene.xml');
  // Every drawer joint should exist and drive open.
  for (let i = 0; i < COUNT; i++) {
    const jid = mj.mj_name2id(model, 3, shelfJointName(i));
    assert.ok(jid >= 0, `${shelfJointName(i)} exists`);
    (data.qpos as Float64Array)[(model.jnt_qposadr as Int32Array)[jid]] = 0.1;
  }
  for (let s = 0; s < 300; s++) mj.mj_step(model, data);
  const blockJid = mj.mj_name2id(model, 3, 'block_free');
  const z = (data.qpos as Float64Array)[(model.jnt_qposadr as Int32Array)[blockJid] + 2];
  assert.ok(Number.isFinite(z), 'sim did not explode');
});
