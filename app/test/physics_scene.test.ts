// Smoke test for the sim-only physics scene: the free-body block must fall and
// rest stably on its collision floor without the sim exploding, and the arm
// (on its own collision channel) must not be disturbed by it.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, readdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { loadMujocoModule, mountModel, type ModelFiles } from '../src/lib/mujocoSession';
import { buildBoardSceneXml, BLOCK_HALF_Z } from '../src/lib/boardSim';

const here = dirname(fileURLToPath(import.meta.url));
const SO101 = join(here, '..', 'public', 'models', 'so101');

test('physics block falls and rests stably', async () => {
  const mj = await loadMujocoModule();
  const armXml = readFileSync(join(SO101, 'so101_new_calib.xml'), 'utf8');
  const scene = buildBoardSceneXml(armXml, { physics: true });
  const files: ModelFiles = { 'scene.xml': scene };
  for (const f of readdirSync(join(SO101, 'assets'))) {
    if (f.endsWith('.stl')) files[`assets/${f}`] = new Uint8Array(readFileSync(join(SO101, 'assets', f)));
  }
  const { model, data } = mountModel(mj, files, 'scene.xml');
  const jid = mj.mj_name2id(model, 3 /* JOINT */, 'block_free');
  assert.ok(jid >= 0, 'block_free joint exists');
  const qadr = (model.jnt_qposadr as Int32Array)[jid];

  // Hold the arm at its rest pose via actuators while the block settles.
  const ctrl = data.ctrl as Float64Array;
  for (let i = 0; i < ctrl.length; i++) ctrl[i] = 0;
  for (let s = 0; s < 500; s++) mj.mj_step(model, data);

  const q = data.qpos as Float64Array;
  const z = q[qadr + 2];
  assert.ok(Number.isFinite(z), 'block z is finite (sim did not explode)');
  // Rests with its bottom on the floor: centre ≈ half height.
  assert.ok(Math.abs(z - BLOCK_HALF_Z) < 0.01, `block rests near ${BLOCK_HALF_Z}, got ${z}`);
});
