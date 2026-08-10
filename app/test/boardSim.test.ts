// Headless tests for the board→sim mapping and scene injection: the augmented
// model must load in MuJoCo, expose a mocap block, and place that block where
// interiorToSim says it should go.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, readdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { loadMujocoModule, mountModel, type ModelFiles } from '../src/lib/mujocoSession';
import { buildBoardSceneXml, interiorToSim } from '../src/lib/boardSim';

const here = dirname(fileURLToPath(import.meta.url));
const MODEL_DIR = join(here, '..', '..', 'models', 'so101');

function loadAugmented(): ModelFiles {
  const base = readFileSync(join(MODEL_DIR, 'so101_new_calib.xml'), 'utf8');
  const files: ModelFiles = { 'scene.xml': buildBoardSceneXml(base) };
  for (const f of readdirSync(join(MODEL_DIR, 'assets'))) {
    if (f.endsWith('.stl')) files[`assets/${f}`] = new Uint8Array(readFileSync(join(MODEL_DIR, 'assets', f)));
  }
  return files;
}

test('board scene loads with a mocap block, placed by interiorToSim', async () => {
  const mj = await loadMujocoModule();
  const { model, data } = mountModel(mj, loadAugmented(), 'scene.xml');

  // The scene also carries the temporary camera-rig markers, so identify the
  // block by name rather than assuming it's the only (or the first) mocap body.
  const blockBody = mj.mj_name2id(model, 1 /* mjOBJ_BODY */, 'block');
  assert.ok(blockBody >= 0, 'block body exists');
  const blockMocap = (model.body_mocapid as Int32Array)[blockBody];
  assert.ok(blockMocap >= 0, 'block is a mocap body');
  assert.ok(mj.mj_name2id(model, 6, 'graspframe') >= 0, 'graspframe site survived injection');
  const blockGeom = mj.mj_name2id(model, 5 /* mjOBJ_GEOM */, 'block_geom');
  assert.ok(blockGeom >= 0, 'block_geom exists');

  const pos = interiorToSim(90, 90); // board interior centre-ish
  const mp = data.mocap_pos as Float64Array;
  mp[blockMocap * 3] = pos[0];
  mp[blockMocap * 3 + 1] = pos[1];
  mp[blockMocap * 3 + 2] = pos[2];
  mj.mj_forward(model, data);

  const gx = data.geom_xpos as Float64Array;
  const at: [number, number, number] = [gx[blockGeom * 3], gx[blockGeom * 3 + 1], gx[blockGeom * 3 + 2]];
  for (let i = 0; i < 3; i++) {
    assert.ok(Math.abs(at[i] - pos[i]) < 1e-6, `block axis ${i} follows mocap`);
  }
});
