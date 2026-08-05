// solvePose (full-pose IK) must drive the gripper's approach axis to point
// straight down while reaching the target position — the top-down grasp used in
// the sim-only physics pick.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, readdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { loadMujocoModule, mountModel, type ModelFiles } from '../src/lib/mujocoSession';
import { buildBoardSceneXml } from '../src/lib/boardSim';
import { IKSolver } from '../src/lib/ik';

const here = dirname(fileURLToPath(import.meta.url));
const SO101 = join(here, '..', 'public', 'models', 'so101');

test('solvePose reaches position and points the gripper down', async () => {
  const mj = await loadMujocoModule();
  const armXml = readFileSync(join(SO101, 'so101_new_calib.xml'), 'utf8');
  const files: ModelFiles = { 'scene.xml': buildBoardSceneXml(armXml, { physics: true }) };
  for (const f of readdirSync(join(SO101, 'assets'))) {
    if (f.endsWith('.stl')) files[`assets/${f}`] = new Uint8Array(readFileSync(join(SO101, 'assets', f)));
  }
  const { model, data } = mountModel(mj, files, 'scene.xml');
  const solver = new IKSolver(mj, model, data, 'graspframe');

  const target: [number, number, number] = [0.2, 0, 0.11];
  const Rd = [0, 0, 1, 0, 1, 0, -1, 0, 0]; // approach (local X) → world -Z
  const res = solver.solvePose(target, Rd, { dofIndices: [0, 1, 2, 3, 4], maxIters: 200 });
  mj.mj_forward(model, data);

  assert.ok(res.error < 0.005, `position within 5mm, got ${(res.error * 1000).toFixed(1)}mm`);

  // Approach axis = local X of the grasp site (first column of its world rotation).
  const sid = mj.mj_name2id(model, 6, 'graspframe');
  const m = data.site_xmat as Float64Array;
  const o = sid * 9;
  const approachZ = m[o + 6]; // world-z component of local X axis
  assert.ok(approachZ < -0.9, `gripper points down (approach z ${approachZ.toFixed(2)} ≈ -1)`);
});
