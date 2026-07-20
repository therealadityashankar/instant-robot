// Headless IK test — loads the real SO-101 model into MuJoCo WASM (no canvas,
// no browser) and checks that the Jacobian IK solver drives the grasp site to
// FK-generated targets. This is the testability payoff of routing IK through
// MuJoCo: mj_forward is an independent ground-truth oracle.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, readdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { loadMujocoModule, mountModel, type ModelFiles } from '../src/lib/mujocoSession';
import { IKSolver } from '../src/lib/ik';

const here = dirname(fileURLToPath(import.meta.url));
const MODEL_DIR = join(here, '..', '..', 'models', 'so101');
const XML = 'so101_simplified.xml';
const ARM_DOFS = [0, 1, 2, 3, 4]; // exclude gripper
const GRASP_SITE = 'graspframe';

function loadFiles(): ModelFiles {
  const files: ModelFiles = {};
  files[XML] = new Uint8Array(readFileSync(join(MODEL_DIR, XML)));
  for (const f of readdirSync(join(MODEL_DIR, 'assets'))) {
    if (f.endsWith('.stl')) {
      files[`assets/${f}`] = new Uint8Array(readFileSync(join(MODEL_DIR, 'assets', f)));
    }
  }
  return files;
}

test('IK reaches FK-generated targets within 1mm', async () => {
  const mj = await loadMujocoModule();
  const { model, data } = mountModel(mj, loadFiles(), XML);
  const solver = new IKSolver(mj, model, data, GRASP_SITE);
  const siteId = mj.mj_name2id(model, 6 /* mjOBJ_SITE */, GRASP_SITE);

  // A handful of reachable poses; take each as ground truth via FK, then solve.
  const poses = [
    [0.3, 0.4, 0.5, 0.2, 0.0],
    [-0.5, 0.3, 0.6, -0.2, 0.4],
    [0.8, 0.6, 0.3, 0.1, -0.3],
    [0.0, 0.2, 0.9, 0.5, 0.0],
  ];

  for (const pose of poses) {
    // FK: set the pose, forward, read the true site position.
    for (let i = 0; i < 6; i++) data.qpos[i] = i < 5 ? pose[i] : 0;
    mj.mj_forward(model, data);
    const sx = data.site_xpos as Float64Array;
    const target: [number, number, number] = [sx[siteId * 3], sx[siteId * 3 + 1], sx[siteId * 3 + 2]];

    // Reset arm, then solve back to the target.
    for (let i = 0; i < 6; i++) data.qpos[i] = 0;
    mj.mj_forward(model, data);
    const res = solver.solve(target, { dofIndices: ARM_DOFS });

    assert.ok(res.ok, `did not converge for pose ${pose}: err=${res.error}`);
    assert.ok(res.error < 1e-3, `error ${res.error * 1000}mm too large for pose ${pose}`);
  }

  solver.dispose();
});

test('IK reports failure for an unreachable target', async () => {
  const mj = await loadMujocoModule();
  const { model, data } = mountModel(mj, loadFiles(), XML);
  const solver = new IKSolver(mj, model, data, GRASP_SITE);

  for (let i = 0; i < 6; i++) data.qpos[i] = 0;
  mj.mj_forward(model, data);
  // Far outside the arm's ~0.4m reach.
  const res = solver.solve([5, 5, 5], { dofIndices: ARM_DOFS, maxIters: 100 });
  assert.equal(res.ok, false);

  solver.dispose();
});
