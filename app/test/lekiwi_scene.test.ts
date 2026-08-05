import { test } from 'node:test';
import { readFileSync, readdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { loadMujocoModule, mountModel, type ModelFiles } from '../src/lib/mujocoSession';
import { buildBoardSceneXml } from '../src/lib/boardSim';
import { injectBase, type BaseManifest } from '../src/lib/robots';

const here = dirname(fileURLToPath(import.meta.url));
const SO101 = join(here, '..', 'public', 'models', 'so101');
const LEKIWI = join(here, '..', 'public', 'models', 'lekiwi');

test('LeKiwi scene compiles', async () => {
  const mj = await loadMujocoModule();
  let armXml = readFileSync(join(SO101, 'so101_new_calib.xml'), 'utf8');
  // Mirror the app: on mobile robots the arm root is a mocap body.
  armXml = armXml.replace('<body name="base" pos="0 0 0"', '<body name="base" mocap="true" pos="0 0 0"');
  const manifest = JSON.parse(readFileSync(join(LEKIWI, 'base.json'), 'utf8')) as BaseManifest;
  const scene = injectBase(buildBoardSceneXml(armXml), manifest);

  const files: ModelFiles = { 'scene.xml': scene };
  for (const f of readdirSync(join(SO101, 'assets'))) {
    if (f.endsWith('.stl')) files[`assets/${f}`] = new Uint8Array(readFileSync(join(SO101, 'assets', f)));
  }
  for (const m of manifest.meshes) {
    files[`assets/${m.file}`] = new Uint8Array(readFileSync(join(LEKIWI, 'assets', m.file)));
  }
  try {
    mountModel(mj, files, 'scene.xml');
    console.log('OK: compiled');
  } catch (e) {
    console.log('COMPILE ERROR:', e instanceof Error ? e.message : String(e));
    throw e;
  }
});
