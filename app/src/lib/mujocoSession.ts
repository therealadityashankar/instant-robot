// MuJoCo WASM session helper — loads the module, mounts a model + its assets
// into the Emscripten FS, and holds the model/data pair. The same session backs
// both the IK solver and the three.js renderer, and runs identically in the
// browser and in Node (only the file bytes are sourced differently), which is
// what makes the IK headlessly testable.

// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type Mj = any;
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type MjModel = any;
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type MjData = any;

/** Files to mount: keys are FS-relative paths (e.g. "model.xml", "assets/x.stl"). */
export type ModelFiles = Record<string, Uint8Array | string>;

let modulePromise: Promise<Mj> | null = null;

/**
 * Load (and cache) the MuJoCo WASM module.
 *
 * We use the single-threaded `mujoco-js` build (the pthread build busy-spins the
 * browser's main thread — it can't `Atomics.wait` — pegging the CPU). Its wasm is
 * embedded in the one `.js` file, so there is no separate asset to locate.
 *
 * In the browser we load it as a static asset from `/mujoco/` at runtime; the URL
 * is built at runtime so Vite's static import analysis never sees a `/public`
 * path (importing one by literal is an error). Under Node (tests) we resolve the
 * npm package by name. The specifier is kept opaque to Vite via `@vite-ignore`.
 */
export function loadMujocoModule(): Promise<Mj> {
  if (!modulePromise) {
    const inBrowser = typeof window !== 'undefined';
    const specifier = inBrowser
      ? new URL('mujoco/mujoco_wasm.js', window.location.origin + '/').href
      : 'mujoco-js';
    modulePromise = import(/* @vite-ignore */ specifier).then((m) => m.default());
  }
  return modulePromise;
}

function mkdirp(mj: Mj, dir: string) {
  const parts = dir.split('/').filter(Boolean);
  let path = '';
  for (const p of parts) {
    path += '/' + p;
    try {
      mj.FS.mkdir(path);
    } catch {
      /* already exists */
    }
  }
}

let mountCounter = 0;

export interface Session {
  mj: Mj;
  model: MjModel;
  data: MjData;
  forward(): void;
  dispose(): void;
}

/**
 * Mount `files` under a fresh FS directory and load the named XML model.
 * `xmlName` must be one of the keys in `files`.
 */
export function mountModel(mj: Mj, files: ModelFiles, xmlName: string): Session {
  const dir = `/model_${mountCounter++}`;
  mkdirp(mj, dir);
  for (const [rel, bytes] of Object.entries(files)) {
    const full = `${dir}/${rel}`;
    const slash = full.lastIndexOf('/');
    if (slash > 0) mkdirp(mj, full.slice(0, slash));
    mj.FS.writeFile(full, bytes);
  }
  // Model loader name differs across builds: `loadFromXML` (mujoco-js) vs
  // `mj_loadXML` (official pthread build).
  const loader = mj.MjModel.loadFromXML ?? mj.MjModel.mj_loadXML;
  const model = loader.call(mj.MjModel, `${dir}/${xmlName}`);
  const data = new mj.MjData(model);
  mj.mj_forward(model, data);

  return {
    mj,
    model,
    data,
    forward: () => mj.mj_forward(model, data),
    dispose: () => {
      try {
        data.delete();
        model.delete();
      } catch {
        /* ignore */
      }
    },
  };
}

/** Fetch a model + its assets over HTTP (browser). Returns a ModelFiles map. */
export async function fetchModelFiles(
  baseUrl: string,
  xmlName: string,
  assetNames: string[],
): Promise<ModelFiles> {
  const files: ModelFiles = {};
  const xml = await fetch(`${baseUrl}/${xmlName}`);
  if (!xml.ok) throw new Error(`Failed to fetch ${xmlName}: ${xml.status}`);
  files[xmlName] = new Uint8Array(await xml.arrayBuffer());
  for (const name of assetNames) {
    const res = await fetch(`${baseUrl}/assets/${name}`);
    if (!res.ok) throw new Error(`Failed to fetch asset ${name}: ${res.status}`);
    files[`assets/${name}`] = new Uint8Array(await res.arrayBuffer());
  }
  return files;
}
