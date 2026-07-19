// Loader + typing shim for OpenCV.js. We use @techstark/opencv-js, a build
// compiled *with* the contrib aruco module (the stock docs.opencv.org build
// omits it). It ships the WASM and initialises itself on import; we just wait
// for the runtime to finish.

// OpenCV.js has no official types, so we treat the module as `any`.
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type Cv = any;

import cvModule from '@techstark/opencv-js';

let readyPromise: Promise<Cv> | null = null;

/**
 * Resolve once OpenCV.js has finished initialising its WASM runtime.
 * The module export takes one of three shapes depending on the build, so we
 * handle all of them (per the @techstark/opencv-js README):
 *   - a Promise that resolves to the ready `cv` (OpenCV 5 builds),
 *   - an already-initialised module (`cv.Mat` present),
 *   - a module that fires `onRuntimeInitialized` when ready.
 */
export function loadCv(): Promise<Cv> {
  if (readyPromise) return readyPromise;

  const mod = cvModule as Cv;

  if (mod instanceof Promise) {
    readyPromise = mod.then((cv: Cv) => cv);
  } else if (mod && mod.Mat) {
    readyPromise = Promise.resolve(mod);
  } else {
    readyPromise = new Promise((resolve, reject) => {
      const timer = setTimeout(
        () => reject(new Error('OpenCV.js failed to initialise within 30s')),
        30_000,
      );
      mod.onRuntimeInitialized = () => {
        clearTimeout(timer);
        resolve(mod);
      };
    });
  }

  return readyPromise;
}
