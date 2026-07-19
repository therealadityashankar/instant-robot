// Loader + typing shim for OpenCV.js. We use @techstark/opencv-js, a build
// compiled *with* the contrib aruco module (the stock docs.opencv.org build
// omits it). It ships the WASM and initialises itself on import; we just wait
// for the runtime to finish.

// OpenCV.js has no official types, so we treat the module as `any`.
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type Cv = any;

import cvModule from '@techstark/opencv-js';

let readyPromise: Promise<Cv> | null = null;

/** Resolve once OpenCV.js has finished initialising its WASM runtime. */
export function loadCv(): Promise<Cv> {
  if (readyPromise) return readyPromise;

  readyPromise = new Promise((resolve, reject) => {
    const cv = cvModule as Cv;
    const deadline = Date.now() + 30_000;

    const check = () => {
      // `cv.Mat` only becomes defined after the WASM runtime is initialised.
      if (cv && cv.Mat) {
        resolve(cv);
        return;
      }
      if (Date.now() > deadline) {
        reject(new Error('OpenCV.js failed to initialise within 30s'));
        return;
      }
      setTimeout(check, 50);
    };

    // Prefer the official hook when the runtime hasn't started yet.
    if (cv && !cv.Mat) {
      cv.onRuntimeInitialized = () => resolve(cv);
    }
    check();
  });

  return readyPromise;
}
