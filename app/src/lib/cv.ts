// Minimal loader + typing shim for OpenCV.js, which is loaded from a <script>
// tag in index.html and attaches a global `cv`. OpenCV.js has no official types,
// so we treat the module as `any` and only wrap the ready handshake.

// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type Cv = any;

declare global {
  interface Window {
    cv?: Cv;
  }
}

let readyPromise: Promise<Cv> | null = null;

/** Resolve once OpenCV.js has finished initialising its WASM runtime. */
export function loadCv(): Promise<Cv> {
  if (readyPromise) return readyPromise;

  readyPromise = new Promise((resolve, reject) => {
    const deadline = Date.now() + 30_000;

    const check = () => {
      const cv = window.cv;
      // OpenCV.js sets `cv.Mat` only after the runtime is fully initialised.
      if (cv && cv.Mat) {
        resolve(cv);
        return;
      }
      if (cv && typeof cv.then === 'function') {
        // Some builds expose a promise-like module object.
        cv.then((real: Cv) => resolve(real)).catch(reject);
        return;
      }
      if (Date.now() > deadline) {
        reject(new Error('OpenCV.js failed to load within 30s'));
        return;
      }
      setTimeout(check, 50);
    };

    check();
  });

  return readyPromise;
}
