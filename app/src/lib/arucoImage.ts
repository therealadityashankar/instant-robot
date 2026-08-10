// Draw ArUco markers as images, so the simulation can show the *same* fiducials
// the real robot looks at.
//
// Why bother: every sim-only bug in this project has come from having a second,
// approximate implementation of something the real robot does properly. Painting
// real markers into the scene and running the real detector over the rendered
// frame collapses those two paths into one — what the sim camera sees is decided
// by geometry and pixels, not by a hand-written visibility test that can quietly
// disagree with the renderer.
//
// The patterns come from OpenCV's own `generateImageMarker`, on the dictionary
// object. Decoding `bytesList` by hand instead looks easy and isn't: the bit
// packing is not what it appears, and a wrong pattern is invisible — it renders
// as a perfectly plausible marker that simply never decodes.

import type { Cv } from './cv';

/**
 * A marker rendered exactly as OpenCV would generate it: `sidePx` square,
 * including `borderBits` cells of black border.
 */
export function markerCanvas(
  cv: Cv,
  dictId: number,
  id: number,
  sidePx = 256,
  borderBits = 1,
): HTMLCanvasElement {
  const dict = cv.getPredefinedDictionary(dictId);
  const mat = new cv.Mat();
  try {
    dict.generateImageMarker(id, sidePx, mat, borderBits);
    const w = mat.cols;
    const h = mat.rows;
    const gray = mat.data as Uint8Array; // CV_8UC1
    const c = document.createElement('canvas');
    c.width = w;
    c.height = h;
    const ctx = c.getContext('2d')!;
    const img = ctx.createImageData(w, h);
    for (let i = 0; i < w * h; i++) {
      const v = gray[i];
      img.data[i * 4] = v;
      img.data[i * 4 + 1] = v;
      img.data[i * 4 + 2] = v;
      img.data[i * 4 + 3] = 255;
    }
    ctx.putImageData(img, 0, 0);
    return c;
  } finally {
    mat.delete();
  }
}

/**
 * A marker on a white backing sheet, the way a printed tag actually looks. The
 * quiet zone matters: the detector needs light around the black border to find
 * the square at all.
 */
export function printedMarkerCanvas(
  cv: Cv,
  dictId: number,
  id: number,
  sidePx = 256,
  quietFrac = 0.18,
): HTMLCanvasElement {
  const inner = markerCanvas(cv, dictId, id, sidePx);
  const pad = Math.round(inner.width * quietFrac);
  const c = document.createElement('canvas');
  c.width = inner.width + pad * 2;
  c.height = inner.height + pad * 2;
  const ctx = c.getContext('2d')!;
  ctx.imageSmoothingEnabled = false;
  ctx.fillStyle = '#fff';
  ctx.fillRect(0, 0, c.width, c.height);
  ctx.drawImage(inner, pad, pad);
  return c;
}
