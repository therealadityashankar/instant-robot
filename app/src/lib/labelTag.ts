// Labeled nav tag extraction — detects the tag pair (200 & 201) and reads
// the label/description text between them via homography + OCR (Tesseract.js)
// or by sending the warped crop to Gemini when an API key is available.
//
// Card layout (as printed by make_labeled_tags.py):
//
//   ┌──────────┬────────────────────┬──────────┐
//   │          │                    │          │
//   │  ArUco   │      LABEL        │  ArUco   │
//   │   200    │                   │   201    │
//   │          │────────────────────│          │
//   │          │   description     │          │
//   │          │     text          │          │
//   └──────────┴────────────────────┴──────────┘
//
// The markers' known corner positions define the geometry: the text region
// sits between the inner edges of the two markers, with label on the top
// half and description on the bottom half.

import type { Cv } from './cv';
import type { Corners } from './geometry';

// ── Constants matching make_labeled_tags.py ──────────────────────────────────
export const LEFT_NAV_ID = 200;
export const RIGHT_NAV_ID = 201;

// Card geometry ratios relative to marker size (from make_labeled_tags.py):
//   quiet zone = 12% of marker, text gap = ~10% of marker
// The text region starts at the right edge of the left marker's quiet zone
// and ends at the left edge of the right marker's quiet zone.
// Vertically, the text region is aligned with the markers.

/** Warp output size for each text region. */
const WARP_W = 400;
const WARP_H = 120;

/** Result from a single labeled tag detection. */
export interface LabeledTagInfo {
  /** Unique key for this tag instance — derived from label or position. */
  key: string;
  /** The extracted label text (uppercase). */
  label: string;
  /** The extracted description text. */
  description: string;
  /** Average position of the two markers in image pixels. */
  centerPx: [number, number];
  /** The left marker's corners (200). */
  leftCorners: Corners;
  /** The right marker's corners (201). */
  rightCorners: Corners;
  /** Raw cropped canvas of the label region (for Gemini fallback). */
  labelCanvas?: HTMLCanvasElement;
  /** Raw cropped canvas of the description region (for Gemini fallback). */
  descCanvas?: HTMLCanvasElement;
}

/**
 * Given detected ArUco corners, find all paired 200+201 tags and extract
 * the text region between each pair.
 *
 * A pair is matched when a tag 200 and a tag 201 are at roughly the same
 * vertical position and the 201 is to the right of the 200, with their
 * centres separated by a plausible distance (1.5–4× the marker size).
 */
export function findLabeledTagPairs(
  corners: Map<number, Corners>,
): Array<{ left: Corners; right: Corners }> {
  const c200 = corners.get(LEFT_NAV_ID);
  const c201 = corners.get(RIGHT_NAV_ID);
  if (!c200 || !c201) return [];

  const cx200 = (c200[0][0] + c200[1][0] + c200[2][0] + c200[3][0]) / 4;
  const cy200 = (c200[0][1] + c200[1][1] + c200[2][1] + c200[3][1]) / 4;
  const cx201 = (c201[0][0] + c201[1][0] + c201[2][0] + c201[3][0]) / 4;
  const cy201 = (c201[0][1] + c201[1][1] + c201[2][1] + c201[3][1]) / 4;

  // Identify which marker is visually on the left (smaller X) and right (larger X)
  const [left, right] = cx200 < cx201 ? [c200, c201] : [c201, c200];
  const lcx = Math.min(cx200, cx201);
  const rcx = Math.max(cx200, cx201);
  const dy = Math.abs(cy200 - cy201);

  // Marker size in pixels (average of left marker's width and height)
  const lw = Math.hypot(left[1][0] - left[0][0], left[1][1] - left[0][1]);
  const dx = rcx - lcx;

  // Right should be separated horizontally by 1.0–8× marker width and vertically aligned
  if (dx < lw * 1.0 || dx > lw * 8.0 || dy > lw * 1.2) return [];

  return [{ left, right }];
}

/**
 * Check if the detected tag pair is fully in frame and large enough for reliable OCR.
 */
export function isTagPairReadyForOcr(
  left: Corners,
  right: Corners,
  imgW: number,
  imgH: number,
  minMarkerWidthFrac = 0.055, // At least ~35-40px on 640p camera
): boolean {
  const margin = 6;
  for (const pt of [...left, ...right]) {
    if (pt[0] < margin || pt[0] > imgW - margin || pt[1] < margin || pt[1] > imgH - margin) {
      return false; // clipped by camera edge
    }
  }

  const lw = Math.hypot(left[1][0] - left[0][0], left[1][1] - left[0][1]);
  if (lw < imgW * minMarkerWidthFrac) {
    return false; // too far away / too small
  }

  const lcy = (left[0][1] + left[1][1] + left[2][1] + left[3][1]) / 4;
  const rcy = (right[0][1] + right[1][1] + right[2][1] + right[3][1]) / 4;
  if (Math.abs(rcy - lcy) > lw * 0.7) {
    return false; // too skewed
  }

  return true;
}

import type { Pt } from './geometry';

/**
 * Extract the 4 corners of the center region directly from the inner edges of both markers:
 * - 2 right-side points of the left marker -> [TL, BL]
 * - 2 left-side points of the right marker -> [TR, BR]
 *
 * Uses the baseline between marker centers so this works at ANY camera angle, tilt, or rotation.
 */
export function getInnerEdgeCorners(
  c200: Corners,
  c201: Corners,
): {
  fullCorners: [Pt, Pt, Pt, Pt]; // [TL, TR, BR, BL]
  labelCorners: [Pt, Pt, Pt, Pt];
  descCorners: [Pt, Pt, Pt, Pt];
} {
  // Center of marker 200 and marker 201
  const cx200 = (c200[0][0] + c200[1][0] + c200[2][0] + c200[3][0]) / 4;
  const cy200 = (c200[0][1] + c200[1][1] + c200[2][1] + c200[3][1]) / 4;
  const cx201 = (c201[0][0] + c201[1][0] + c201[2][0] + c201[3][0]) / 4;
  const cy201 = (c201[0][1] + c201[1][1] + c201[2][1] + c201[3][1]) / 4;

  // Decide which marker is on the visual left vs right
  const [leftTag, rightTag, cLeft, cRight] = cx200 < cx201
    ? [c200, c201, [cx200, cy200], [cx201, cy201]]
    : [c201, c200, [cx201, cy201], [cx200, cy200]];

  // Card baseline vector u (pointing from left tag to right tag)
  const dx = cRight[0] - cLeft[0];
  const dy = cRight[1] - cLeft[1];
  const len = Math.hypot(dx, dy) || 1;
  const ux = dx / len;
  const uy = dy / len;
  // Perpendicular upward vector v (perpendicular to u in image coords)
  const vx = uy;
  const vy = -ux;

  // On leftTag: find the 2 corners with largest projection along u (facing right toward the card center)
  const leftSorted = [...leftTag].map((p) => ({
    p,
    projU: (p[0] - cLeft[0]) * ux + (p[1] - cLeft[1]) * uy,
    projV: (p[0] - cLeft[0]) * vx + (p[1] - cLeft[1]) * vy,
  })).sort((a, b) => b.projU - a.projU);

  // The 2 rightmost corners of leftTag
  const [l1, l2] = [leftSorted[0], leftSorted[1]];
  const tl_raw: Pt = l1.projV >= l2.projV ? l1.p : l2.p;
  const bl_raw: Pt = l1.projV >= l2.projV ? l2.p : l1.p;

  // On rightTag: find the 2 corners with smallest projection along u (facing left toward the card center)
  const rightSorted = [...rightTag].map((p) => ({
    p,
    projU: (p[0] - cRight[0]) * ux + (p[1] - cRight[1]) * uy,
    projV: (p[0] - cRight[0]) * vx + (p[1] - cRight[1]) * vy,
  })).sort((a, b) => a.projU - b.projU);

  // The 2 leftmost corners of rightTag
  const [r1, r2] = [rightSorted[0], rightSorted[1]];
  const tr_raw: Pt = r1.projV >= r2.projV ? r1.p : r2.p;
  const br_raw: Pt = r1.projV >= r2.projV ? r2.p : r1.p;

  // Small quiet-zone inset (4%) to stay clear of black marker borders
  const inset = 0.04;
  const dxt = tr_raw[0] - tl_raw[0], dyt = tr_raw[1] - tl_raw[1];
  const dxb = br_raw[0] - bl_raw[0], dyb = br_raw[1] - bl_raw[1];

  const tl: Pt = [tl_raw[0] + inset * dxt, tl_raw[1] + inset * dyt];
  const tr: Pt = [tr_raw[0] - inset * dxt, tr_raw[1] - inset * dyt];
  const br: Pt = [br_raw[0] - inset * dxb, br_raw[1] - inset * dyb];
  const bl: Pt = [bl_raw[0] + inset * dxb, bl_raw[1] + inset * dyb];

  // Midpoints to divide top (label) and bottom (description)
  const ml: Pt = [(tl[0] + bl[0]) / 2, (tl[1] + bl[1]) / 2];
  const mr: Pt = [(tr[0] + br[0]) / 2, (tr[1] + br[1]) / 2];

  return {
    fullCorners: [tl, tr, br, bl],
    labelCorners: [tl, tr, mr, ml],
    descCorners: [ml, mr, br, bl],
  };
}

/**
 * Extract and rectanglify the center region between a paired 200/201 into flat images.
 */
export function warpTextRegion(
  cv: Cv,
  gray: any, // cv.Mat (grayscale source image)
  c200: Corners,
  c201: Corners,
): {
  fullMat: any;
  labelMat: any;
  descMat: any;
  fullCenterCorners: Pt[];
  labelCorners: Pt[];
  descCorners: Pt[];
} | null {
  const { fullCorners, labelCorners, descCorners } = getInnerEdgeCorners(c200, c201);

  const rectanglify = (corners: Pt[], outW: number, outH: number) => {
    const src = cv.matFromArray(4, 1, cv.CV_32FC2, [
      corners[0][0], corners[0][1],
      corners[1][0], corners[1][1],
      corners[2][0], corners[2][1],
      corners[3][0], corners[3][1],
    ]);
    const dst = cv.matFromArray(4, 1, cv.CV_32FC2, [
      0, 0, outW, 0, outW, outH, 0, outH,
    ]);
    const mat = new cv.Mat();
    try {
      const H_warp = cv.getPerspectiveTransform(src, dst);
      cv.warpPerspective(gray, mat, H_warp, new cv.Size(outW, outH));
      H_warp.delete();
      return mat;
    } finally {
      src.delete();
      dst.delete();
    }
  };

  try {
    const fullMat = rectanglify(fullCorners, WARP_W, WARP_H * 2);
    const labelMat = rectanglify(labelCorners, WARP_W, WARP_H);
    const descMat = rectanglify(descCorners, WARP_W, WARP_H);
    return {
      fullMat,
      labelMat,
      descMat,
      fullCenterCorners: fullCorners,
      labelCorners,
      descCorners,
    };
  } catch {
    return null;
  }
}

/** Convert a grayscale cv.Mat to a canvas for display or Gemini. */
export function matToCanvas(cv: Cv, mat: any): HTMLCanvasElement {
  const c = document.createElement('canvas');
  c.width = mat.cols;
  c.height = mat.rows;
  const rgba = new cv.Mat();
  cv.cvtColor(mat, rgba, cv.COLOR_GRAY2RGBA);
  const imgData = new ImageData(
    new Uint8ClampedArray(rgba.data),
    mat.cols,
    mat.rows,
  );
  rgba.delete();
  c.getContext('2d')!.putImageData(imgData, 0, 0);
  return c;
}

/** Convert a grayscale cv.Mat to a data URL (for display/logging). */
export function matToDataUrl(cv: Cv, mat: any): string {
  const c = matToCanvas(cv, mat);
  return c.toDataURL('image/png');
}

// ── Tesseract.js OCR ─────────────────────────────────────────────────────────

let tesseractWorker: any = null;
let tesseractReady = false;
let tesseractLoading = false;

/**
 * Lazily initialise the Tesseract.js worker. Called once on first use;
 * subsequent calls return immediately. Configured with PSM 6 (single uniform text block).
 */
export async function initTesseract(): Promise<void> {
  if (tesseractReady) return;
  if (tesseractLoading) {
    while (tesseractLoading && !tesseractReady) {
      await new Promise((r) => setTimeout(r, 50));
    }
    return;
  }
  tesseractLoading = true;
  try {
    const Tesseract = await import('tesseract.js');
    tesseractWorker = await Tesseract.createWorker('eng');
    await tesseractWorker.setParameters({
      tessedit_pageseg_mode: '6', // PSM_SINGLE_BLOCK: Assume a single uniform block of text
    });
    tesseractReady = true;
    console.log('[Tesseract.js] Initialized with PSM 6 successfully.');
  } catch (e) {
    console.warn('[Tesseract.js] Failed to load:', e);
  } finally {
    tesseractLoading = false;
  }
}

export interface OcrResult {
  text: string;
  rawText: string;
  confidence: number;
}

/**
 * OCR a cv.Mat using Tesseract.js.
 */
export async function ocrMat(cv: Cv, mat: any): Promise<OcrResult> {
  if (!tesseractReady) {
    await initTesseract();
  }
  if (!tesseractReady || !tesseractWorker) {
    return { text: '', rawText: '', confidence: 0 };
  }
  const canvas = matToCanvas(cv, mat);
  try {
    const res = await tesseractWorker.recognize(canvas);
    const rawText = (res.data?.text || '').trim();
    const confidence = Math.round(res.data?.confidence ?? 0);
    return {
      text: rawText,
      rawText,
      confidence,
    };
  } catch (err) {
    console.warn('[Tesseract.js] OCR recognition error:', err);
    return { text: '', rawText: '', confidence: 0 };
  }
}

export interface LabeledTagInfo {
  key: string;
  label: string;
  description: string;
  rawLabelText: string;
  rawDescText: string;
  confidence: number;
  centerPx: [number, number];
  leftCorners: Corners;
  rightCorners: Corners;
  labelCanvas?: HTMLCanvasElement;
  descCanvas?: HTMLCanvasElement;
  fullCanvas?: HTMLCanvasElement;
  labelDataUrl?: string;
  descDataUrl?: string;
  fullDataUrl?: string;
}

/**
 * Full pipeline: find a labeled tag pair, warp the text regions, OCR both.
 */
export async function extractLabeledTag(
  cv: Cv,
  corners: Map<number, Corners>,
  gray: any,
): Promise<LabeledTagInfo | null> {
  const c200 = corners.get(LEFT_NAV_ID);
  const c201 = corners.get(RIGHT_NAV_ID);
  if (!c200 || !c201) return null;

  const warped = warpTextRegion(cv, gray, c200, c201);
  if (!warped) return null;

  const { fullMat, labelMat, descMat } = warped;

  try {
    const threshLabel = new cv.Mat();
    const threshDesc = new cv.Mat();
    cv.threshold(labelMat, threshLabel, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU);
    cv.threshold(descMat, threshDesc, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU);

    // Try Otsu thresholded first; fallback to raw grayscale if needed
    let labelRes = await ocrMat(cv, threshLabel);
    if (!labelRes.text) {
      labelRes = await ocrMat(cv, labelMat);
    }

    let descRes = await ocrMat(cv, threshDesc);
    if (!descRes.text) {
      descRes = await ocrMat(cv, descMat);
    }

    const labelCanvas = matToCanvas(cv, threshLabel);
    const descCanvas = matToCanvas(cv, threshDesc);
    const fullCanvas = matToCanvas(cv, fullMat);

    const labelDataUrl = labelCanvas.toDataURL('image/png');
    const descDataUrl = descCanvas.toDataURL('image/png');
    const fullDataUrl = fullCanvas.toDataURL('image/png');

    threshLabel.delete();
    threshDesc.delete();

    const cx200 = (c200[0][0] + c200[1][0] + c200[2][0] + c200[3][0]) / 4;
    const cy200 = (c200[0][1] + c200[1][1] + c200[2][1] + c200[3][1]) / 4;
    const cx201 = (c201[0][0] + c201[1][0] + c201[2][0] + c201[3][0]) / 4;
    const cy201 = (c201[0][1] + c201[1][1] + c201[2][1] + c201[3][1]) / 4;

    const cleanLabel = labelRes.text.replace(/[^A-Za-z0-9 ]/g, '').trim().toUpperCase();
    const cleanDesc = descRes.text.replace(/[^A-Za-z0-9 ,.'!-]/g, '').trim();

    return {
      key: cleanLabel || `tag_${Math.round(cx200)}_${Math.round(cy200)}`,
      label: cleanLabel,
      description: cleanDesc,
      rawLabelText: labelRes.rawText,
      rawDescText: descRes.rawText,
      confidence: labelRes.confidence,
      centerPx: [(cx200 + cx201) / 2, (cy200 + cy201) / 2],
      leftCorners: c200,
      rightCorners: c201,
      labelCanvas,
      descCanvas,
      fullCanvas,
      labelDataUrl,
      descDataUrl,
      fullDataUrl,
    };
  } finally {
    fullMat.delete();
    labelMat.delete();
    descMat.delete();
  }
}

/**
 * Check whether a set of detected corners contains a labeled nav tag pair.
 * This is a fast check (no OCR) — use it to decide whether to run the
 * full extraction pipeline.
 */
export function hasLabeledTagPair(corners: Map<number, Corners>): boolean {
  return findLabeledTagPairs(corners).length > 0;
}
