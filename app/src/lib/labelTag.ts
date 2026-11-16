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
export const ANCHOR_RIGHT_NAV_ID = 200;
export const RIGHT_NAV_ID = 200; // Universal right anchor marker on all cards
export const LEFT_NAV_ID = 201;  // Default left marker ID for single-station compatibility

// Card geometry ratios relative to marker size (from make_labeled_tags.py):
//   quiet zone = 12% of marker, text gap = ~10% of marker
// The text region starts at the right edge of the left marker's quiet zone
// and ends at the left edge of the right marker's quiet zone.
// Vertically, the text region is aligned with the markers.

/** Warp output size for each text region. */
const WARP_W = 400;
const WARP_H = 120;

export interface LabeledTagPair {
  /** The unique station ID from the left marker (e.g. 201, 202, 250, 301, etc.). */
  stationId: number;
  /** The left marker's corners (unique station ID). */
  left: Corners;
  /** The right marker's corners (anchor ID 200). */
  right: Corners;
  /** Center of left marker in image pixels. */
  leftCenter?: [number, number];
  /** Center of right anchor marker in image pixels. */
  rightCenter?: [number, number];
  /** Exact optical midpoint between left and right markers in image pixels. */
  centerPx?: [number, number];
}

/** Result from a single labeled tag detection. */
export interface LabeledTagInfo {
  /** The unique station ID from the left marker. */
  stationId: number;
  /** Unique key for this tag instance — derived from label or station ID. */
  key: string;
  /** The extracted label text (uppercase). */
  label: string;
  /** The extracted description text. */
  description: string;
  /** Raw label OCR text before sanitization. */
  rawLabelText: string;
  /** Raw description OCR text before sanitization. */
  rawDescText: string;
  /** OCR confidence score (0..100). */
  confidence: number;
  /** Average position of the two markers in image pixels. */
  centerPx: [number, number];
  /** The left marker's corners (unique station ID). */
  leftCorners: Corners;
  /** The right marker's corners (anchor ID 200). */
  rightCorners: Corners;
  /** Raw cropped canvas of the label region (for Gemini fallback). */
  labelCanvas?: HTMLCanvasElement;
  /** Raw cropped canvas of the description region (for Gemini fallback). */
  descCanvas?: HTMLCanvasElement;
  /** Full cropped card canvas. */
  fullCanvas?: HTMLCanvasElement;
  labelDataUrl?: string;
  descDataUrl?: string;
  fullDataUrl?: string;
}

/**
 * Given detected ArUco corners, find all paired station cards:
 * [Unique Left Tag ID (201..499)] + [Right Anchor Tag 200].
 *
 * Rules:
 * 1. The anchor tag 200 MUST be placed to the RIGHT of the station tag (dx = cx200 - cxLeft > 0).
 * 2. Any anchor tag 200 in the opposite direction (to the left, dx <= 0) is STRICTLY IGNORED.
 * 3. The pair must be separated by a plausible card distance (0.8–8.0× marker size) and vertically aligned.
 */
export function findLabeledTagPairs(
  cornersInput: Map<number, Corners> | { id: number; corners: Corners }[],
): LabeledTagPair[] {
  // Normalize input into an array of { id, corners }
  const detections: { id: number; corners: Corners }[] = Array.isArray(cornersInput)
    ? cornersInput
    : Array.from(cornersInput.entries()).map(([id, corners]) => ({ id, corners }));

  const anchorDetections = detections.filter((d) => d.id === ANCHOR_RIGHT_NAV_ID);
  const stationDetections = detections.filter((d) => d.id !== ANCHOR_RIGHT_NAV_ID);

  if (!anchorDetections.length || !stationDetections.length) return [];

  const pairs: LabeledTagPair[] = [];

  for (const s of stationDetections) {
    const cLeft = s.corners;
    const cxLeft = (cLeft[0][0] + cLeft[1][0] + cLeft[2][0] + cLeft[3][0]) / 4;
    const cyLeft = (cLeft[0][1] + cLeft[1][1] + cLeft[2][1] + cLeft[3][1]) / 4;
    const lw = Math.hypot(cLeft[1][0] - cLeft[0][0], cLeft[1][1] - cLeft[0][1]);

    // Find valid companion anchor 200 strictly to the RIGHT of this station tag
    let bestAnchor: { id: number; corners: Corners } | null = null;
    let bestDx = Infinity;

    for (const a of anchorDetections) {
      const cRight = a.corners;
      const cxRight = (cRight[0][0] + cRight[1][0] + cRight[2][0] + cRight[3][0]) / 4;
      const cyRight = (cRight[0][1] + cRight[1][1] + cRight[2][1] + cRight[3][1]) / 4;
      const rw = Math.hypot(cRight[1][0] - cRight[0][0], cRight[1][1] - cRight[0][1]);
      const avgW = (lw + rw) / 2;

      const dx = cxRight - cxLeft;
      const dy = Math.abs(cyRight - cyLeft);

      // Station Tag MUST be on the left, Anchor 200 MUST be on the right (dx > 0)
      if (dx >= avgW * 0.8 && dx <= avgW * 8.0 && dy <= avgW * 1.2) {
        if (dx < bestDx) {
          bestDx = dx;
          bestAnchor = a;
        }
      }
    }

    if (bestAnchor) {
      const cRight = bestAnchor.corners;
      const cxRight = (cRight[0][0] + cRight[1][0] + cRight[2][0] + cRight[3][0]) / 4;
      const cyRight = (cRight[0][1] + cRight[1][1] + cRight[2][1] + cRight[3][1]) / 4;
      pairs.push({
        stationId: s.id,
        left: cLeft,
        right: cRight,
        leftCenter: [cxLeft, cyLeft],
        rightCenter: [cxRight, cyRight],
        centerPx: [(cxLeft + cxRight) / 2, (cyLeft + cyRight) / 2],
      });
    }
  }

  // Sort pairs from left to right across the image
  pairs.sort((a, b) => (a.centerPx ? a.centerPx[0] : 0) - (b.centerPx ? b.centerPx[0] : 0));
  return pairs;
}

/**
 * Check whether a set of detected corners contains a labeled nav tag pair.
 */
export function hasLabeledTagPair(
  cornersInput: Map<number, Corners> | { id: number; corners: Corners }[],
): boolean {
  return findLabeledTagPairs(cornersInput).length > 0;
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
    const dsize = new cv.Size(outW, outH);
    try {
      const H = cv.getPerspectiveTransform(src, dst);
      cv.warpPerspective(gray, mat, H, dsize);
      H.delete();
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

/** Convert a grayscale cv.Mat to a canvas for display or OCR without leaking memory. */
export function matToCanvas(cv: Cv, mat: any): HTMLCanvasElement {
  const c = document.createElement('canvas');
  c.width = mat.cols;
  c.height = mat.rows;
  const rgba = new cv.Mat();
  try {
    cv.cvtColor(mat, rgba, cv.COLOR_GRAY2RGBA);
    const imgData = new ImageData(
      new Uint8ClampedArray(rgba.data),
      mat.cols,
      mat.rows,
    );
    c.getContext('2d')!.putImageData(imgData, 0, 0);
  } finally {
    rgba.delete();
  }
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

/** Cleanly terminate the Tesseract worker to free Web Worker memory. */
export async function terminateTesseract(): Promise<void> {
  if (tesseractWorker) {
    try {
      await tesseractWorker.terminate();
    } catch {
      /* ignore */
    }
    tesseractWorker = null;
    tesseractReady = false;
  }
}

export interface OcrResult {
  text: string;
  rawText: string;
  confidence: number;
}

/**
 * OCR an HTMLCanvasElement using Tesseract.js (no cv.Mat held in memory).
 */
export async function ocrCanvas(canvas: HTMLCanvasElement): Promise<OcrResult> {
  if (!tesseractReady) {
    await initTesseract();
  }
  if (!tesseractReady || !tesseractWorker) {
    return { text: '', rawText: '', confidence: 0 };
  }
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

/**
 * Legacy wrapper for ocrMat (converts to canvas and immediately disposes Mat).
 */
export async function ocrMat(cv: Cv, mat: any): Promise<OcrResult> {
  const canvas = matToCanvas(cv, mat);
  return ocrCanvas(canvas);
}

/**
 * Full pipeline: find a labeled tag pair, warp the text regions, OCR both.
 * All OpenCV Mats are deleted synchronously before running async Tesseract OCR.
 */
export async function extractLabeledTag(
  cv: Cv,
  corners: Map<number, Corners>,
  gray: any,
  targetStationId?: number,
): Promise<LabeledTagInfo | null> {
  const pairs = findLabeledTagPairs(corners);
  if (!pairs.length) return null;

  const pair = (targetStationId !== undefined ? pairs.find((p) => p.stationId === targetStationId) : null) || pairs[0];
  if (!pair) return null;

  const cLeft = pair.left;
  const cRight = pair.right;

  // Warp into clean Mats
  const warped = warpTextRegion(cv, gray, cLeft, cRight);
  if (!warped) return null;

  const { fullMat, labelMat, descMat } = warped;
  let fullCanvas: HTMLCanvasElement;
  let labelCanvas: HTMLCanvasElement;
  let descCanvas: HTMLCanvasElement;
  let threshLabelCanvas: HTMLCanvasElement;
  let threshDescCanvas: HTMLCanvasElement;

  try {
    const threshLabel = new cv.Mat();
    const threshDesc = new cv.Mat();
    try {
      cv.threshold(labelMat, threshLabel, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU);
      cv.threshold(descMat, threshDesc, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU);

      fullCanvas = matToCanvas(cv, fullMat);
      labelCanvas = matToCanvas(cv, labelMat);
      descCanvas = matToCanvas(cv, descMat);
      threshLabelCanvas = matToCanvas(cv, threshLabel);
      threshDescCanvas = matToCanvas(cv, threshDesc);
    } finally {
      threshLabel.delete();
      threshDesc.delete();
    }
  } finally {
    fullMat.delete();
    labelMat.delete();
    descMat.delete();
  }

  const labelDataUrl = threshLabelCanvas.toDataURL('image/png');
  const descDataUrl = threshDescCanvas.toDataURL('image/png');
  const fullDataUrl = fullCanvas.toDataURL('image/png');

  // Now run Tesseract OCR on pure JS canvases (0 OpenCV Mats held in memory)
  let labelRes = await ocrCanvas(threshLabelCanvas);
  if (!labelRes.text) {
    labelRes = await ocrCanvas(labelCanvas);
  }

  let descRes = await ocrCanvas(threshDescCanvas);
  if (!descRes.text) {
    descRes = await ocrCanvas(descCanvas);
  }

  const cxLeft = (cLeft[0][0] + cLeft[1][0] + cLeft[2][0] + cLeft[3][0]) / 4;
  const cyLeft = (cLeft[0][1] + cLeft[1][1] + cLeft[2][1] + cLeft[3][1]) / 4;
  const cxRight = (cRight[0][0] + cRight[1][0] + cRight[2][0] + cRight[3][0]) / 4;
  const cyRight = (cRight[0][1] + cRight[1][1] + cRight[2][1] + cRight[3][1]) / 4;

  const cleanLabel = labelRes.text.replace(/[^A-Za-z0-9 ]/g, '').trim().toUpperCase();
  const cleanDesc = descRes.text.replace(/[^A-Za-z0-9 ,.'!-]/g, '').trim();

  return {
    stationId: pair.stationId,
    key: cleanLabel || `station_${pair.stationId}`,
    label: cleanLabel,
    description: cleanDesc,
    rawLabelText: labelRes.rawText,
    rawDescText: descRes.rawText,
    confidence: labelRes.confidence,
    centerPx: [(cxLeft + cxRight) / 2, (cyLeft + cyRight) / 2],
    leftCorners: cLeft,
    rightCorners: cRight,
    labelCanvas: threshLabelCanvas,
    descCanvas: threshDescCanvas,
    fullCanvas,
    labelDataUrl,
    descDataUrl,
    fullDataUrl,
  };
}

