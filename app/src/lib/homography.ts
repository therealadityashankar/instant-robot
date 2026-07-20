// OpenCV.js wrappers for ArUco detection, homography and point transforms.
// Ports compute_homography / tag_interior_pos and the detector setup from
// calibrate_board.py.

import type { Cv } from './cv';
import type { Corners, Pt } from './geometry';
import { OUT_W, type TagCentres } from './board';

export interface Detector {
  detector: any;
  dictionary: any;
  params: any;
}

/** Build the ArUco detector with the tuned parameters from the Python script. */
export function createDetector(cv: Cv): Detector {
  const dictionary = cv.getPredefinedDictionary(cv.DICT_6X6_250);
  const params = new cv.aruco_DetectorParameters();
  // Slightly more aggressive detection (matches ARUCO_PARAMS in Python).
  params.adaptiveThreshWinSizeMin = 3;
  params.adaptiveThreshWinSizeMax = 53;
  params.adaptiveThreshWinSizeStep = 4;
  params.errorCorrectionRate = 0.7;
  const refine = new cv.aruco_RefineParameters(10, 3, true);
  const detector = new cv.aruco_ArucoDetector(dictionary, params, refine);
  return { detector, dictionary, params };
}

/** Detect markers in a grayscale Mat. Returns {tagId -> corners [TL,TR,BR,BL]}. */
export function detectMarkers(
  cv: Cv,
  det: Detector,
  gray: any,
): Map<number, Corners> {
  const cornersVec = new cv.MatVector();
  const ids = new cv.Mat();
  const rejected = new cv.MatVector();
  const out = new Map<number, Corners>();
  try {
    det.detector.detectMarkers(gray, cornersVec, ids, rejected);
    // Drive off the corner vector's size; ids may be laid out N×1 or 1×N, so
    // read it from its flat typed array rather than assuming a shape.
    const n = cornersVec.size();
    const idData: Int32Array | undefined = ids.data32S;
    for (let i = 0; i < n; i++) {
      const m = cornersVec.get(i); // 1x4 CV_32FC2
      if (!m) continue;
      const id = idData ? idData[i] : ids.intAt(i, 0);
      const d = m.data32F;
      const corners: Corners = [
        [d[0], d[1]],
        [d[2], d[3]],
        [d[4], d[5]],
        [d[6], d[7]],
      ];
      out.set(id, corners);
      m.delete();
    }
  } finally {
    cornersVec.delete();
    ids.delete();
    rejected.delete();
  }
  return out;
}

export interface HomographyResult {
  /** 3x3 homography Mat (caller owns it and must delete), or null. */
  H: any | null;
  inliers: number;
}

/**
 * Build src (image px) -> dst (mm-on-board scaled to output px) correspondences
 * and solve for the homography. Returns null H if fewer than 4 known tags.
 */
export function computeHomography(
  cv: Cv,
  cornersDict: Map<number, Corners>,
  tagCentresMm: TagCentres,
  squareMm: number,
): HomographyResult {
  const src: number[] = [];
  const dst: number[] = [];
  const scale = OUT_W / squareMm;

  for (const [tagId, corners] of cornersDict) {
    const centre = tagCentresMm.get(tagId);
    if (!centre) continue;
    const cxImg = (corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]) / 4;
    const cyImg = (corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]) / 4;
    src.push(cxImg, cyImg);
    dst.push(centre[0] * scale, centre[1] * scale);
  }

  const nPts = src.length / 2;
  if (nPts < 4) return { H: null, inliers: nPts };

  const srcMat = cv.matFromArray(nPts, 1, cv.CV_32FC2, src);
  const dstMat = cv.matFromArray(nPts, 1, cv.CV_32FC2, dst);
  const mask = new cv.Mat();
  let H: any | null = null;
  let inliers = 0;
  try {
    H = cv.findHomography(srcMat, dstMat, cv.RANSAC, 5.0, mask);
    if (H.empty()) {
      H.delete();
      H = null;
    } else {
      for (let i = 0; i < mask.rows; i++) inliers += mask.ucharAt(i, 0);
    }
  } finally {
    srcMat.delete();
    dstMat.delete();
    mask.delete();
  }
  return { H, inliers };
}

/** Apply a homography to a list of image points, returning transformed points. */
export function perspectiveTransformPts(cv: Cv, pts: Pt[], H: any): Pt[] {
  const flat: number[] = [];
  for (const p of pts) flat.push(p[0], p[1]);
  const srcMat = cv.matFromArray(pts.length, 1, cv.CV_32FC2, flat);
  const dstMat = new cv.Mat();
  try {
    cv.perspectiveTransform(srcMat, dstMat, H);
    const d = dstMat.data32F;
    const out: Pt[] = [];
    for (let i = 0; i < pts.length; i++) out.push([d[i * 2], d[i * 2 + 1]]);
    return out;
  } finally {
    srcMat.delete();
    dstMat.delete();
  }
}

/**
 * Return the TL corner (xInt, yInt) of the ArUco tag in interior mm.
 * Port of tag_interior_pos.
 */
export function tagInteriorPos(
  cv: Cv,
  corners: Corners,
  H: any,
  scale: number,
  interiorInsetMm: number,
): [number, number] {
  const [p] = perspectiveTransformPts(cv, [corners[0]], H);
  return [p[0] / scale - interiorInsetMm, p[1] / scale - interiorInsetMm];
}
