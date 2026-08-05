// solvePnP-based marker pose — the "3D position from a single ArUco marker"
// alternative to the homography. Used by the detection-noise logger to compare
// jitter against the homography method.
//
// NOTE: without a real intrinsic calibration this uses an *approximate* pinhole
// camera (focal ≈ image width, principal point at centre, no distortion). That's
// fine for measuring RELATIVE jitter (std dev), but the absolute mm values —
// especially depth — are only as good as that guess.

import type { Cv } from './cv';
import type { Corners } from './geometry';
import type { Intrinsics } from './charuco';

/**
 * Estimate the marker's 3D position (camera frame, same units as `markerMm`).
 * `corners` are the detected ArUco corners [TL, TR, BR, BL] in image pixels.
 * If `intr` is given, its calibrated camera matrix + distortion are used;
 * otherwise an approximate pinhole (focal ≈ image width, no distortion) is used.
 * Returns [x, y, z] or null if the solve fails.
 */
export function solvePnpTvec(
  cv: Cv,
  corners: Corners,
  markerMm: number,
  imgW: number,
  imgH: number,
  intr: Intrinsics | null = null,
): [number, number, number] | null {
  const h = markerMm / 2;
  // Object points in the marker frame (z=0), matching corner order [TL,TR,BR,BL].
  const objectPoints = cv.matFromArray(4, 1, cv.CV_32FC3, [
    -h, h, 0,
    h, h, 0,
    h, -h, 0,
    -h, -h, 0,
  ]);
  const imagePoints = cv.matFromArray(4, 1, cv.CV_32FC2, [
    corners[0][0], corners[0][1],
    corners[1][0], corners[1][1],
    corners[2][0], corners[2][1],
    corners[3][0], corners[3][1],
  ]);
  const { cameraMatrix, distCoeffs } = camMats(cv, intr, imgW, imgH);
  const rvec = new cv.Mat();
  const tvec = new cv.Mat();
  try {
    const flag = cv.SOLVEPNP_IPPE_SQUARE ?? 0;
    const ok = cv.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, flag);
    if (!ok) return null;
    const t = tvec.data64F as Float64Array;
    return [t[0], t[1], t[2]];
  } catch {
    return null;
  } finally {
    objectPoints.delete();
    imagePoints.delete();
    cameraMatrix.delete();
    distCoeffs.delete();
    rvec.delete();
    tvec.delete();
  }
}

export interface Pose {
  R: number[]; // row-major 3×3, maps board→camera: p_cam = R·p_board + t
  t: number[]; // 3
}

/** Full pose of a single square marker (marker→camera). */
export function solvePnpMarkerPose(
  cv: Cv,
  corners: Corners,
  markerMm: number,
  imgW: number,
  imgH: number,
  intr: Intrinsics | null,
): Pose | null {
  const h = markerMm / 2;
  const objectPoints = cv.matFromArray(4, 1, cv.CV_32FC3, [-h, h, 0, h, h, 0, h, -h, 0, -h, -h, 0]);
  const imagePoints = cv.matFromArray(4, 1, cv.CV_32FC2, [
    corners[0][0], corners[0][1], corners[1][0], corners[1][1],
    corners[2][0], corners[2][1], corners[3][0], corners[3][1],
  ]);
  const { cameraMatrix, distCoeffs } = camMats(cv, intr, imgW, imgH);
  const rvec = new cv.Mat();
  const tvec = new cv.Mat();
  const R = new cv.Mat();
  try {
    const flag = cv.SOLVEPNP_IPPE_SQUARE ?? 0;
    const ok = cv.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, flag);
    if (!ok) return null;
    cv.Rodrigues(rvec, R);
    return {
      R: Array.from(R.data64F as Float64Array),
      t: Array.from(tvec.data64F as Float64Array),
    };
  } catch {
    return null;
  } finally {
    objectPoints.delete();
    imagePoints.delete();
    cameraMatrix.delete();
    distCoeffs.delete();
    rvec.delete();
    tvec.delete();
    R.delete();
  }
}

/** 3×3 row-major transpose. */
export function transpose3(A: number[]): number[] {
  return [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
}

/** 3×3 row-major matrix product A·B. */
export function matMul3(A: number[], B: number[]): number[] {
  const C = new Array(9).fill(0);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) {
      let s = 0;
      for (let k = 0; k < 3; k++) s += A[i * 3 + k] * B[k * 3 + j];
      C[i * 3 + j] = s;
    }
  return C;
}

/** Apply a 3×3 row-major matrix to a 3-vector. */
export function apply3(A: number[], v: [number, number, number]): [number, number, number] {
  return [
    A[0] * v[0] + A[1] * v[1] + A[2] * v[2],
    A[3] * v[0] + A[4] * v[1] + A[5] * v[2],
    A[6] * v[0] + A[7] * v[1] + A[8] * v[2],
  ];
}

function camMats(cv: Cv, intr: Intrinsics | null, imgW: number, imgH: number) {
  let cameraMatrix;
  if (intr) {
    // Intrinsics are resolution-dependent — scale to the live image size in case
    // the current camera stream differs from the one calibration ran at.
    const sx = imgW / intr.imgW;
    const sy = imgH / intr.imgH;
    const m = intr.cameraMatrix;
    cameraMatrix = cv.matFromArray(3, 3, cv.CV_64F, [
      m[0] * sx, 0, m[2] * sx,
      0, m[4] * sy, m[5] * sy,
      0, 0, 1,
    ]);
  } else {
    cameraMatrix = cv.matFromArray(3, 3, cv.CV_64F, [imgW, 0, imgW / 2, 0, imgW, imgH / 2, 0, 0, 1]);
  }
  const distCoeffs =
    intr && intr.distCoeffs.length
      ? cv.matFromArray(1, intr.distCoeffs.length, cv.CV_64F, intr.distCoeffs)
      : cv.matFromArray(1, 5, cv.CV_64F, [0, 0, 0, 0, 0]);
  return { cameraMatrix, distCoeffs };
}

/**
 * Board pose (board→camera) from all detected border tags. `objFlat` is the tag
 * corners in board-mm (z=0), `imgFlat` the matching image-pixel corners.
 */
export function solvePnpPose(
  cv: Cv,
  objFlat: number[],
  imgFlat: number[],
  intr: Intrinsics,
  imgW: number,
  imgH: number,
): Pose | null {
  const n = objFlat.length / 3;
  if (n < 4) return null;
  const objectPoints = cv.matFromArray(n, 1, cv.CV_32FC3, objFlat);
  const imagePoints = cv.matFromArray(n, 1, cv.CV_32FC2, imgFlat);
  const { cameraMatrix, distCoeffs } = camMats(cv, intr, imgW, imgH);
  const rvec = new cv.Mat();
  const tvec = new cv.Mat();
  const R = new cv.Mat();
  try {
    const ok = cv.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
    if (!ok) return null;
    cv.Rodrigues(rvec, R);
    return {
      R: Array.from(R.data64F as Float64Array),
      t: Array.from(tvec.data64F as Float64Array),
    };
  } catch {
    return null;
  } finally {
    objectPoints.delete();
    imagePoints.delete();
    cameraMatrix.delete();
    distCoeffs.delete();
    rvec.delete();
    tvec.delete();
    R.delete();
  }
}

/** A camera-frame point → board frame: p_board = Rᵀ (p_cam − t). */
export function cameraToBoard(pose: Pose, pCam: [number, number, number]): [number, number, number] {
  const { R, t } = pose;
  const d = [pCam[0] - t[0], pCam[1] - t[1], pCam[2] - t[2]];
  return [
    R[0] * d[0] + R[3] * d[1] + R[6] * d[2],
    R[1] * d[0] + R[4] * d[1] + R[7] * d[2],
    R[2] * d[0] + R[5] * d[1] + R[8] * d[2],
  ];
}

/** Mean and sample standard deviation of a list. */
export function meanStd(xs: number[]): { mean: number; std: number } {
  const n = xs.length;
  if (n === 0) return { mean: NaN, std: NaN };
  const mean = xs.reduce((a, b) => a + b, 0) / n;
  const varc = n > 1 ? xs.reduce((a, b) => a + (b - mean) ** 2, 0) / (n - 1) : 0;
  return { mean, std: Math.sqrt(varc) };
}
