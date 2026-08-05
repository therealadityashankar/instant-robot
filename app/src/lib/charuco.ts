// ChArUco camera-intrinsic calibration via OpenCV.js.
//
// Flow: build a CharucoBoard matching the printed PDF → per captured view,
// detect the board and match detected charuco corners to their 3D board points
// → accumulate views → calibrateCamera → {cameraMatrix, distCoeffs}.
//
// IMPORTANT: these WASM aruco/calib calls are not exercised in CI (no camera),
// so the exact binding shapes may need on-device tweaking.

import type { Cv } from './cv';

// Must match tags-and-borders/make_charuco_board.py.
export const CHARUCO = {
  squaresX: 7,
  squaresY: 5,
  squareMm: 25,
  markerMm: 18,
  dict: 'DICT_5X5_100',
} as const;

/** A matched set of 3D board points + 2D image points for one captured view. */
export interface CharucoView {
  obj: number[]; // flat [x,y,z, x,y,z, …] in mm
  img: number[]; // flat [x,y, x,y, …] in px
  n: number;
}

export interface Intrinsics {
  cameraMatrix: number[]; // row-major 3×3
  distCoeffs: number[];
  imgW: number;
  imgH: number;
  rms: number;
}

export function createCharucoBoard(cv: Cv): any {
  const dictionary = cv.getPredefinedDictionary(cv[CHARUCO.dict]);
  // The JS binding requires the marker-ids Mat explicitly (Python defaults it).
  // A ChArUco board has floor(squaresX*squaresY / 2) markers, ids 0..n-1 —
  // matching the default board the PDF generator used.
  const n = Math.floor((CHARUCO.squaresX * CHARUCO.squaresY) / 2);
  const ids = cv.matFromArray(n, 1, cv.CV_32S, Array.from({ length: n }, (_, i) => i));
  return new cv.aruco_CharucoBoard(
    new cv.Size(CHARUCO.squaresX, CHARUCO.squaresY),
    CHARUCO.squareMm,
    CHARUCO.markerMm,
    dictionary,
    ids,
  );
}

export function createCharucoDetector(cv: Cv, board: any): any {
  // JS binding requires all four args: board + charuco/detector/refine params.
  const charucoParams = new cv.aruco_CharucoParameters();
  const detectorParams = new cv.aruco_DetectorParameters();
  // Same aggressive marker-detection tuning as the bordered board — without this
  // the default thresholds miss most markers, so few/no charuco corners interpolate.
  detectorParams.adaptiveThreshWinSizeMin = 3;
  detectorParams.adaptiveThreshWinSizeMax = 53;
  detectorParams.adaptiveThreshWinSizeStep = 4;
  detectorParams.errorCorrectionRate = 0.7;
  const refineParams = new cv.aruco_RefineParameters(10, 3, true);
  return new cv.aruco_CharucoDetector(board, charucoParams, detectorParams, refineParams);
}

/**
 * Detect the ChArUco board in a grayscale frame and match to 3D board points.
 * Returns a view (obj/img points) or null if too few corners were found.
 */
export function detectView(cv: Cv, detector: any, board: any, gray: any): CharucoView | null {
  const corners = new cv.Mat();
  const ids = new cv.Mat();
  // 3D coords of every charuco corner, as a Point3fVector (get(i) → {x,y,z}).
  const allObj = board.getChessboardCorners();
  try {
    detector.detectBoard(gray, corners, ids);
    // OpenCV.js returns 1×N Mats, so count with total().
    const n = ids.total();
    if (n < 6) return null; // need a decent number of corners
    const cd = corners.data32F as Float32Array; // 2 per detected corner
    const idd = ids.data32S as Int32Array;
    const obj: number[] = [];
    const img: number[] = [];
    for (let i = 0; i < n; i++) {
      const id = idd[i]; // which board corner this detection is
      const p = allObj.get(id);
      obj.push(p.x, p.y, p.z ?? 0);
      img.push(cd[i * 2], cd[i * 2 + 1]);
    }
    return { obj, img, n };
  } catch {
    return null;
  } finally {
    corners.delete();
    ids.delete();
    allObj.delete?.();
  }
}

/** Run calibrateCamera over the accumulated views. */
export function calibrateIntrinsics(
  cv: Cv,
  views: CharucoView[],
  imgW: number,
  imgH: number,
): Intrinsics {
  const objVec = new cv.MatVector();
  const imgVec = new cv.MatVector();
  const pushed: any[] = [];
  const cameraMatrix = new cv.Mat();
  const distCoeffs = new cv.Mat();
  const rvecs = new cv.MatVector();
  const tvecs = new cv.MatVector();
  const stdI = new cv.Mat();
  const stdE = new cv.Mat();
  const perView = new cv.Mat();
  try {
    for (const v of views) {
      const om = cv.matFromArray(v.n, 1, cv.CV_32FC3, v.obj);
      const im = cv.matFromArray(v.n, 1, cv.CV_32FC2, v.img);
      objVec.push_back(om);
      imgVec.push_back(im);
      pushed.push(om, im);
    }
    const size = new cv.Size(imgW, imgH);
    // Only calibrateCameraExtended is bound in this build (not calibrateCamera).
    const criteria = new cv.TermCriteria(3, 30, 1e-6); // COUNT|EPS
    const rms = cv.calibrateCameraExtended(
      objVec, imgVec, size, cameraMatrix, distCoeffs, rvecs, tvecs,
      stdI, stdE, perView, 0, criteria,
    );
    return {
      cameraMatrix: Array.from(cameraMatrix.data64F as Float64Array),
      distCoeffs: Array.from(distCoeffs.data64F as Float64Array),
      imgW,
      imgH,
      rms,
    };
  } finally {
    pushed.forEach((m) => m.delete());
    objVec.delete();
    imgVec.delete();
    cameraMatrix.delete();
    distCoeffs.delete();
    rvecs.delete();
    tvecs.delete();
    stdI.delete();
    stdE.delete();
    perView.delete();
  }
}
