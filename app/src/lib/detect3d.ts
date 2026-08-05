// solvePnP-based block localisation (replaces the homography path).
//
// Board border tags → camera→board pose (solvePnP). Block tag → camera-frame 3D
// (solvePnP) → transformed into the board frame. We take the block's board-plane
// XY (Z is the block sitting on the board); parallax is handled natively by the
// full 3D pose, so no linear correction is needed.

import type { Cv } from './cv';
import type { Corners } from './geometry';
import type { TagCentres } from './board';
import type { Intrinsics } from './charuco';
import { solvePnpPose, solvePnpMarkerPose, cameraToBoard, type Pose } from './pose';

/** Estimate the board pose (board→camera) from all detected border tags. */
export function boardPoseFromTags(
  cv: Cv,
  cornersDict: Map<number, Corners>,
  tagCentresMm: TagCentres,
  tagMm: number,
  intr: Intrinsics,
  imgW: number,
  imgH: number,
): Pose | null {
  const obj: number[] = [];
  const img: number[] = [];
  const h = tagMm / 2;
  for (const [id, c] of cornersDict) {
    const centre = tagCentresMm.get(id);
    if (!centre) continue;
    const [cx, cy] = centre;
    // Board-mm corners in detection order [TL, TR, BR, BL], z = 0.
    obj.push(cx - h, cy - h, 0, cx + h, cy - h, 0, cx + h, cy + h, 0, cx - h, cy + h, 0);
    for (let k = 0; k < 4; k++) img.push(c[k][0], c[k][1]);
  }
  return solvePnpPose(cv, obj, img, intr, imgW, imgH);
}

/** Block localisation result in the board frame. */
export interface BlockBoard {
  x: number; // board-mm
  y: number; // board-mm
  /**
   * In-plane orientation of the block (radians): the angle of the tag's local
   * +Y axis (the block's length direction) measured in the board XY plane from
   * the board +X axis. Lets the sim render the block at its real rotation.
   */
  yaw: number;
}

/**
 * Block position + in-plane orientation on the board via solvePnP + the board
 * pose.
 *
 * `tagOffsetMm` is the vector from the tag centre to the desired grasp point
 * (block centre) expressed in the TAG's own frame [along tag-X, along tag-Y].
 * Because it's applied through the tag's measured orientation, it rotates with
 * the block — so blocks at arbitrary rotations are handled correctly.
 * Returns null if the block pose can't be solved.
 */
export function blockBoardXY(
  cv: Cv,
  blockCorners: Corners,
  markerMm: number,
  intr: Intrinsics,
  imgW: number,
  imgH: number,
  boardPose: Pose,
  tagOffsetMm: [number, number] = [0, 0],
): BlockBoard | null {
  // Position (translation) from solvePnP is reliable even when the single-tag
  // *rotation* is ambiguous — use it for the tag centre and its board-frame
  // height. Orientation is recovered geometrically below.
  const marker = solvePnpMarkerPose(cv, blockCorners, markerMm, imgW, imgH, intr);
  if (!marker) return null;
  const tagBoard = cameraToBoard(boardPose, marker.t as [number, number, number]);
  const z0 = tagBoard[2]; // tag height above the board plane

  // Scaled pinhole intrinsics for the live resolution (matches camMats()).
  const m = intr.cameraMatrix;
  const sx = imgW / intr.imgW;
  const sy = imgH / intr.imgH;
  const fx = m[0] * sx, cx = m[2] * sx, fy = m[4] * sy, cy = m[5] * sy;
  const R = boardPose.R;
  const t = boardPose.t;
  const col3 = [R[2], R[5], R[8]]; // 3rd column of R (board→camera)
  const col3t = col3[0] * t[0] + col3[1] * t[1] + col3[2] * t[2];

  // Back-project a pixel onto the plane Z=z0 in the board frame (distortion
  // ignored — negligible over a tag near the image). Uses the well-constrained
  // BOARD pose, so it doesn't inherit the block tag's rotation ambiguity.
  const bp = (u: number, v: number): [number, number] => {
    const d: [number, number, number] = [(u - cx) / fx, (v - cy) / fy, 1];
    const col3d = col3[0] * d[0] + col3[1] * d[1] + col3[2] * d[2];
    const lambda = (z0 + col3t) / col3d;
    const pb = cameraToBoard(boardPose, [lambda * d[0], lambda * d[1], lambda * d[2]]);
    return [pb[0], pb[1]];
  };

  // Corner order [TL, TR, BR, BL]. Tag's board-plane axes:
  //   +X along TL→TR, +Y along BL→TL (image-up).
  const TL = bp(blockCorners[0][0], blockCorners[0][1]);
  const TR = bp(blockCorners[1][0], blockCorners[1][1]);
  const BL = bp(blockCorners[3][0], blockCorners[3][1]);
  const ax: [number, number] = [TR[0] - TL[0], TR[1] - TL[1]];
  const ay: [number, number] = [TL[0] - BL[0], TL[1] - BL[1]];
  const nx = Math.hypot(ax[0], ax[1]) || 1;
  const ny = Math.hypot(ay[0], ay[1]) || 1;
  const ux: [number, number] = [ax[0] / nx, ax[1] / nx];
  const uy: [number, number] = [ay[0] / ny, ay[1] / ny];
  const yaw = Math.atan2(uy[1], uy[0]); // angle of tag +Y in the board plane

  const [ox, oy] = tagOffsetMm;
  return {
    x: tagBoard[0] + ux[0] * ox + uy[0] * oy,
    y: tagBoard[1] + ux[1] * ox + uy[1] * oy,
    yaw,
  };
}
