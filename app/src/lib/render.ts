// Canvas rendering — ports draw_raw_panel and draw_rectified_panel from
// calibrate_board.py. OpenCV uses BGR tuples; the CSS colours below are the
// RGB equivalents.

import type { Cv } from './cv';
import type { Corners, Pt } from './geometry';
import { borderedCornersImage } from './geometry';
import { perspectiveTransformPts } from './homography';
import {
  OUT_W,
  OUT_H,
  BORDERED_IDS,
  CALIB_CORNER_NAMES,
  correctPos,
  type TagCentres,
} from './board';
import type { CalibState } from './types';

const GREEN = 'rgb(0,220,0)';
const ORANGE = 'rgb(255,100,0)';
const AMBER = 'rgb(255,200,0)';
const YELLOW = 'rgb(255,255,0)';
const RED = 'rgb(255,0,0)';
const BOARD_OUTLINE = 'rgb(255,200,0)';

function polyline(ctx: CanvasRenderingContext2D, pts: Pt[], color: string, w = 2) {
  ctx.strokeStyle = color;
  ctx.lineWidth = w;
  ctx.beginPath();
  ctx.moveTo(pts[0][0], pts[0][1]);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]);
  ctx.closePath();
  ctx.stroke();
}

function cross(ctx: CanvasRenderingContext2D, x: number, y: number, color: string, s = 7) {
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(x - s, y);
  ctx.lineTo(x + s, y);
  ctx.moveTo(x, y - s);
  ctx.lineTo(x, y + s);
  ctx.stroke();
}

function dot(ctx: CanvasRenderingContext2D, x: number, y: number, r: number, color: string) {
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fill();
}

function label(ctx: CanvasRenderingContext2D, text: string, x: number, y: number, color: string, px = 11) {
  ctx.fillStyle = color;
  ctx.font = `${px}px sans-serif`;
  ctx.fillText(text, x, y);
}

/** Left panel: raw camera frame with detected tags + board outline. */
export function drawRawPanel(
  cv: Cv,
  ctx: CanvasRenderingContext2D,
  frame: CanvasImageSource,
  cornersDict: Map<number, Corners>,
  tagCentresMm: TagCentres,
  H: any | null,
  squareMm: number,
) {
  const canvas = ctx.canvas;
  ctx.drawImage(frame, 0, 0, canvas.width, canvas.height);

  for (const [tagId, corners] of cornersDict) {
    const known = tagCentresMm.has(tagId);
    const color = known ? GREEN : ORANGE;
    polyline(ctx, corners, color);
    const cx = (corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]) / 4;
    const cy = (corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]) / 4;
    label(ctx, String(tagId), cx - 8, cy + 5, color, 10);
  }

  if (H) {
    const Hinv = new cv.Mat();
    try {
      cv.invert(H, Hinv);
      const scale = OUT_W / squareMm;
      const brd: Pt[] = [
        [0, 0],
        [squareMm * scale, 0],
        [squareMm * scale, squareMm * scale],
        [0, squareMm * scale],
      ];
      const img = perspectiveTransformPts(cv, brd, Hinv);
      polyline(ctx, img, BOARD_OUTLINE);
    } finally {
      Hinv.delete();
    }
  }
}

/**
 * Right panel: bird's-eye rectified view with grid, tag dots, objects and
 * calibration overlays. The warped image is written first via cv.imshow, then
 * overlays are drawn on top with the 2D context.
 */
export function drawRectifiedPanel(
  cv: Cv,
  ctx: CanvasRenderingContext2D,
  frameMat: any,
  H: any | null,
  squareMm: number,
  tagMm: number,
  gapMm: number,
  cornersDict: Map<number, Corners>,
  tagCentresMm: TagCentres,
  innerTagsStart: number,
  calib: CalibState,
) {
  const canvas = ctx.canvas;
  if (!H) {
    ctx.fillStyle = '#111';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = '#888';
    ctx.font = '16px sans-serif';
    ctx.fillText('Need >= 4 border tags', 20, OUT_H / 2);
    return;
  }

  // Warp the frame into the canonical view and blit it.
  const warped = new cv.Mat();
  const dsize = new cv.Size(OUT_W, OUT_H);
  try {
    cv.warpPerspective(frameMat, warped, H, dsize, cv.INTER_LINEAR, cv.BORDER_CONSTANT, new cv.Scalar());
    cv.imshow(canvas, warped);
  } finally {
    warped.delete();
  }

  const scale = OUT_W / squareMm;
  const interiorInsetMm = 2 * tagMm + gapMm;
  const interiorSizeMm = squareMm - 2 * interiorInsetMm;
  const insetPx = Math.round(interiorInsetMm * scale);
  const isizePx = Math.round(interiorSizeMm * scale);

  // Outer + interior border.
  polyline(
    ctx,
    [
      [0, 0],
      [OUT_W - 1, 0],
      [OUT_W - 1, OUT_H - 1],
      [0, OUT_H - 1],
    ],
    'rgb(60,60,60)',
    1,
  );
  ctx.strokeStyle = 'rgb(0,0,0)';
  ctx.lineWidth = 2;
  ctx.strokeRect(insetPx, insetPx, isizePx, isizePx);

  // Grid lines every 5mm, major every 10mm.
  for (let dMm = 5; dMm < interiorSizeMm; dMm += 5) {
    const dPx = insetPx + Math.round(dMm * scale);
    const isMajor = dMm % 10 === 0;
    const color = isMajor ? 'rgb(100,100,100)' : 'rgb(180,180,180)';
    ctx.strokeStyle = color;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(dPx, insetPx);
    ctx.lineTo(dPx, insetPx + isizePx);
    ctx.moveTo(insetPx, dPx);
    ctx.lineTo(insetPx + isizePx, dPx);
    ctx.stroke();
    if (isMajor) {
      label(ctx, String(dMm), dPx + 2, insetPx + 10, 'rgb(80,80,80)', 9);
      label(ctx, String(dMm), insetPx + 2, dPx - 2, 'rgb(80,80,80)', 9);
    }
  }

  // Border tag positions.
  for (const [tagId, [cxMm, cyMm]] of tagCentresMm) {
    const color = tagId < innerTagsStart ? 'rgb(0,180,0)' : 'rgb(200,120,0)';
    dot(ctx, cxMm * scale, cyMm * scale, 4, color);
  }

  // Inner bordered (object) tags.
  for (const [tagId, corners] of cornersDict) {
    if (tagCentresMm.has(tagId)) continue;
    if (!BORDERED_IDS.has(tagId)) continue;

    const bImg = borderedCornersImage(corners);
    const bBoard = perspectiveTransformPts(cv, bImg, H);
    const [tlBoard] = perspectiveTransformPts(cv, [corners[0]], H);
    const xInt = tlBoard[0] / scale - interiorInsetMm;
    const yInt = tlBoard[1] / scale - interiorInsetMm;

    const isCalib = tagId === calib.blockTag;
    const color = isCalib ? AMBER : RED;

    polyline(ctx, bBoard, color);
    cross(ctx, tlBoard[0], tlBoard[1], color, 7);

    const [cxInt, cyInt] = correctPos(xInt, yInt);
    const cxPx = (cxInt + interiorInsetMm) * scale;
    const cyPx = (cyInt + interiorInsetMm) * scale;
    dot(ctx, cxPx, cyPx, 6, 'rgb(0,255,0)');
    label(
      ctx,
      `ID${tagId} raw=(${xInt.toFixed(1)},${yInt.toFixed(1)}) corr=(${cxInt.toFixed(1)},${cyInt.toFixed(1)})mm`,
      tlBoard[0] + 4,
      tlBoard[1] - 4,
      'rgb(0,255,0)',
      10,
    );
  }

  // Calibration overlays.
  if (calib.active) {
    const step = calib.step;
    const bwPx = Math.round(calib.blockW * scale);
    const bdPx = Math.round(calib.blockD * scale);
    const cornerTouchPx: Array<[number, number]> = [
      [insetPx, insetPx],
      [insetPx + isizePx, insetPx],
      [insetPx, insetPx + isizePx],
      [insetPx + isizePx, insetPx + isizePx],
    ];
    const blockRects: Array<[number, number, number, number]> = [
      [0, 0, bwPx, bdPx],
      [-bwPx, 0, 0, bdPx],
      [0, -bdPx, bwPx, 0],
      [-bwPx, -bdPx, 0, 0],
    ];

    if (step < calib.cornerPositions.length) {
      const [trueX, trueY] = calib.cornerPositions[step];
      const txPx = insetPx + trueX * scale;
      const tyPx = insetPx + trueY * scale;
      const [cxTouch, cyTouch] = cornerTouchPx[step];
      const [dx0, dy0, dx1, dy1] = blockRects[step];
      ctx.strokeStyle = YELLOW;
      ctx.lineWidth = 2;
      ctx.strokeRect(cxTouch + dx0, cyTouch + dy0, dx1 - dx0, dy1 - dy0);
      cross(ctx, txPx, tyPx, YELLOW, 8);
      label(ctx, CALIB_CORNER_NAMES[step], cxTouch + 4, cyTouch + 14, YELLOW, 14);
    }

    calib.donePx.forEach(([px, py], idx) => {
      dot(ctx, px, py, 6, 'rgb(0,220,0)');
      label(ctx, CALIB_CORNER_NAMES[idx], px + 4, py - 4, 'rgb(0,220,0)', 10);
    });
  }
}
