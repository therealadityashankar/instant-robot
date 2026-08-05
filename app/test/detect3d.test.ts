// Synthetic-projection test for the solvePnP board-frame localisation. We place
// a board + a raised block tag at known board coordinates, project them through a
// known camera, then check the pipeline recovers the block's board XY — including
// the fact that the tag sits above the board plane (parallax).

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { loadCv } from '../src/lib/cv';
import { boardPoseFromTags, blockBoardXY } from '../src/lib/detect3d';
import type { Corners } from '../src/lib/geometry';
import type { TagCentres } from '../src/lib/board';
import type { Intrinsics } from '../src/lib/charuco';

test('solvePnP recovers block board-XY through a tilted camera, ignoring tag height', async () => {
  const cv = await loadCv();
  const fx = 600, fy = 600, cx = 320, cy = 240, W = 640, H = 480;
  const camMat = cv.matFromArray(3, 3, cv.CV_64F, [fx, 0, cx, 0, fy, cy, 0, 0, 1]);
  const dist = cv.matFromArray(1, 5, cv.CV_64F, [0, 0, 0, 0, 0]);
  const rvec = cv.matFromArray(3, 1, cv.CV_64F, [0.12, -0.06, 0.03]);
  const tvec = cv.matFromArray(3, 1, cv.CV_64F, [-60, -90, 500]);

  const project = (pts: number[]): Array<[number, number]> => {
    const obj = cv.matFromArray(pts.length / 3, 1, cv.CV_32FC3, pts);
    const out = new cv.Mat();
    cv.projectPoints(obj, rvec, tvec, camMat, dist, out);
    const d = out.data32F as Float32Array;
    const res: Array<[number, number]> = [];
    for (let i = 0; i < out.total(); i++) res.push([d[i * 2], d[i * 2 + 1]]);
    obj.delete();
    out.delete();
    return res;
  };

  const tagMm = 16, h = tagMm / 2;
  const centres: TagCentres = new Map([
    [0, [8, 8]], [1, [172, 8]], [2, [172, 172]], [3, [8, 172]], [4, [90, 8]], [5, [8, 90]],
  ]);
  const cornersDict = new Map<number, Corners>();
  for (const [id, [X, Y]] of centres) {
    cornersDict.set(id, project([X - h, Y - h, 0, X + h, Y - h, 0, X + h, Y + h, 0, X - h, Y + h, 0]) as Corners);
  }

  const intr: Intrinsics = {
    cameraMatrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1],
    distCoeffs: [0, 0, 0, 0, 0],
    imgW: W, imgH: H, rms: 0,
  };

  for (const [BX, BY, BZ] of [[90, 90, 15], [40, 130, 15], [150, 60, 30]]) {
    const m = 20, mh = m / 2;
    const blockPx = project([BX - mh, BY - mh, BZ, BX + mh, BY - mh, BZ, BX + mh, BY + mh, BZ, BX - mh, BY + mh, BZ]) as Corners;
    const pose = boardPoseFromTags(cv, cornersDict, centres, tagMm, intr, W, H);
    assert.ok(pose, 'board pose solved');
    const xy = blockBoardXY(cv, blockPx, m, intr, W, H, pose!);
    assert.ok(xy, 'block XY solved');
    assert.ok(Math.abs(xy!.x - BX) < 0.5, `X ${xy!.x} ≈ ${BX}`);
    assert.ok(Math.abs(xy!.y - BY) < 0.5, `Y ${xy!.y} ≈ ${BY}`);
  }
});
