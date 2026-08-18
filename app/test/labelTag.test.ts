import test from 'node:test';
import assert from 'node:assert/strict';
import {
  findLabeledTagPairs,
  LEFT_NAV_ID,
  RIGHT_NAV_ID,
  hasLabeledTagPair,
} from '../src/lib/labelTag';
import type { Corners } from '../src/lib/geometry';

test('findLabeledTagPairs identifies valid left and right 200/201 pairs', () => {
  const corners = new Map<number, Corners>();

  // Left marker 200: (x=100..140, y=200..240)
  const left: Corners = [
    [100, 200], // TL
    [140, 200], // TR
    [140, 240], // BR
    [100, 240], // BL
  ];

  // Right marker 201: (x=240..280, y=200..240) - shifted right by 100px (gap is 100px)
  const right: Corners = [
    [240, 200], // TL
    [280, 200], // TR
    [280, 240], // BR
    [240, 240], // BL
  ];

  corners.set(LEFT_NAV_ID, left);
  corners.set(RIGHT_NAV_ID, right);

  assert.equal(hasLabeledTagPair(corners), true);
  const pairs = findLabeledTagPairs(corners);
  assert.equal(pairs.length, 1);
  assert.deepEqual(pairs[0].left, left);
  assert.deepEqual(pairs[0].right, right);
});

test('findLabeledTagPairs handles inverted visual order and rejects vertical misalignment', () => {
  const corners = new Map<number, Corners>();

  // 200 is visually on the right, 201 is visually on the left
  const tag200: Corners = [
    [300, 200],
    [340, 200],
    [340, 240],
    [300, 240],
  ];
  const tag201: Corners = [
    [100, 200],
    [140, 200],
    [140, 240],
    [100, 240],
  ];

  corners.set(LEFT_NAV_ID, tag200);
  corners.set(RIGHT_NAV_ID, tag201);

  assert.equal(hasLabeledTagPair(corners), true);
  const pairs = findLabeledTagPairs(corners);
  assert.equal(pairs.length, 1);
  assert.deepEqual(pairs[0].left, tag201); // 201 is visually on left
  assert.deepEqual(pairs[0].right, tag200); // 200 is visually on right

  // Now test excessive vertical misalignment
  const tag201Misaligned: Corners = [
    [100, 400],
    [140, 400],
    [140, 440],
    [100, 440],
  ];
  corners.set(RIGHT_NAV_ID, tag201Misaligned);
  assert.equal(hasLabeledTagPair(corners), false);
  assert.equal(findLabeledTagPairs(corners).length, 0);
});

import { getInnerEdgeCorners, warpTextRegion } from '../src/lib/labelTag';
import { loadCv } from '../src/lib/cv';

test('getInnerEdgeCorners extracts accurate inner boundary corners under arbitrary rotation & scale', async () => {
  const cv = await loadCv();

  // Test across multiple angles: 0°, 30°, 45°, 90°, 180°, -60°
  const testAngles = [0, Math.PI / 6, Math.PI / 4, Math.PI / 2, Math.PI, -Math.PI / 3];
  const scale = 2.5;
  const tx = 320, ty = 240;

  for (const theta of testAngles) {
    const cos = Math.cos(theta);
    const sin = Math.sin(theta);

    const transform = (x: number, y: number): [number, number] => [
      scale * (x * cos - y * sin) + tx,
      scale * (x * sin + y * cos) + ty,
    ];

    // Left marker (200) from x = -75 to -35, y = -20 to +20
    const c200: Corners = [
      transform(-75, -20),
      transform(-35, -20),
      transform(-35,  20),
      transform(-75,  20),
    ];

    // Right marker (201) from x = 35 to 75, y = -20 to +20
    const c201: Corners = [
      transform( 35, -20),
      transform( 75, -20),
      transform( 75,  20),
      transform( 35,  20),
    ];

    const res = getInnerEdgeCorners(c200, c201);
    assert.ok(res !== null, `getInnerEdgeCorners should not be null for angle ${theta}`);

    // Inner edge of left marker is at x = -35 (inset 4% of (70) = ~2.8 -> x ≈ -32.2)
    // Inner edge of right marker is at x = +35 (inset 4% of (70) = ~2.8 -> x ≈ +32.2)
    const [tl, tr, br, bl] = res.fullCorners;

    // Check that tl is top-left in card coordinates, tr is top-right, etc.
    const expectedCenter = transform(0, 0);
    const measuredCenter = [
      (tl[0] + tr[0] + br[0] + bl[0]) / 4,
      (tl[1] + tr[1] + br[1] + bl[1]) / 4,
    ];
    assert.ok(Math.hypot(measuredCenter[0] - expectedCenter[0], measuredCenter[1] - expectedCenter[1]) < 2.0);

    // Test warpTextRegion
    const fakeMat = new cv.Mat(480, 640, cv.CV_8UC1);
    const warped = warpTextRegion(cv, fakeMat, c200, c201);
    assert.ok(warped !== null);
    assert.equal(warped.fullMat.cols, 400);
    assert.equal(warped.fullMat.rows, 240);
    assert.equal(warped.labelMat.cols, 400);
    assert.equal(warped.labelMat.rows, 120);

    warped.fullMat.delete();
    warped.labelMat.delete();
    warped.descMat.delete();
    fakeMat.delete();
  }
});
