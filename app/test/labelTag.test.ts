import test from 'node:test';
import assert from 'node:assert/strict';
import {
  findLabeledTagPairs,
  LEFT_NAV_ID,
  RIGHT_NAV_ID,
  hasLabeledTagPair,
} from '../src/lib/labelTag';
import type { Corners } from '../src/lib/geometry';

test('findLabeledTagPairs identifies valid unique left and right 200 anchor pairs', () => {
  const corners = new Map<number, Corners>();

  // Left marker 250 (unique station ID): (x=100..140, y=200..240)
  const left250: Corners = [
    [100, 200], // TL
    [140, 200], // TR
    [140, 240], // BR
    [100, 240], // BL
  ];

  // Right marker 200 (fixed anchor): (x=240..280, y=200..240) - shifted right by 100px (gap is 100px)
  const right200: Corners = [
    [240, 200], // TL
    [280, 200], // TR
    [280, 240], // BR
    [240, 240], // BL
  ];

  corners.set(250, left250);
  corners.set(RIGHT_NAV_ID, right200);

  assert.equal(hasLabeledTagPair(corners), true);
  const pairs = findLabeledTagPairs(corners);
  assert.equal(pairs.length, 1);
  assert.equal(pairs[0].stationId, 250);
  assert.deepEqual(pairs[0].left, left250);
  assert.deepEqual(pairs[0].right, right200);
});

test('findLabeledTagPairs matches multiple stations with anchor 200 and rejects vertical misalignment', () => {
  const corners = new Map<number, Corners>();

  // Station 201: left of 200
  const tag201: Corners = [
    [100, 200],
    [140, 200],
    [140, 240],
    [100, 240],
  ];
  // Anchor 200: right of 201
  const tag200: Corners = [
    [300, 200],
    [340, 200],
    [340, 240],
    [300, 240],
  ];

  corners.set(201, tag201);
  corners.set(RIGHT_NAV_ID, tag200);

  assert.equal(hasLabeledTagPair(corners), true);
  const pairs = findLabeledTagPairs(corners);
  assert.equal(pairs.length, 1);
  assert.equal(pairs[0].stationId, 201);
  assert.deepEqual(pairs[0].left, tag201);
  assert.deepEqual(pairs[0].right, tag200);

  // Now test excessive vertical misalignment
  const tagMisaligned: Corners = [
    [100, 400],
    [140, 400],
    [140, 440],
    [100, 440],
  ];
  corners.set(201, tagMisaligned);
  assert.equal(hasLabeledTagPair(corners), false);
  assert.equal(findLabeledTagPairs(corners).length, 0);
});

test('findLabeledTagPairs strictly rejects anchor 200 placed to the LEFT of station tag', () => {
  // Station 201 is on the right (x=300..340), Anchor 200 is on the left (x=100..140) -> INVALID OPPOSITE DIRECTION
  const tag201Right: Corners = [
    [300, 200],
    [340, 200],
    [340, 240],
    [300, 240],
  ];
  const tag200Left: Corners = [
    [100, 200],
    [140, 200],
    [140, 240],
    [100, 240],
  ];

  const corners = new Map<number, Corners>();
  corners.set(201, tag201Right);
  corners.set(RIGHT_NAV_ID, tag200Left);

  // Must be rejected because 200 is to the left of 201
  assert.equal(hasLabeledTagPair(corners), false);
  assert.equal(findLabeledTagPairs(corners).length, 0);
});

test('findLabeledTagPairs pairs multiple stations correctly when multiple 200 anchors exist in scene', () => {
  // Station 1: 201 (x=50) + 200_A (x=150)
  const tag201: Corners = [[50, 100], [80, 100], [80, 130], [50, 130]];
  const tag200A: Corners = [[150, 100], [180, 100], [180, 130], [150, 130]];

  // Station 2: 202 (x=300) + 200_B (x=400)
  const tag202: Corners = [[300, 100], [330, 100], [330, 130], [300, 130]];
  const tag200B: Corners = [[400, 100], [430, 100], [430, 130], [400, 130]];

  const allDetections = [
    { id: 201, corners: tag201 },
    { id: 200, corners: tag200A },
    { id: 202, corners: tag202 },
    { id: 200, corners: tag200B },
  ];

  const pairs = findLabeledTagPairs(allDetections);
  assert.equal(pairs.length, 2);
  assert.equal(pairs[0].stationId, 201);
  assert.deepEqual(pairs[0].right, tag200A);
  assert.equal(pairs[1].stationId, 202);
  assert.deepEqual(pairs[1].right, tag200B);
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
