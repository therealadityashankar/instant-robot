import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  DEFAULT_GOAL_QUAD,
  DEFAULT_GOAL_QUAD_REAL,
  DEFAULT_GOAL_QUAD_SIM,
  computeQuadCenter,
  computeQuadSideLengths,
  computeQuadAvgSize,
  computeQuadOrientationDeg,
  translateQuad,
  scaleQuad,
  rotateQuad,
  evaluateTolerance,
  type CornerPoint,
} from '../src/lib/pickCalibration';

test('real goal quad default corners produce valid centroid on the right side', () => {
  const [cx, cy] = computeQuadCenter(DEFAULT_GOAL_QUAD_REAL.corners);
  assert.ok(cx > 400, `Centroid X (${cx}) should be on right side of 640px frame`);
  assert.ok(cy > 150 && cy < 350, `Centroid Y (${cy}) should be roughly centered vertically`);
});

test('simulation default goal quad is centered for simulated arm camera', () => {
  const [cx, cy] = computeQuadCenter(DEFAULT_GOAL_QUAD_SIM.corners);
  assert.ok(cx >= 300 && cx <= 340, `Centroid X (${cx}) should be centered in 640px frame`);
  assert.ok(cy >= 200 && cy <= 240, `Centroid Y (${cy}) should be centered in 480px frame`);
});

test('generic quadrilateral side lengths and scaling', () => {
  const quad: [CornerPoint, CornerPoint, CornerPoint, CornerPoint] = [
    [100, 100],
    [200, 100],
    [200, 200],
    [100, 200],
  ];

  const sides = computeQuadSideLengths(quad);
  assert.deepEqual(sides, [100, 100, 100, 100]);
  assert.equal(computeQuadAvgSize(quad), 100);

  // Scaled by 1.5x
  const scaled = scaleQuad(quad, 1.5);
  const scaledSides = computeQuadSideLengths(scaled);
  assert.ok(Math.abs(scaledSides[0] - 150) < 1e-5);
  assert.ok(Math.abs(scaledSides[1] - 150) < 1e-5);
  assert.ok(Math.abs(scaledSides[2] - 150) < 1e-5);
  assert.ok(Math.abs(scaledSides[3] - 150) < 1e-5);
});

test('quadrilateral translation and rotation', () => {
  const quad: [CornerPoint, CornerPoint, CornerPoint, CornerPoint] = [
    [100, 100],
    [200, 100],
    [200, 200],
    [100, 200],
  ];

  // Translate +50 X, -20 Y
  const translated = translateQuad(quad, 50, -20);
  assert.deepEqual(translated, [
    [150, 80],
    [250, 80],
    [250, 180],
    [150, 180],
  ]);

  // Rotate 90 deg around centroid (150, 150)
  const rotated = rotateQuad(quad, 90);
  const [rcx, rcy] = computeQuadCenter(rotated);
  assert.ok(Math.abs(rcx - 150) < 1e-5);
  assert.ok(Math.abs(rcy - 150) < 1e-5);
  const rotAngle = computeQuadOrientationDeg(rotated);
  assert.ok(Math.abs(rotAngle - 90) < 1e-5);
});

test('evaluateTolerance verifies points within error border region', () => {
  const goal: [CornerPoint, CornerPoint, CornerPoint, CornerPoint] = [
    [100, 100],
    [200, 100],
    [200, 200],
    [100, 200],
  ];

  // Inside tolerance (+6px offset per corner, tolerance = 10px)
  const insideTag: [CornerPoint, CornerPoint, CornerPoint, CornerPoint] = [
    [106, 100],
    [200, 106],
    [194, 200],
    [100, 194],
  ];
  const resInside = evaluateTolerance(goal, insideTag, 10);
  assert.equal(resInside.isInside, true);
  assert.equal(resInside.maxDist, 6);

  // Outside tolerance (+15px offset on one corner, tolerance = 10px)
  const outsideTag: [CornerPoint, CornerPoint, CornerPoint, CornerPoint] = [
    [115, 100],
    [200, 100],
    [200, 200],
    [100, 200],
  ];
  const resOutside = evaluateTolerance(goal, outsideTag, 10);
  assert.equal(resOutside.isInside, false);
  assert.equal(resOutside.maxDist, 15);
});
