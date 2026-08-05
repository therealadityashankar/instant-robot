// Unit tests for the empirical LeKiwi drive model (pure math, no hardware).

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { wheelSpeeds, bodyToPrimitives, DEFAULT_BASE_CONFIG } from '../src/lib/lekiwiBase';

test('idle command → all wheels zero', () => {
  const s = wheelSpeeds(0, 0, 0, 0, DEFAULT_BASE_CONFIG);
  for (const v of s.values()) assert.equal(v, 0);
});

test('a single primitive fires exactly its pattern (2 wheels)', () => {
  const ids = DEFAULT_BASE_CONFIG.wheelIds;
  const s = wheelSpeeds(1, 0, 0, 0, DEFAULT_BASE_CONFIG);
  DEFAULT_BASE_CONFIG.forward.forEach((p, i) =>
    assert.equal(s.get(ids[i]), p * DEFAULT_BASE_CONFIG.speed),
  );
  // forward pattern [0,-1,1] → wheel 0 idle, two wheels drive.
  assert.equal(s.get(ids[0]), 0);
});

test('reversing every amount negates every wheel', () => {
  const f = wheelSpeeds(0.5, 0.3, -0.2, 0.4, DEFAULT_BASE_CONFIG);
  const r = wheelSpeeds(-0.5, -0.3, 0.2, -0.4, DEFAULT_BASE_CONFIG);
  for (const id of DEFAULT_BASE_CONFIG.wheelIds) assert.equal(r.get(id), -f.get(id)!);
});

test('speeds are clamped to the Feetech limit', () => {
  const cfg = { ...DEFAULT_BASE_CONFIG, speed: 100000 };
  const s = wheelSpeeds(1, 1, 1, 1, cfg);
  for (const v of s.values()) assert.ok(Math.abs(v) <= 1023);
});

test('bodyToPrimitives reconstructs the commanded direction', () => {
  const d = { forward: [1, 0], backLeft: [-0.5, 0.8660254], backRight: [-0.5, -0.8660254] };
  for (const [vx, vy] of [[1, 0], [0, 1], [0.4, -0.7]] as const) {
    const { fwd, bl, br } = bodyToPrimitives(vx, vy);
    const rx = fwd * d.forward[0] + bl * d.backLeft[0] + br * d.backRight[0];
    const ry = fwd * d.forward[1] + bl * d.backLeft[1] + br * d.backRight[1];
    assert.ok(Math.abs(rx - vx) < 1e-6 && Math.abs(ry - vy) < 1e-6);
  }
});
