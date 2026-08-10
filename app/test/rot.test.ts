import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  matMul3x3,
  rotAngleBetween,
  rotateAboutApproach,
  slewRotation,
  nearestHalfTurn,
} from '../src/lib/rot';

/** An arbitrary but honest rotation to work from. */
function sampleRotation(a = 0.7, b = -1.1, g = 0.35): number[] {
  const Rz = [Math.cos(a), -Math.sin(a), 0, Math.sin(a), Math.cos(a), 0, 0, 0, 1];
  const Ry = [Math.cos(b), 0, Math.sin(b), 0, 1, 0, -Math.sin(b), 0, Math.cos(b)];
  const Rx = [1, 0, 0, 0, Math.cos(g), -Math.sin(g), 0, Math.sin(g), Math.cos(g)];
  return matMul3x3(matMul3x3(Rz, Ry), Rx);
}

function det(m: number[]): number {
  return (
    m[0] * (m[4] * m[8] - m[5] * m[7]) -
    m[1] * (m[3] * m[8] - m[5] * m[6]) +
    m[2] * (m[3] * m[7] - m[4] * m[6])
  );
}

test('a half-turn about the approach axis keeps that axis and reverses the jaws', () => {
  const A = sampleRotation();
  const B = rotateAboutApproach(A, Math.PI);
  // det +1 matters: a frame built by hand once came out a mirror, and a mirror
  // converts to a quaternion as branch-jumping garbage rather than as an error.
  assert.ok(Math.abs(det(A) - 1) < 1e-9, `det(A) = ${det(A)}`);
  assert.ok(Math.abs(det(B) - 1) < 1e-9, `det(B) = ${det(B)}`);
  assert.ok(Math.abs(rotAngleBetween(A, B) - Math.PI) < 1e-9);
  for (const k of [0, 3, 6]) assert.ok(Math.abs(A[k] - B[k]) < 1e-12, 'approach axis moved');
});

test('rotAngleBetween measures the turn it was given', () => {
  const A = sampleRotation();
  for (const want of [0, 0.1, 0.9, 2.0, Math.PI]) {
    const got = rotAngleBetween(A, rotateAboutApproach(A, want));
    assert.ok(Math.abs(got - want) < 1e-9, `${want} rad measured as ${got}`);
  }
});

test('nearestHalfTurn follows the wrist instead of flipping', () => {
  const A = sampleRotation();
  const B = rotateAboutApproach(A, Math.PI);
  // This is the bug it exists for: a wrist sitting just short of B must be given
  // B, not A. Choosing the representative nearest zero swung the wrist through a
  // half-turn mid-approach, which reconfigured the arm and lost the tag.
  const nearB = rotateAboutApproach(A, Math.PI - 0.05);
  assert.ok(rotAngleBetween(nearestHalfTurn(A, nearB), nearB) < 0.06, 'did not pick the near twin');
  const nearA = rotateAboutApproach(A, 0.05);
  assert.ok(rotAngleBetween(nearestHalfTurn(A, nearA), nearA) < 0.06, 'flipped when it should not');
  // Whichever it picks is still a grasp about the same approach axis.
  const picked = nearestHalfTurn(A, nearB);
  for (const k of [0, 3, 6]) assert.ok(Math.abs(A[k] - picked[k]) < 1e-12);
});

test('slewRotation caps the turn and keeps the result a rotation', () => {
  const A = sampleRotation();
  const cap = 0.44;
  const far = rotateAboutApproach(A, 0.9);
  const step = slewRotation(A, far, cap);
  assert.ok(Math.abs(rotAngleBetween(A, step) - cap) < 1e-9, 'not capped at the limit');
  assert.ok(Math.abs(det(step) - 1) < 1e-9, 'result is not a rotation');
  assert.ok(rotAngleBetween(step, far) < rotAngleBetween(A, far), 'stepped the wrong way');
});

test('slewRotation passes a small turn straight through', () => {
  const A = sampleRotation();
  const near = rotateAboutApproach(A, 0.1);
  assert.ok(rotAngleBetween(slewRotation(A, near, 0.44), near) < 1e-9);
});

test('slewRotation caps a half-turn instead of snapping through it', () => {
  // A 180° difference has no skew part to read the axis from, and the obvious
  // bail-out — return the target — snaps the full half-turn, which is the one
  // move the cap exists to prevent.
  const A = sampleRotation();
  const B = rotateAboutApproach(A, Math.PI);
  const step = slewRotation(A, B, 0.44);
  assert.ok(Math.abs(rotAngleBetween(A, step) - 0.44) < 1e-6, 'half-turn was not capped');
  assert.ok(Math.abs(det(step) - 1) < 1e-9);
  assert.ok(rotAngleBetween(step, B) < Math.PI - 1e-6, 'stepped away from the target');
});

test('repeated slewing converges rather than oscillating', () => {
  const A = sampleRotation();
  const B = rotateAboutApproach(A, Math.PI);
  let cur: number[] = A;
  for (let i = 0; i < 12; i++) cur = slewRotation(cur, B, 0.44);
  assert.ok(rotAngleBetween(cur, B) < 1e-6, `left ${rotAngleBetween(cur, B)} rad short`);
  assert.ok(Math.abs(det(cur) - 1) < 1e-9);
});

test('matMul3x3 composes row-major frames', () => {
  const I = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const A = sampleRotation();
  for (let i = 0; i < 9; i++) assert.ok(Math.abs(matMul3x3(A, I)[i] - A[i]) < 1e-12);
  // Aᵀ·A is the identity for a rotation, which pins down the index order.
  const At = [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
  const p = matMul3x3(At, A);
  for (let i = 0; i < 9; i++) assert.ok(Math.abs(p[i] - I[i]) < 1e-12, `AᵀA[${i}] = ${p[i]}`);
});
