import { test } from 'node:test';
import assert from 'node:assert/strict';
import { baseCameraPose, seeTag } from '../src/lib/simVision';
import { stationTagWorld, stationTagNormal } from '../src/lib/boardSim';

const ARM_LIFT = 0.15022448; // lekiwi base.json baseDrop
const CAM_FWD = 0.12;
const CAM_H = 0.15;
const HFOV = (30 * Math.PI) / 180;
const VFOV = (23 * Math.PI) / 180;
const OPTS = { hfov: HFOV, vfov: VFOV };

test('station tag is not visible from the start pose', () => {
  const cam = baseCameraPose(0, 0, 0, ARM_LIFT, CAM_FWD, CAM_H);
  const r = seeTag(cam, stationTagWorld(), stationTagNormal(), OPTS);
  assert.equal(r.hit, null, `expected hidden at start, got: ${r.why}`);
});

test('turning to face the station makes its tag visible', () => {
  const p = stationTagWorld();
  const bearing = (Math.atan2(p[1], p[0]) * 180) / Math.PI;
  const cam = baseCameraPose(0, 0, bearing, ARM_LIFT, CAM_FWD, CAM_H);
  const r = seeTag(cam, p, stationTagNormal(), OPTS);
  assert.ok(r.hit, `expected visible when facing it, got: ${r.why}`);
});

test('some heading in a full sweep sees it', () => {
  const p = stationTagWorld();
  const seen: number[] = [];
  for (let deg = -180; deg < 180; deg += 2) {
    const cam = baseCameraPose(0, 0, deg, ARM_LIFT, CAM_FWD, CAM_H);
    if (seeTag(cam, p, stationTagNormal(), OPTS).hit) seen.push(deg);
  }
  assert.ok(seen.length > 0, 'a full turn never sees the station');
  console.log(`  visible over headings ${seen[0]}°..${seen[seen.length - 1]}°`);
});

test('the standoff point sits square in front of the tag face', () => {
  // Parking "near" a tag is not the same as parking in front of it: this is the
  // difference between facing a drawer and facing past it.
  const standoff = 0.3;
  const p = stationTagWorld();
  const n = stationTagNormal();
  const gx = p[0] + n[0] * standoff;
  const gy = p[1] + n[1] * standoff;
  // From the standoff point, the tag lies exactly opposite its own normal…
  const toTag = [p[0] - gx, p[1] - gy];
  const len = Math.hypot(toTag[0], toTag[1]);
  assert.ok(Math.abs(len - standoff) < 1e-9, 'standoff distance is honoured');
  const dot = (toTag[0] / len) * n[0] + (toTag[1] / len) * n[1];
  assert.ok(Math.abs(dot + 1) < 1e-9, `should approach head-on, got dot ${dot}`);
});
