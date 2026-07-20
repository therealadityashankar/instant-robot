// Pick state machine — pure logic, no MuJoCo. Feeding back the returned target
// as the EE position (perfect tracking) should walk the phases in order and
// finish, closing the gripper during grasp/lift.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { PickController, DEFAULT_PICK } from '../src/lib/pick';

test('pick walks approach → descend → grasp → lift → done', () => {
  const pc = new PickController();
  const block: [number, number, number] = [0.2, 0.03, DEFAULT_PICK.hoverZ];
  pc.start(block);

  const seen = new Set<string>();
  let ee: [number, number, number] = [0, 0, 0];
  let last = pc.step(ee);
  for (let i = 0; i < 500 && !last.done; i++) {
    // Perfect tracking: EE reaches the requested target each frame.
    ee = last.target;
    last = pc.step(ee);
    seen.add(last.phase);
  }

  assert.ok(last.done, 'pick completed');
  assert.equal(pc.phase, 'done');
  for (const phase of ['approach', 'descend', 'grasp', 'lift']) {
    assert.ok(seen.has(phase), `went through ${phase}`);
  }
  // Gripper ends up closed.
  assert.ok(last.gripper > 0, 'gripper closed at the end');
});

test('cancel returns to idle and holds position', () => {
  const pc = new PickController();
  pc.start([0.2, 0, 0.1]);
  pc.step([0, 0, 0]);
  pc.cancel();
  assert.equal(pc.phase, 'idle');
  assert.equal(pc.active, false);
  const s = pc.step([0.11, 0.12, 0.13]);
  assert.deepEqual(s.target, [0.11, 0.12, 0.13]); // holds current EE
});
