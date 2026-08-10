import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, readdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { loadMujocoModule, mountModel, type ModelFiles } from '../src/lib/mujocoSession';
import { buildBoardSceneXml, stationTagWorld, STATION_H } from '../src/lib/boardSim';

const here = dirname(fileURLToPath(import.meta.url));
const MODEL_DIR = join(here, '..', '..', 'models', 'so101');
const XML = 'so101_simplified.xml';

test('station geoms compile into the scene', async () => {
  const mj = await loadMujocoModule();
  const files: ModelFiles = {};
  const base = readFileSync(join(MODEL_DIR, XML), 'utf8');
  files['scene.xml'] = new TextEncoder().encode(buildBoardSceneXml(base, { physics: false }));
  for (const f of readdirSync(join(MODEL_DIR, 'assets'))) {
    if (f.endsWith('.stl')) files[`assets/${f}`] = new Uint8Array(readFileSync(join(MODEL_DIR, 'assets', f)));
  }
  const { model } = mountModel(mj, files, 'scene.xml');
  const { STATIONS } = await import('../src/lib/boardSim');
  for (const st of STATIONS) {
    const bid = mj.mj_name2id(model, 1 /* mjOBJ_BODY */, `station_${st.navTag}`);
    assert.ok(bid >= 0, `station_${st.navTag} body missing`);
    const gid = mj.mj_name2id(model, 5 /* mjOBJ_GEOM */, `station_navtag_${st.navTag}`);
    assert.ok(gid >= 0, `nav tag geom missing for station ${st.navTag}`);
    if (st.propTag) {
      // The block is a free body injected separately, so its marker is block_tag;
      // every other prop is part of its station body.
      const name = st.prop === 'block' ? 'block_tag' : `ptag_${st.propTag}`;
      const pid = mj.mj_name2id(model, 5 /* mjOBJ_GEOM */, name);
      assert.ok(pid >= 0, `prop tag geom "${name}" missing for station ${st.navTag}`);
    }
  }

  // The tag must sit on the pedestal's side, below its top.
  const p = stationTagWorld();
  assert.ok(p[2] > 0 && p[2] < STATION_H, `tag z ${p[2]} not on the pedestal side`);
});

test('station starts outside the camera view, and a turn brings it in', async () => {
  // The robot starts at the origin facing +X. Discovery is only meaningful if the
  // tag is NOT visible from there but is visible from somewhere in a full turn.
  const HFOV = (30 * Math.PI) / 180;
  const p = stationTagWorld();
  const bearing = Math.atan2(p[1], p[0]); // relative to the +X start heading
  assert.ok(
    Math.abs(bearing) > HFOV,
    `station is already in view at start (bearing ${(bearing * 180) / Math.PI}°)`,
  );
  // Somewhere in a 360° sweep the heading lines up with it.
  let seen = false;
  for (let deg = 0; deg < 360; deg += 5) {
    const yaw = (deg * Math.PI) / 180;
    const rel = Math.atan2(Math.sin(bearing - yaw), Math.cos(bearing - yaw));
    if (Math.abs(rel) < HFOV) seen = true;
  }
  assert.ok(seen, 'a full turn never brings the station into view');
});

test('the station tag faces the robot once found', async () => {
  // Bearing alone isn't enough: a tag turned edge-on or away can be in frame and
  // still unreadable, which is exactly how a badly-placed station fails.
  const { stationTagNormal } = await import('../src/lib/boardSim');
  const p = stationTagWorld();
  const n = stationTagNormal();
  const facing = n[0] * p[0] + n[1] * p[1]; // normal · (tag − origin)
  assert.ok(facing < -0.1, `tag is not turned toward the start point (dot ${facing.toFixed(3)})`);
});


test('the wrist-cam bracket mesh loads and carries no gripper jaw', async () => {
  const mj = await loadMujocoModule();
  const files: ModelFiles = {};
  const base = readFileSync(join(MODEL_DIR, XML), 'utf8');
  files['scene.xml'] = new TextEncoder().encode(buildBoardSceneXml(base, { physics: false }));
  for (const f of readdirSync(join(MODEL_DIR, 'assets'))) {
    if (f.endsWith('.stl')) files[`assets/${f}`] = new Uint8Array(readFileSync(join(MODEL_DIR, 'assets', f)));
  }
  const { model } = mountModel(mj, files, 'scene.xml');
  // A binary-STL or scale problem here shows up as the whole scene failing to
  // compile, so this is really a guard on the mesh asset.
  const bid = mj.mj_name2id(model, 1 /* mjOBJ_BODY */, 'wrist_cam_mount');
  assert.ok(bid >= 0, 'wrist_cam_mount body missing');
  assert.ok((model.body_mocapid as Int32Array)[bid] >= 0, 'wrist_cam_mount is not mocap');

  // The upstream STL fuses the bracket to a copy of the gripper's fixed jaw, which
  // drew a second jaw over the arm's own. The jaw is the tall finger above z=16mm;
  // if a re-download ever reinstates it, this catches it.
  const stl = readFileSync(join(MODEL_DIR, 'assets', 'wrist_cam_mount.stl'));
  const tris = stl.readUInt32LE(80);
  let jaw = 0;
  for (let i = 0; i < tris; i++) {
    const o = 84 + i * 50 + 12;
    let z = 0, y = 0;
    for (let v = 0; v < 3; v++) {
      y += stl.readFloatLE(o + v * 12 + 4);
      z += stl.readFloatLE(o + v * 12 + 8);
    }
    if (z / 3 > 16 && y / 3 < 28) jaw++;
  }
  assert.equal(jaw, 0, `${jaw} jaw triangles left in the bracket mesh`);
});

test('a scattered block stays fully on the board at every angle', async () => {
  const { BLOCK_SCATTER_R, BLOCK_HALF_X, BLOCK_HALF_Y, SQUARE_MM } = await import('../src/lib/boardSim');
  const half = SQUARE_MM / 2 / 1000;
  assert.ok(BLOCK_SCATTER_R > 0.02, `scatter radius ${BLOCK_SCATTER_R} leaves no room to move`);
  // Worst case over both the placement angle and the block's own yaw: a corner of
  // the block must never cross the board edge. Checked by brute force rather than
  // by repeating the derivation, so a wrong formula can't agree with itself.
  for (let p = 0; p < 360; p += 5) {
    const pa = (p * Math.PI) / 180;
    const cx = BLOCK_SCATTER_R * Math.cos(pa);
    const cy = BLOCK_SCATTER_R * Math.sin(pa);
    for (let y = 0; y < 360; y += 5) {
      const ya = (y * Math.PI) / 180;
      const c = Math.cos(ya), s = Math.sin(ya);
      for (const [dx, dy] of [
        [BLOCK_HALF_X, BLOCK_HALF_Y],
        [BLOCK_HALF_X, -BLOCK_HALF_Y],
        [-BLOCK_HALF_X, BLOCK_HALF_Y],
        [-BLOCK_HALF_X, -BLOCK_HALF_Y],
      ]) {
        const x = cx + dx * c - dy * s;
        const yy = cy + dx * s + dy * c;
        assert.ok(
          Math.abs(x) <= half + 1e-9 && Math.abs(yy) <= half + 1e-9,
          `corner (${x.toFixed(4)}, ${yy.toFixed(4)}) off a ${half}m board at placement ${p}°, yaw ${y}°`,
        );
      }
    }
  }
});

test('wrist_roll carries the servo-zero reference', async () => {
  const { withWristRollRef, WRIST_ROLL_REF } = await import('../src/lib/boardSim');
  const base = readFileSync(join(MODEL_DIR, XML), 'utf8');
  const fixed = withWristRollRef(base);
  assert.match(fixed, /name="wrist_roll"[^>]*ref="/, 'ref not applied');
  assert.equal(withWristRollRef(fixed), fixed, 'applying twice must not stack');

  // It has to survive compilation and land on the right joint, not just the text.
  const mj = await loadMujocoModule();
  const files: ModelFiles = {};
  files['scene.xml'] = new TextEncoder().encode(buildBoardSceneXml(fixed, { physics: false }));
  for (const f of readdirSync(join(MODEL_DIR, 'assets'))) {
    if (f.endsWith('.stl')) files[`assets/${f}`] = new Uint8Array(readFileSync(join(MODEL_DIR, 'assets', f)));
  }
  const { model } = mountModel(mj, files, 'scene.xml');
  const jid = mj.mj_name2id(model, 3 /* mjOBJ_JOINT */, 'wrist_roll');
  assert.ok(jid >= 0, 'wrist_roll joint missing');
  const adr = (model.jnt_qposadr as Int32Array)[jid];
  // MuJoCo defines the joint's rotation as qpos − ref and sets qpos0 = ref, so the
  // model rests in its authored pose while a commanded 0 now draws the wrist 90°
  // from where it used to — which is the whole point. (jnt_ref isn't exposed by
  // this WASM binding; qpos0 is what it lands in.)
  assert.ok(
    Math.abs((model.qpos0 as Float64Array)[adr] - WRIST_ROLL_REF) < 1e-6,
    'the reference did not reach the compiled model',
  );
});

test('every station turns its nav tag back toward the robot start', async () => {
  // A tag facing away is invisible from the start point however long the robot
  // spins, so this is the property that makes one opening sweep enough.
  const { STATIONS, stationTagWorldFor, stationYaw } = await import('../src/lib/boardSim');
  for (const s of STATIONS) {
    const p = stationTagWorldFor(s);
    const yaw = stationYaw(s);
    const n = [Math.cos(yaw), Math.sin(yaw)];
    const facing = n[0] * p[0] + n[1] * p[1]; // normal · (tag − origin)
    if (s.discoverable === false) {
      // Deliberately hidden: turned away, so a sweep from the start finds nothing.
      assert.ok(facing > 0.1, `station ${s.navTag} should be hidden but faces the start`);
    } else {
      assert.ok(facing < -0.1, `station ${s.navTag} tag is turned away (dot ${facing.toFixed(3)})`);
    }
  }
});

test('stations are far enough apart not to overlap', async () => {
  const { STATIONS, STATION_HALF } = await import('../src/lib/boardSim');
  for (let i = 0; i < STATIONS.length; i++) {
    for (let j = i + 1; j < STATIONS.length; j++) {
      const a = STATIONS[i], b = STATIONS[j];
      const d = Math.hypot(a.x - b.x, a.y - b.y);
      // Two axis-aligned squares of half-width STATION_HALF need 2·half between
      // centres in the worst case; a diagonal offset needs less, so this is the
      // conservative bar.
      assert.ok(d > STATION_HALF * 2.4, `stations ${a.navTag} and ${b.navTag} are ${d.toFixed(2)} m apart`);
    }
  }
});

test('nav tags and prop tags are unique and in their id ranges', async () => {
  const { STATIONS } = await import('../src/lib/boardSim');
  const nav = STATIONS.map((s) => s.navTag);
  const props = STATIONS.filter((s) => s.propTag).map((s) => s.propTag);
  assert.equal(new Set(nav).size, nav.length, 'duplicate nav tag');
  assert.equal(new Set(props).size, props.length, 'duplicate prop tag');
  // 200-250 is what the app auto-offers as a place; 100-199 is what it treats as
  // a pickable object. A tag in the wrong band silently stops being either.
  for (const id of nav) assert.ok(id >= 200 && id <= 250, `nav tag ${id} out of range`);
  for (const id of props) assert.ok(id >= 100 && id <= 199, `prop tag ${id} out of range`);
});

test('prop markers sit clear of the props they are stuck to', async () => {
  // A marker sunk into the apple it is meant to be sitting on still renders as a
  // square from directly above, so this is invisible until the detector quietly
  // fails on a half-buried tag. Checked against the compiled model, where every
  // geom's world position and size are whatever MuJoCo actually made of them.
  const mj = await loadMujocoModule();
  const files: ModelFiles = {};
  const base = readFileSync(join(MODEL_DIR, XML), 'utf8');
  files['scene.xml'] = new TextEncoder().encode(buildBoardSceneXml(base, { physics: false }));
  for (const f of readdirSync(join(MODEL_DIR, 'assets'))) {
    if (f.endsWith('.stl')) files[`assets/${f}`] = new Uint8Array(readFileSync(join(MODEL_DIR, 'assets', f)));
  }
  const { model, data } = mountModel(mj, files, 'scene.xml');
  mj.mj_forward(model, data);
  const { STATIONS } = await import('../src/lib/boardSim');
  const PROP_TAG_HALF = 0.0075; // 15 mm marker, as built
  const xpos = data.geom_xpos as Float64Array;
  const size = model.geom_size as Float64Array;
  const type = model.geom_type as Int32Array;

  for (const s of STATIONS) {
    if (!s.propTag || s.prop === 'block' || s.prop === 'bottle') continue; // flat-topped or side-tagged
    const tag = mj.mj_name2id(model, 5, `ptag_${s.propTag}`);
    assert.ok(tag >= 0, `no marker for station ${s.navTag}`);
    const tagZ = xpos[tag * 3 + 2];
    // Every solid piece of the prop must top out below the marker.
    const parts = [`${s.prop}_${s.navTag}`, `${s.prop}_${s.navTag}_0`, `${s.prop}_${s.navTag}_1`,
                   `${s.prop}_${s.navTag}_2`, `apple_stalk_${s.navTag}`];
    for (const name of parts) {
      const gid = mj.mj_name2id(model, 5, name);
      if (gid < 0) continue;
      // Highest point: centre plus the vertical half-extent. Capsules and
      // spheres carry their radius in size[0]; ellipsoids and boxes use size[2].
      const t = type[gid];
      const rad = size[gid * 3];
      const half = t === 3 /* capsule */ || t === 2 /* sphere */ ? rad : size[gid * 3 + 2];
      // Only parts that overlap the marker's footprint can bury or occlude it —
      // a stalk leaning away beside the tag is exactly what a real apple has.
      const dx = xpos[gid * 3] - xpos[tag * 3];
      const dy = xpos[gid * 3 + 1] - xpos[tag * 3 + 1];
      const clearance = Math.hypot(dx, dy) - rad;
      if (clearance > PROP_TAG_HALF * 1.5) continue;
      const topOf = xpos[gid * 3 + 2] + half;
      assert.ok(
        topOf <= tagZ + 1e-6,
        `${name} reaches ${topOf.toFixed(4)} m over the marker at ${tagZ.toFixed(4)} m`,
      );
    }
  }
});

test('station bodies compile to the yaw they were given', async () => {
  // Guards an angle-unit mistake, which is invisible in the XML and catastrophic
  // in the scene: the model sets <compiler angle="radian">, so a yaw written in
  // degrees is read as that many radians. 163° became 163 rad ≡ 350°, spinning
  // each pedestal round so its nav tag faced away and was never detected.
  const mj = await loadMujocoModule();
  const files: ModelFiles = {};
  const base = readFileSync(join(MODEL_DIR, XML), 'utf8');
  files['scene.xml'] = new TextEncoder().encode(buildBoardSceneXml(base, { physics: false }));
  for (const f of readdirSync(join(MODEL_DIR, 'assets'))) {
    if (f.endsWith('.stl')) files[`assets/${f}`] = new Uint8Array(readFileSync(join(MODEL_DIR, 'assets', f)));
  }
  const { model, data } = mountModel(mj, files, 'scene.xml');
  mj.mj_forward(model, data);
  const { STATIONS, stationYaw } = await import('../src/lib/boardSim');
  const xmat = data.xmat as Float64Array;

  for (const s of STATIONS) {
    const bid = mj.mj_name2id(model, 1 /* mjOBJ_BODY */, `station_${s.navTag}`);
    assert.ok(bid >= 0);
    // Row-major 3×3; the body's local +X in world is its first column.
    const ax = xmat[bid * 9], ay = xmat[bid * 9 + 3];
    const got = Math.atan2(ay, ax);
    const want = stationYaw(s);
    const err = Math.abs(Math.atan2(Math.sin(got - want), Math.cos(got - want)));
    // 1e-4, not tighter: the yaw is written into the XML at 5 decimal places.
    assert.ok(err < 1e-4, `station ${s.navTag} compiled to ${(got * 180 / Math.PI).toFixed(1)}°, wanted ${(want * 180 / Math.PI).toFixed(1)}°`);

    // And the consequence that actually matters: whether the tagged face looks at
    // the origin, which is what decides if an opening sweep offers it.
    const facing = ax * (s.x + ax * 0.12) + ay * (s.y + ay * 0.12);
    if (s.discoverable === false) {
      assert.ok(facing > 0.1, `station ${s.navTag} should be hidden but faces the start`);
    } else {
      assert.ok(facing < -0.1, `station ${s.navTag} nav tag faces away (dot ${facing.toFixed(3)})`);
    }
  }
});

test('everything meant to be worked with is within reach of the start', async () => {
  // The pickable props and the basket have to be somewhere the robot can drive to
  // and find; the not-yet-pickable ones are deliberately set out beyond them.
  const { STATIONS } = await import('../src/lib/boardSim');
  for (const s of STATIONS) {
    const d = Math.hypot(s.x, s.y);
    if (s.discoverable === false) {
      assert.ok(d > 1.5, `hidden station ${s.navTag} is only ${d.toFixed(2)} m out — put it further`);
    } else {
      assert.ok(d < 1.2, `station ${s.navTag} is ${d.toFixed(2)} m away, too far to be handy`);
    }
  }
});

test('the workable stations are evenly spaced on their ring', async () => {
  // "Equidistant" is impossible for five points in a plane — only three can be.
  // A regular pentagon is what neat means here: same distance from the robot,
  // same distance between neighbours.
  const { STATIONS, RING_SPACING } = await import('../src/lib/boardSim');
  const ring = STATIONS.filter((s) => s.discoverable !== false);
  assert.equal(ring.length, 5, 'ring size changed — update RING_N with it');
  const radii = ring.map((s) => Math.hypot(s.x, s.y));
  for (const r of radii) {
    assert.ok(Math.abs(r - radii[0]) < 1e-9, `station off the ring at ${r.toFixed(3)} m`);
  }
  // Sorted by bearing, consecutive stations must be one chord apart.
  const byAngle = [...ring].sort((a, b) => Math.atan2(a.y, a.x) - Math.atan2(b.y, b.x));
  for (let i = 0; i < byAngle.length; i++) {
    const a = byAngle[i], b = byAngle[(i + 1) % byAngle.length];
    const d = Math.hypot(a.x - b.x, a.y - b.y);
    assert.ok(
      Math.abs(d - RING_SPACING) < 1e-6,
      `${a.navTag}→${b.navTag} is ${d.toFixed(3)} m, not the ${RING_SPACING.toFixed(3)} m step`,
    );
  }
});
