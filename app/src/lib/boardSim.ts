// Board ⇄ simulator coordinate mapping and scene injection — ports the pieces of
// run_real_ik.py that place the calibration board and the detected block into the
// MuJoCo world. Camera detection gives a block position in interior-board mm;
// `interiorToSim` maps that to the robot's world frame, and `buildBoardSceneXml`
// paints the board (surface, tags, base line) and a movable block into the model.

import { boardTagCentres } from './board';

// Board geometry (must match calibrate_board.py / run_real_ik.py defaults).
export const SQUARE_MM = 180.0;
export const TAG_MM = 16.0;
export const GAP_MM = 2.0;
export const N_OUTER = 10;
export const N_INNER = 8;
export const INSET_MM = 2 * TAG_MM + GAP_MM; // 34 mm

// Robot ↔ board placement.
export const BOARD_OFFSET_MM = 70.0; // board bottom edge → robot base tip
export const ROBOT_INT_X = 56.0; // interior mm, lateral centre of robot base
export const ROBOT_INT_Y = SQUARE_MM - INSET_MM + BOARD_OFFSET_MM; // 216 mm
export const ROBOT_BASE_TIP_X = 0.055; // sim metres, front face of base

// Real jenga block: 75 × 25 × 15 mm. Long axis along interior Y = sim X.
export const BLOCK_HALF_X = 0.0375;
export const BLOCK_HALF_Y = 0.0125;
export const BLOCK_HALF_Z = 0.0075;

/** Corrected interior-board position (mm) → sim world position (m). */
export function interiorToSim(intXmm: number, intYmm: number): [number, number, number] {
  const simX = ROBOT_BASE_TIP_X + (ROBOT_INT_Y - intYmm) / 1000.0;
  const simY = (ROBOT_INT_X - intXmm) / 1000.0;
  return [simX, simY, BLOCK_HALF_Z];
}

/** Board-frame position (mm, origin top-left) → sim world x,y (m). */
export function boardToSim(boardXmm: number, boardYmm: number): [number, number] {
  const simX = ROBOT_BASE_TIP_X + (SQUARE_MM + BOARD_OFFSET_MM - boardYmm) / 1000.0;
  const simY = -(boardXmm - SQUARE_MM / 2) / 1000.0;
  return [simX, simY];
}

// ── The station ──────────────────────────────────────────────────────────────
// A sim stand-in for the real setup: a pedestal with an ArUco board on its top
// face and a big nav tag on one side, so the robot finds it the same way it
// would in the room — spot the side tag from across the floor, drive to it, then
// look down at the board. Deliberately placed behind-right of the robot's start
// pose so it is NOT in the camera's opening view; exploring is what reveals it.
export const STATION_TAG_ID = 200;
export const STATION_H = 0.16; // pedestal height (m) — board sits on top
export const STATION_HALF = 0.12; // pedestal half-width (m)
export const STATION_TAG_HALF = 0.04; // 80 mm nav tag

/**
 * The workable stations stand on a ring around the robot, evenly spaced.
 *
 * Five points cannot all be the same distance from each other — only three can,
 * as an equilateral triangle. A regular pentagon is the nearest thing that reads
 * as even: every station the same distance from the robot, and every neighbour
 * the same distance apart, which is what makes the layout look deliberate.
 *
 * The first sits at −45°, outside the camera's opening view, so finding it still
 * takes a turn rather than being handed over on load.
 */
const RING_R = 0.95;
const RING_START = (-45 * Math.PI) / 180;
const RING_N = 5;
function ringPos(i: number): { x: number; y: number } {
  const a = RING_START + (i * 2 * Math.PI) / RING_N;
  return { x: RING_R * Math.cos(a), y: RING_R * Math.sin(a) };
}
/** Distance between neighbours on the ring (m) — the chord of one step. */
export const RING_SPACING = 2 * RING_R * Math.sin(Math.PI / RING_N);

/** Tag id of the pickable object sitting on the station's board. */
export const STATION_OBJECT_TAG = 101;

/** What sits on a station's board. */
export type PropKind = 'block' | 'apple' | 'orange' | 'banana' | 'bottle' | 'plant' | 'basket';

export interface StationDef {
  /** Nav fiducial on the pedestal's side, used to find and drive to it. */
  navTag: number;
  label?: string;
  description?: string;
  x: number;
  y: number;
  prop: PropKind;
  /** Tag on the prop itself; 0 for props that carry none. */
  propTag: number;
  /**
   * Whether an opening sweep should find it. Default true.
   *
   * False turns the nav tag away from the start point, which is what actually
   * hides a station — distance alone doesn't, since the sweep sees as far as the
   * camera does. Used for the things that can't be picked yet, so they stay out
   * of the way instead of offering themselves as targets that go nowhere.
   */
  discoverable?: boolean;
}

/**
 * The stations, and what is on each.
 *
 * Spread around the robot's start point rather than clustered, so exploring has
 * something to do and each one is approached from a different heading. Every
 * pedestal is turned so its nav tag looks back at the origin — see stationYaw —
 * which is what makes a single opening sweep enough to find all of them.
 */
export const STATIONS: StationDef[] = [
  { navTag: 200, label: 'BLOCK', description: 'Wooden jenga block', ...ringPos(0), prop: 'block', propTag: 101 },
  { navTag: 201, label: 'APPLE', description: 'Red fruit, pick it up', ...ringPos(1), prop: 'apple', propTag: 102 },
  { navTag: 202, label: 'BANANA', description: 'Yellow curved fruit', ...ringPos(2), prop: 'banana', propTag: 103 },
  // Where picked things go. No marker: it is a destination, not a target.
  { navTag: 205, label: 'BASKET', description: 'Drop zone for picked items', ...ringPos(3), prop: 'basket', propTag: 0 },
  { navTag: 206, label: 'ORANGE', description: 'Round citrus fruit', ...ringPos(4), prop: 'orange', propTag: 105 },
  // Bottle and plant: the watering job, which nothing can do yet. Set out beyond
  // the basket on the same line, with their nav tags turned away so an opening
  // sweep doesn't offer them. They are there to be found deliberately, once the
  // sideways-tag grasp exists — not to clutter the list with dead ends.
  { navTag: 203, label: 'BOTTLE', description: 'Water bottle, side tag', x: -0.10, y: -1.95, prop: 'bottle', propTag: 104, discoverable: false },
  { navTag: 204, label: 'PLANT', description: 'Potted plant, water me', x: -0.15, y: -2.75, prop: 'plant', propTag: 0, discoverable: false },
];

/**
 * The yaw that turns a station's tagged face back toward the origin.
 *
 * Derived rather than stored: a tag facing the wrong way is invisible from the
 * start point no matter how long the robot spins, and that is not the kind of
 * mistake worth leaving to a hand-entered angle per station.
 */
export function stationYaw(s: { x: number; y: number; discoverable?: boolean }): number {
  const toOrigin = Math.atan2(-s.y, -s.x);
  return s.discoverable === false ? toOrigin + Math.PI : toOrigin;
}

/**
 * The block's station, under the names the rest of the app already uses. Derived
 * rather than repeated: these decide where the block itself is placed, and a
 * hand-copied duplicate would leave it sitting beside its own pedestal the first
 * time the layout moved.
 */
const BLOCK_STATION = STATIONS.find((s) => s.prop === 'block')!;
export const STATION_X = BLOCK_STATION.x;
export const STATION_Y = BLOCK_STATION.y;
/** Pedestal yaw (rad). Its tagged face looks back toward the start. */
export const STATION_YAW = stationYaw(BLOCK_STATION);

/** World position of a station's side nav tag. */
export function stationTagWorldFor(s: StationDef): [number, number, number] {
  const yaw = stationYaw(s);
  return [
    s.x + Math.cos(yaw) * STATION_HALF,
    s.y + Math.sin(yaw) * STATION_HALF,
    STATION_H - STATION_TAG_HALF - 0.02,
  ];
}

/** World position of the prop resting on a station's board. */
export function stationPropWorld(s: StationDef): [number, number, number] {
  return [s.x, s.y, STATION_H + 0.0015];
}

/**
 * How far from the station's centre the block may be scattered (m) and still sit
 * fully on the board at *any* rotation.
 *
 * The binding case is a 45°-ish yaw, where the block's half-diagonal — not its
 * half-length — is what reaches for the edge. Deriving it keeps the two in step
 * if the board or the block is ever resized.
 */
export const BLOCK_SCATTER_R =
  SQUARE_MM / 2 / 1000 - Math.hypot(BLOCK_HALF_X, BLOCK_HALF_Y);

/** World position of the pickable object's centre, resting on the board top. */
export function stationObjectWorld(): [number, number, number] {
  return [STATION_X, STATION_Y, STATION_H + 0.0015 + BLOCK_HALF_Z];
}

/** World position of the centre of the station's side nav tag. */
export function stationTagWorld(): [number, number, number] {
  const n: [number, number] = [Math.cos(STATION_YAW), Math.sin(STATION_YAW)];
  return [
    STATION_X + n[0] * STATION_HALF,
    STATION_Y + n[1] * STATION_HALF,
    STATION_H - STATION_TAG_HALF - 0.02,
  ];
}

/** Outward normal of that tag face. */
export function stationTagNormal(): [number, number, number] {
  return [Math.cos(STATION_YAW), Math.sin(STATION_YAW), 0];
}

const f = (n: number) => n.toFixed(5);

/** Half-side of a prop's ArUco marker (m), matching what solvePnP is told. */
const PROP_TAG_HALF = 0.0075; // 15 mm marker

/**
 * A flat pad stuck to the top of a rounded prop, with the marker on it.
 *
 * `surfaceZ` is the highest point of the thing it sits on. The pad sinks 2 mm
 * into that surface so it reads as stuck on rather than hovering, and gives the
 * marker somewhere flat to live: a square tag laid straight onto a sphere or a
 * capsule intersects the curve and comes out half-buried, which is both wrong to
 * look at and wrong to measure. It stands in for the blob of clay you would use
 * to do this on a real apple.
 */
function padTagGeoms(name: string, surfaceZ: number, padR = 0.016, tagYaw = 0): string[] {
  const padHalf = 0.003;
  const padZ = surfaceZ - 0.002 + padHalf;
  const topZ = padZ + padHalf;
  // The marker's own yaw decides which way the jaws line up: the grasp takes an
  // axis from the tag's rotation, so on a long object the tag has to be turned to
  // put that axis across the short side rather than along the length.
  const rot = tagYaw ? ` euler="0 0 ${f(tagYaw)}"` : '';
  return [
    `<geom name="${name}_pad" type="cylinder" pos="0 0 ${f(padZ)}" ` +
      `size="${f(padR)} ${f(padHalf)}" rgba="0.90 0.88 0.84 1" ` +
      `contype="0" conaffinity="0" group="1"/>`,
    // White backing then the marker just proud of it — the detector needs light
    // around the black square to find it at all.
    `<geom name="${name}_face" type="box" pos="0 0 ${f(topZ + 0.0003)}"${rot} ` +
      `size="${f(PROP_TAG_HALF * 1.5)} ${f(PROP_TAG_HALF * 1.5)} 0.0004" ` +
      `rgba="0.97 0.97 0.97 1" contype="0" conaffinity="0" group="1"/>`,
    `<geom name="${name}" type="box" pos="0 0 ${f(topZ + 0.0009)}"${rot} ` +
      `size="${f(PROP_TAG_HALF)} ${f(PROP_TAG_HALF)} 0.0002" ` +
      `rgba="0.15 0.15 0.15 1" contype="0" conaffinity="0" group="1"/>`,
  ];
}


/** Props that are their own free body, so the arm can actually carry them off. */
export const FREE_PROPS: PropKind[] = ['apple', 'orange', 'banana'];

/**
 * The apple or banana, built about z = 0 — the surface it rests on.
 *
 * Emitted into a free body rather than into the station, because a prop welded to
 * the scenery can be approached and gripped and still never move. Sizes are
 * chosen to fit between the jaws: an apple the size of a real one is wider than
 * the gripper opens, and a target that cannot be closed on is not a test of
 * anything.
 */
function fruitGeoms(s: StationDef, physics: boolean): string[] {
  const g: string[] = [];
  const solid = physics
    ? 'contype="3" conaffinity="3" friction="1.4 0.02 0.001"'
    : 'contype="0" conaffinity="0" group="1"';
  const mass = physics ? ' mass="0.05"' : '';
  const n = s.navTag;
  if (s.prop === 'apple' || s.prop === 'orange') {
    const round = s.prop === 'orange';
    const r = round ? 0.028 : 0.026;
    const rz = round ? 0.026 : 0.024;
    const skin = round ? '0.95 0.55 0.10 1' : '0.80 0.15 0.12 1';
    const apex = rz * 2;
    g.push(
      // Flat facet underneath. A round body on a flat surface is free to roll, and
      // a rolled fruit carries its marker round with it — face-down, the pick has
      // nothing to aim at. Real fruit sits on a flattened base for the same reason.
      `<geom name="${s.prop}_base_${n}" type="box" pos="0 0 0.004" ` +
        `size="${f(r * 0.55)} ${f(r * 0.55)} 0.004" rgba="${skin}"${mass} ${solid}/>`,
      `<geom name="${s.prop}_${n}" type="ellipsoid" pos="0 0 ${f(rz)}" ` +
        `size="${f(r)} ${f(r)} ${f(rz)}" rgba="${skin}"${mass} ${solid}/>`,
      `<geom name="${s.prop}_stalk_${n}" type="capsule" ` +
        `fromto="${f(r * 0.62)} 0 ${f(apex - 0.008)} ${f(r * 0.86)} 0 ${f(apex + 0.012)}" ` +
        `size="0.003" rgba="0.35 0.25 0.12 1" contype="0" conaffinity="0" group="1"/>`,
    );
    // A sphere has no long axis, so the tag's yaw makes no difference to the grip.
    g.push(...padTagGeoms(`ptag_${s.propTag}`, apex, 0.014));
  } else {
    const y0 = -0.045, r = 0.014, seg = 0.030;
    let apex = 0;
    // Same reason as the apple, and it matters more here: an arc of capsules rests
    // on two round points and rocks about its length at the slightest contact,
    // which was turning the banana over before the run even started.
    g.push(
      `<geom name="banana_base_${n}" type="box" pos="0 ${f(y0 + seg * 1.5)} 0.004" ` +
        `size="${f(r * 0.8)} ${f(seg * 1.35)} 0.004" rgba="0.92 0.80 0.18 1"${mass} ${solid}/>`,
    );
    for (let i = 0; i < 3; i++) {
      const a0 = y0 + i * seg, a1 = y0 + (i + 1) * seg;
      const z0 = r + 0.010 * Math.sin((i / 3) * Math.PI);
      const z1 = r + 0.010 * Math.sin(((i + 1) / 3) * Math.PI);
      apex = Math.max(apex, z0 + r, z1 + r);
      g.push(
        `<geom name="banana_${n}_${i}" type="capsule" ` +
          `fromto="0 ${f(a0)} ${f(z0)} 0 ${f(a1)} ${f(z1)}" size="${f(r)}" ` +
          `rgba="0.92 0.80 0.18 1"${mass} ${solid}/>`,
      );
    }
    // Turned a quarter turn: the fruit runs along local Y, and the grasp lines the
    // jaws up with the tag's Y axis — unturned, that closes them along the length
    // of the banana instead of across it.
    g.push(...padTagGeoms(`ptag_${s.propTag}`, apex, 0.012, Math.PI / 2));
  }
  return g;
}

/** Every free-body prop, as world-frame bodies. */
function freePropBodies(physics: boolean): string {
  const out: string[] = [];
  for (const s of STATIONS) {
    if (!FREE_PROPS.includes(s.prop)) continue;
    const [x, y, z] = stationPropWorld(s);
    const yaw = stationYaw(s);
    out.push(
      `<body name="prop_${s.navTag}" pos="${f(x)} ${f(y)} ${f(z)}" euler="0 0 ${f(yaw)}">` +
        (physics ? `<freejoint name="prop_${s.navTag}_free"/>` : '') +
        fruitGeoms(s, physics).join('') +
        `</body>`,
    );
  }
  return out.join('\n    ');
}

/**
 * The props, built from MuJoCo primitives rather than imported meshes.
 *
 * A sphere is an apple as far as this simulation is concerned: what the pick
 * needs from an object is a place to put the jaws and a tag to aim at, and a
 * primitive gives exact collision geometry for a few bytes where a downloaded
 * mesh gives approximate collision for megabytes. It also keeps the scene free of
 * third-party assets and the licence tracking that comes with them.
 */
function propGeoms(s: StationDef, physics: boolean): string[] {
  const g: string[] = [];
  const solid = physics ? 'contype="3" conaffinity="3"' : 'contype="0" conaffinity="0" group="1"';
  const decor = 'contype="0" conaffinity="0" group="1"';
  const n = s.navTag;
  const top = STATION_H + 0.0015;
  switch (s.prop) {
    case 'bottle': {
      // Upright cylinder. Its tag is on the *side*, standing vertical — the whole
      // point of this one, and why it is not yet pickable: every grasp so far has
      // come down onto a tag lying flat, and this one faces sideways.
      const r = 0.035, h = 0.11;
      g.push(
        `<geom name="bottle_${n}" type="cylinder" pos="0 0 ${f(top + h / 2)}" ` +
          `size="${f(r)} ${f(h / 2)}" rgba="0.30 0.55 0.85 0.85" ${solid}/>`,
        `<geom name="bottle_cap_${n}" type="cylinder" pos="0 0 ${f(top + h + 0.012)}" ` +
          `size="0.014 0.012" rgba="0.90 0.90 0.92 1" ${decor}/>`,
        `<geom name="bottle_neck_${n}" type="cylinder" pos="0 0 ${f(top + h + 0.001)}" ` +
          `size="0.016 0.010" rgba="0.30 0.55 0.85 0.85" ${decor}/>`,
      );
      // Marker wrapped on the curved face, turned to look back at the robot.
      const tz = top + h * 0.55;
      g.push(
        `<geom name="ptag_${s.propTag}_face" type="box" pos="${f(r + 0.0005)} 0 ${f(tz)}" ` +
          `size="0.0005 ${f(PROP_TAG_HALF * 1.5)} ${f(PROP_TAG_HALF * 1.5)}" ` +
          `rgba="0.97 0.97 0.97 1" ${decor}/>`,
        `<geom name="ptag_${s.propTag}" type="box" pos="${f(r + 0.0015)} 0 ${f(tz)}" ` +
          `size="0.0005 ${f(PROP_TAG_HALF)} ${f(PROP_TAG_HALF)}" ` +
          `rgba="0.15 0.15 0.15 1" ${decor}/>`,
      );
      break;
    }
    case 'plant': {
      // A potted plant: the thing to be watered, so it carries no pick tag.
      const pr = 0.045, ph = 0.055;
      g.push(
        `<geom name="pot_${n}" type="cylinder" pos="0 0 ${f(top + ph / 2)}" ` +
          `size="${f(pr)} ${f(ph / 2)}" rgba="0.62 0.33 0.22 1" ${solid}/>`,
        `<geom name="soil_${n}" type="cylinder" pos="0 0 ${f(top + ph - 0.004)}" ` +
          `size="${f(pr * 0.86)} 0.006" rgba="0.22 0.16 0.11 1" ${decor}/>`,
      );
      const base = top + ph;
      for (let i = 0; i < 5; i++) {
        const a = (i / 5) * Math.PI * 2;
        const lx = Math.cos(a) * 0.045, ly = Math.sin(a) * 0.045;
        g.push(
          `<geom name="stem_${n}_${i}" type="capsule" ` +
            `fromto="0 0 ${f(base)} ${f(lx * 0.5)} ${f(ly * 0.5)} ${f(base + 0.07)}" ` +
            `size="0.004" rgba="0.18 0.45 0.16 1" ${decor}/>`,
          `<geom name="leaf_${n}_${i}" type="ellipsoid" ` +
            `pos="${f(lx * 0.7)} ${f(ly * 0.7)} ${f(base + 0.085)}" ` +
            `size="0.030 0.018 0.006" euler="0 0.35 ${f(a)}" ` +
            `rgba="0.20 0.55 0.18 1" ${decor}/>`,
        );
      }
      break;
    }
    case 'basket': {
      // An open box to drop things into: floor plus four low walls.
      const hw = 0.085, wall = 0.006, h = 0.055;
      const wallGeom = (tag: string, px: number, py: number, sx: number, sy: number) =>
        `<geom name="basket_${n}_${tag}" type="box" ` +
        `pos="${f(px)} ${f(py)} ${f(top + h / 2)}" size="${f(sx)} ${f(sy)} ${f(h / 2)}" ` +
        `rgba="0.55 0.38 0.20 1" ${solid}/>`;
      g.push(
        `<geom name="basket_${n}_floor" type="box" pos="0 0 ${f(top + wall)}" ` +
          `size="${f(hw)} ${f(hw)} ${f(wall)}" rgba="0.50 0.34 0.18 1" ${solid}/>`,
        wallGeom('nx', -hw + wall, 0, wall, hw),
        wallGeom('px', hw - wall, 0, wall, hw),
        wallGeom('ny', 0, -hw + wall, hw, wall),
        wallGeom('py', 0, hw - wall, hw, wall),
      );
      break;
    }
    case 'apple':
    case 'orange':
    case 'banana':
    case 'block':
      break; // free bodies, injected separately
  }
  return g;
}

/** Every pedestal, its board top, its side nav tag, and whatever stands on it. */
function stationGeoms(physics: boolean): string[] {
  const g: string[] = [];
  const boardHalf = SQUARE_MM / 2 / 1000;
  const tagHalf = TAG_MM / 2 / 1000;
  for (const s of STATIONS) {
    const n = s.navTag;
    // Radians, not degrees: the model sets <compiler angle="radian">, so a value
    // in degrees is read as that many radians. 163° became 163 rad ≡ 350°, which
    // spun the pedestal round and pointed its nav tag away from the robot — the
    // tags were never detected because the marker was on the far side.
    const yaw = stationYaw(s);
    // A body lets everything below be written in the station's own frame.
    g.push(`<body name="station_${n}" pos="${f(s.x)} ${f(s.y)} 0" euler="0 0 ${f(yaw)}">`);
    g.push(
      `<geom name="station_pedestal_${n}" type="box" pos="0 0 ${f(STATION_H / 2)}" ` +
        `size="${f(STATION_HALF)} ${f(STATION_HALF)} ${f(STATION_H / 2)}" ` +
        `rgba="0.35 0.36 0.40 1" ` +
        // Channel 3 in physics mode, so it collides with both the props (which
        // rest on it) and the arm (1 & 3 is non-zero). The body has no joint, so
        // it is immovable: the arm stops against it rather than pushing it or
        // passing through, which is what a real pedestal does.
        (physics ? 'contype="3" conaffinity="3"' : 'contype="0" conaffinity="0" group="1"') +
        `/>`,
    );
    // Board on the top face.
    g.push(
      `<geom name="station_board_${n}" type="box" pos="0 0 ${f(STATION_H + 0.0005)}" ` +
        `size="${f(boardHalf)} ${f(boardHalf)} 0.0005" rgba="0.85 0.80 0.65 1" ` +
        `contype="0" conaffinity="0" group="1"/>`,
    );
    // The calibration border only goes on the first station: the tag ids are
    // fixed, and repeating them would put the same marker in several places.
    if (n === STATION_TAG_ID) {
      for (const [id, [bx, by]] of boardTagCentres(SQUARE_MM, TAG_MM, GAP_MM, N_OUTER, N_INNER)) {
        const lx = (bx - SQUARE_MM / 2) / 1000;
        const ly = -(by - SQUARE_MM / 2) / 1000;
        g.push(
          `<geom name="stag_${id}" type="box" pos="${f(lx)} ${f(ly)} ${f(STATION_H + 0.0012)}" ` +
            `size="${f(tagHalf)} ${f(tagHalf)} 0.0002" rgba="0.15 0.15 0.15 1" ` +
            `contype="0" conaffinity="0" group="1"/>`,
        );
      }
    }
    // The dual nav tag card on the +X side, which stationYaw has turned toward the origin.
    const tz = STATION_H - 0.045;
    // White sheet backing plate for the entire card (160mm wide x 60mm high)
    g.push(
      `<geom name="station_navtag_face_${n}" type="box" ` +
        `pos="${f(STATION_HALF + 0.0005)} 0 ${f(tz)}" ` +
        `size="0.0005 0.080 0.030" ` +
        `rgba="1 1 1 1" contype="0" conaffinity="0" group="1"/>`,
    );
    // Left marker (Tag 200) - size 40mm x 40mm (half 0.020), on -Y side (viewer left)
    g.push(
      `<geom name="station_navtag_${n}" type="box" ` +
        `pos="${f(STATION_HALF + 0.0015)} -0.055 ${f(tz)}" ` +
        `size="0.0005 0.020 0.020" ` +
        `rgba="1 1 1 1" contype="0" conaffinity="0" group="1"/>`,
    );
    // Right marker (Tag 201) - size 40mm x 40mm (half 0.020), on +Y side (viewer right)
    g.push(
      `<geom name="station_navtag_right_${n}" type="box" ` +
        `pos="${f(STATION_HALF + 0.0015)} 0.055 ${f(tz)}" ` +
        `size="0.0005 0.020 0.020" ` +
        `rgba="1 1 1 1" contype="0" conaffinity="0" group="1"/>`,
    );
    // Middle text plate carrying the station's label (top) and description (bottom)
    g.push(
      `<geom name="station_label_${n}" type="box" ` +
        `pos="${f(STATION_HALF + 0.0015)} 0 ${f(tz)}" ` +
        `size="0.0005 0.030 0.020" ` +
        `rgba="1 1 1 1" contype="0" conaffinity="0" group="1"/>`,
    );
    g.push(...propGeoms(s, physics));
    g.push('</body>');
  }
  return g;
}

/**
 * Inject the board (decorative geoms) and a mocap block body into a model's
 * `<worldbody>`. The block is a mocap body so its pose is set kinematically via
 * `data.mocap_pos` — no physics/free joint needed. Returns the modified XML.
 */
export function buildBoardSceneXml(
  baseXml: string,
  opts: { physics?: boolean; centreBoard?: boolean } = {},
): string {
  const tagHalf = TAG_MM / 2 / 1000;
  const boardHalf = SQUARE_MM / 2 / 1000;
  const intHalf = (SQUARE_MM / 2 - INSET_MM) / 1000;
  const [boardCx, boardCy] = boardToSim(SQUARE_MM / 2, SQUARE_MM / 2);

  // The flat calibration board at the origin. Off by default: it's a calibration
  // fixture, not scenery, and it clutters the view of the station.
  const geoms: string[] = [];
  if (opts.centreBoard) {
  geoms.push(
    `<geom name="board_surface" type="box" pos="${f(boardCx)} ${f(boardCy)} -0.0004" ` +
      `size="${f(boardHalf)} ${f(boardHalf)} 0.0004" rgba="0.85 0.80 0.65 1" ` +
      `contype="0" conaffinity="0" group="1"/>`,
  );
  geoms.push(
    `<geom name="board_interior" type="box" pos="${f(boardCx)} ${f(boardCy)} 0.00005" ` +
      `size="${f(intHalf)} ${f(intHalf)} 0.00005" rgba="0.92 0.88 0.72 1" ` +
      `contype="0" conaffinity="0" group="1"/>`,
  );
  const tags = boardTagCentres(SQUARE_MM, TAG_MM, GAP_MM, N_OUTER, N_INNER);
  for (const [id, [bx, by]] of tags) {
    const [sx, sy] = boardToSim(bx, by);
    geoms.push(
      `<geom name="btag_${id}" type="box" pos="${f(sx)} ${f(sy)} 0.0001" ` +
        `size="${f(tagHalf)} ${f(tagHalf)} 0.0001" rgba="0.15 0.15 0.15 1" ` +
        `contype="0" conaffinity="0" group="1"/>`,
    );
  }
  geoms.push(
    `<geom name="base_tip_line" type="box" pos="${f(ROBOT_BASE_TIP_X)} 0 0.0003" ` +
      `size="0.0008 ${f(boardHalf)} 0.0003" rgba="1 0.15 0.15 1" ` +
      `contype="0" conaffinity="0" group="1"/>`,
  );
  }

  // The block. In physics mode it's a real free body (falls, rests, can be
  // pushed and grasped); otherwise a mocap body whose pose mirrors the detected
  // real block. Same 75×25×15 mm box either way.
  const op = stationObjectWorld();
  const blockPos = `${f(op[0])} ${f(op[1])} ${f(op[2])}`;
  const blockSize = `${f(BLOCK_HALF_X)} ${f(BLOCK_HALF_Y)} ${f(BLOCK_HALF_Z)}`;
  // Physics block + floor share collision channel 2, isolated from the arm
  // (channel 1): the block rests on the floor and is held by a weld-attach on
  // grasp, so it never fights finger contacts or the arm base.
  // A dark square on the object's top face standing in for its printed ArUco tag,
  // so what the arm camera is looking for is visible in the sim too.
  const objTagHalf = Math.min(BLOCK_HALF_X, BLOCK_HALF_Y) * 0.72;
  const objTag =
    // White backing first — the detector needs a light quiet zone around the
    // black border, and the block itself is red.
    `<geom name="block_tag_face" type="box" pos="0 0 ${f(BLOCK_HALF_Z + 0.0002)}" ` +
    `size="${f(objTagHalf * 1.35)} ${f(objTagHalf * 1.35)} 0.0002" rgba="0.97 0.97 0.97 1" ` +
    `contype="0" conaffinity="0" group="1"/>` +
    `<geom name="block_tag" type="box" pos="0 0 ${f(BLOCK_HALF_Z + 0.0006)}" ` +
    `size="${f(objTagHalf)} ${f(objTagHalf)} 0.0002" rgba="0.12 0.12 0.12 1" ` +
    `contype="0" conaffinity="0" group="1"/>`;
  const block = opts.physics
    ? `<body name="block" pos="${blockPos}"><freejoint name="block_free"/>` +
      `<geom name="block_geom" type="box" size="${blockSize}" mass="0.02" ` +
      // Channel 3 = 1 | 2: collides with the arm (channel 1) so the fingers can
      // actually grip it, and with the floor and station (channel 2) so it rests
      // on them. It used to be on 2 alone, isolated from the arm, and was held by
      // pinning it to the gripper instead of being gripped.
      `rgba="0.90 0.30 0.20 1" contype="3" conaffinity="3" friction="1.4 0.02 0.001"/>${objTag}</body>`
    : `<body name="block" mocap="true" pos="${blockPos}">` +
      `<geom name="block_geom" type="box" size="${blockSize}" ` +
      `rgba="0.90 0.30 0.20 1" contype="0" conaffinity="0" group="1"/>${objTag}</body>`;

  const floor = opts.physics
    ? `<geom name="floor" type="plane" size="0 0 0.05" pos="0 0 0" contype="2" conaffinity="3" ` +
      // Collision only: the renderer draws its own wooden floor, and this grey
      // plane sat on top of it. Group 3 keeps the block resting on something
      // without drawing anything.
      `rgba="0.12 0.13 0.15 1" group="3"/>`
    : '';

  const station = stationGeoms(!!opts.physics).join('\n    ');

  // The real wrist-cam bracket (SO-ARM100 Optional/Wrist_Cam_Mount_32x32_UVC_Module,
  // with the fused gripper jaw trimmed off so it doesn't draw a second jaw over the
  // arm's own). Mocap, so it can ride the gripper frame without being welded in.
  const mount =
    `<body name="wrist_cam_mount" mocap="true" pos="0 0 -5">` +
    `<geom name="wrist_cam_mount_geom" type="mesh" mesh="wrist_cam_mount" ` +
    `rgba="0.55 0.57 0.62 1" contype="0" conaffinity="0" group="1"/></body>`;
  const injection =
    `\n    ${geoms.join('\n    ')}\n    ${station}\n    ${mount}\n    ` +
    `${floor}\n    ${block}\n    ${freePropBodies(!!opts.physics)}\n  </worldbody>`;
  const idx = baseXml.lastIndexOf('</worldbody>');
  if (idx < 0) throw new Error('No </worldbody> in base model XML');
  let out = baseXml.slice(0, idx) + injection + baseXml.slice(idx + '</worldbody>'.length);

  // STL is in millimetres, like the arm's own meshes.
  const meshDecl =
    '<mesh name="wrist_cam_mount" file="wrist_cam_mount.stl" scale="0.001 0.001 0.001"/>\n  </asset>';
  const aidx = out.lastIndexOf('</asset>');
  if (aidx < 0) throw new Error('No </asset> in base model XML');
  out = out.slice(0, aidx) + meshDecl + out.slice(aidx + '</asset>'.length);
  return out;
}

/**
 * The wrist_roll joint's zero in the model sits 90° away from the real servo's
 * zero, so a commanded roll of 0 drew the wrist rotated.
 *
 * This is fixed with the joint's `ref`, which is MuJoCo's own mechanism for
 * exactly this: the joint's rotation is `qpos - ref`, so ref = +π/2 makes qpos 0
 * mean what servo 0 means, and `range` keeps its servo-space meaning. Doing it in
 * the model means physics, IK, the meshes, the wrist camera and the streamed
 * command all see one consistent convention.
 *
 * The alternative — writing a corrected value into qpos just before rendering —
 * was tried and is what produced months of "the camera randomly flips": a render-
 * time edit is invisible to every other reader of those frames.
 */
export const WRIST_ROLL_REF = Math.PI / 2;

export function withWristRollRef(xml: string, ref = WRIST_ROLL_REF): string {
  const re = /<joint([^>]*\bname="wrist_roll"[^>]*)\/>/;
  const m = xml.match(re);
  if (!m) throw new Error('No wrist_roll joint in the model XML');
  if (/\bref=/.test(m[1])) return xml; // already set — leave it alone
  return xml.replace(re, `<joint${m[1]} ref="${ref}"/>`);
}
