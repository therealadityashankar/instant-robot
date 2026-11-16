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
export const STATION_TAG_ID = 201;
export const BOUNDARY_TAG_ID = 240; // Universal stop/end boundary tag (within 6x6_250 dictionary)
export const STATION_H = 0.16; // pedestal height (m) — board sits on top
export const STATION_HALF = 0.12; // pedestal half-width (m)
export const STATION_TAG_HALF = 0.04; // 80 mm nav tag

/**
 * All workable stations are placed in a straight line along the rack/table
 * in front of the robot's driving corridor (at X = 0.85m), with realistic
 * non-equidistant spacing along the Y axis.
 *
 * At the positive end of the line sits the unified Stop / End Tag (Tag 250).
 */
export const LINE_X = 0.85;

/** Tag id of the pickable object sitting on the station's board. */
export const STATION_OBJECT_TAG = 101;

/** What sits on a station's board. */
export type PropKind = 'block' | 'apple' | 'orange' | 'banana' | 'bottle' | 'plant' | 'basket' | 'none';

/** Specialized station archetypes */
export type StationType = 'simple' | 'printer' | 'deposit' | 'modifier' | 'table';

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
  discoverable?: boolean;
  isBoundary?: boolean;
  /** Station archetype / operational mode */
  stationType?: StationType;
  /** Custom STL model filename (e.g. 'models/gear.stl' or 'bracket.stl') */
  stlName?: string;
  /** Current quantity of items available at this station (e.g. for deposit stations) */
  itemCount?: number;
  /** Maximum capacity / replenish target for deposit stations */
  maxItemCount?: number;
  /** Current operational state (e.g. for 3d printer or magic modifier) */
  state?: 'idle' | 'printing' | 'ready' | 'processing';
  /** Output STL for magic modification station */
  outputStlName?: string;
  /** Associated ACT model ID (optional per-station policy) */
  actModelId?: string;
  /** Associated Hugging Face dataset ID (optional) */
  actDatasetId?: string;
}

export interface ActModelDef {
  id: string;
  name: string;
  datasetId: string;
  stationTag?: number;
  description?: string;
}

export const DEFAULT_ACT_MODELS: ActModelDef[] = [
  {
    id: 'lerobot/act_so100_pick_block',
    name: 'Block Precision Pick',
    datasetId: 'lerobot/so100_jenga_pick',
    stationTag: 201,
    description: 'ACT policy trained to pick up wooden blocks from the pedestal',
  },
  {
    id: 'lerobot/act_so100_sort_apple',
    name: 'Apple Sorter',
    datasetId: 'lerobot/so100_apple_sort',
    stationTag: 202,
    description: 'ACT policy for picking and inspecting the apple prop',
  },
  {
    id: 'lerobot/act_so100_banana_grab',
    name: 'Banana Transfer',
    datasetId: 'lerobot/so100_banana_pick',
    stationTag: 203,
    description: 'ACT policy for grasping curved banana prop',
  },
  {
    id: 'lerobot/act_so100_basket_drop',
    name: 'Basket Deposit',
    datasetId: 'lerobot/so100_basket_drop',
    stationTag: 204,
    description: 'ACT policy for safely depositing held objects into payload basket',
  },
];

export type PathPlanAction =
  | 'navigate'
  | 'run_act'
  | 'pick_tag'
  | 'put_basket'
  | 'open_gripper'
  | 'close_gripper'
  | 'wait';

export interface PathPlanStep {
  id: string;
  action: PathPlanAction;
  target?: number | string;
  modelId?: string;
  tagId?: number;
  duration?: number;
  description: string;
  status: 'pending' | 'active' | 'completed' | 'failed';
  error?: string;
}

export interface PathPlan {
  id?: string;
  name: string;
  description?: string;
  steps: PathPlanStep[];
  status: 'idle' | 'running' | 'paused' | 'completed' | 'aborted';
  currentStepIndex: number;
}

export const DEFAULT_MISSIONS: PathPlan[] = [
  {
    id: 'mission-pick-store',
    name: 'Multi-Station Pick & Store',
    description: 'Navigate to Block #201, precision pick with ACT policy, drive to Basket #204 and deposit.',
    status: 'idle',
    currentStepIndex: 0,
    steps: [
      {
        id: 'step-def-1',
        action: 'navigate',
        target: 'BLOCK',
        description: 'Drive to Block station (#201)',
        status: 'pending',
      },
      {
        id: 'step-def-2',
        action: 'run_act',
        modelId: 'lerobot/act_so100_pick_block',
        description: 'Run ACT Model: Block Precision Pick',
        status: 'pending',
      },
      {
        id: 'step-def-3',
        action: 'navigate',
        target: 'BASKET',
        description: 'Drive to Basket zone (#204)',
        status: 'pending',
      },
      {
        id: 'step-def-4',
        action: 'run_act',
        modelId: 'lerobot/act_so100_basket_drop',
        description: 'Run ACT Model: Basket Deposit',
        status: 'pending',
      },
    ],
  },
  {
    id: 'mission-fruit-sort',
    name: 'Fruit Sorting Routine',
    description: 'Drive to Apple station (#202), sort apple prop using ACT, and proceed to Banana (#203).',
    status: 'idle',
    currentStepIndex: 0,
    steps: [
      {
        id: 'step-fruit-1',
        action: 'navigate',
        target: 'APPLE',
        description: 'Drive to Apple station (#202)',
        status: 'pending',
      },
      {
        id: 'step-fruit-2',
        action: 'run_act',
        modelId: 'lerobot/act_so100_sort_apple',
        description: 'Run ACT Model: Apple Sorter',
        status: 'pending',
      },
      {
        id: 'step-fruit-3',
        action: 'navigate',
        target: 'BANANA',
        description: 'Drive to Banana station (#203)',
        status: 'pending',
      },
      {
        id: 'step-fruit-4',
        action: 'run_act',
        modelId: 'lerobot/act_so100_banana_grab',
        description: 'Run ACT Model: Banana Transfer',
        status: 'pending',
      },
    ],
  },
  {
    id: 'mission-corridor-patrol',
    name: 'Full Corridor Patrol',
    description: 'Sweep along the track from START boundary to STOP boundary, inspecting all tags.',
    status: 'idle',
    currentStepIndex: 0,
    steps: [
      {
        id: 'step-patrol-1',
        action: 'navigate',
        target: 'START',
        description: 'Drive to Start post boundary',
        status: 'pending',
      },
      {
        id: 'step-patrol-2',
        action: 'wait',
        duration: 1.5,
        description: 'Scan fiducial anchor tags',
        status: 'pending',
      },
      {
        id: 'step-patrol-3',
        action: 'navigate',
        target: 'STOP',
        description: 'Drive to Stop post boundary',
        status: 'pending',
      },
    ],
  },
];

/**
 * The stations along the straight line, ordered from left (-Y) to right (+Y).
 * Every station features a unique left nav tag (201..499) paired with a universal
 * right anchor tag (200), except the boundary stop tag (250).
 */
export const DEFAULT_STATIONS: StationDef[] = [
  // Start Boundary Tag Post (Left End, single tag without anchor 200)
  { navTag: BOUNDARY_TAG_ID, label: 'START', description: 'Start of station rack', x: LINE_X, y: -1.05, prop: 'plant', propTag: 0, isBoundary: true, stationType: 'simple' },
  { navTag: 201, label: 'BLOCK', description: 'Wooden jenga block', x: LINE_X, y: -0.75, prop: 'block', propTag: 101, stationType: 'simple', actModelId: 'lerobot/act_so100_pick_block', actDatasetId: 'lerobot/so100_jenga_pick' },
  { navTag: 202, label: 'APPLE', description: 'Red fruit, pick it up', x: LINE_X, y: -0.40, prop: 'apple', propTag: 102, stationType: 'simple', actModelId: 'lerobot/act_so100_sort_apple', actDatasetId: 'lerobot/so100_apple_sort' },
  { navTag: 203, label: 'BANANA', description: 'Yellow curved fruit', x: LINE_X, y: -0.05, prop: 'banana', propTag: 103, stationType: 'simple', actModelId: 'lerobot/act_so100_banana_grab', actDatasetId: 'lerobot/so100_banana_pick' },
  // Where picked things go. No marker: it is a destination, not a target.
  { navTag: 204, label: 'BASKET', description: 'Drop zone for picked items', x: LINE_X, y: 0.30, prop: 'basket', propTag: 0, stationType: 'simple', actModelId: 'lerobot/act_so100_basket_drop', actDatasetId: 'lerobot/so100_basket_drop' },
  { navTag: 205, label: 'ORANGE', description: 'Round citrus fruit', x: LINE_X, y: 0.65, prop: 'orange', propTag: 105, stationType: 'simple' },
  // End Boundary Tag Post (Right End, single tag without anchor 200)
  { navTag: BOUNDARY_TAG_ID, label: 'STOP', description: 'End of station rack', x: LINE_X, y: 0.95, prop: 'plant', propTag: 0, isBoundary: true, stationType: 'simple' },
];

export let STATIONS: StationDef[] = [...DEFAULT_STATIONS];

export function setCustomStations(stations: StationDef[]) {
  STATIONS = [...stations];
}

/**
 * The yaw that turns a station's tagged face back toward the robot corridor (along -X).
 */
export function stationYaw(_s?: { x?: number; y?: number; discoverable?: boolean }): number {
  return Math.PI;
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

function printedItemGeoms(s: StationDef, physics: boolean, relZ: number, availableMeshes?: any): string[] {
  const g: string[] = [];
  const solid = physics
    ? 'contype="3" conaffinity="3" friction="1.4 0.02 0.001" mass="0.04"'
    : 'contype="0" conaffinity="0" group="1"';
  const stl = s.outputStlName || s.stlName;
  if (stl && isMeshFileAvailable(stl, availableMeshes)) {
    const meshName = `mesh_st_${s.navTag}`;
    g.push(
      `<geom name="station_custom_stl_${s.navTag}" type="mesh" mesh="${meshName}" ` +
        `pos="0 0 ${f(relZ)}" rgba="0.25 0.72 0.95 1" ${solid}/>`,
    );
  } else {
    g.push(
      `<geom name="printer_${s.navTag}_printed_item" type="box" pos="0 0 ${f(relZ + 0.008)}" ` +
        `size="0.025 0.025 0.008" rgba="0.10 0.80 0.90 1" ${solid}/>`,
    );
  }
  return g;
}

/** Every free-body prop, as world-frame bodies. */
function freePropBodies(physics: boolean, availableMeshes?: any): string {
  const out: string[] = [];
  for (const s of STATIONS) {
    if (FREE_PROPS.includes(s.prop)) {
      const [x, y, z] = stationPropWorld(s);
      const yaw = stationYaw(s);
      out.push(
        `<body name="prop_${s.navTag}" pos="${f(x)} ${f(y)} ${f(z)}" euler="0 0 ${f(yaw)}">` +
          (physics ? `<freejoint name="prop_${s.navTag}_free"/>` : '') +
          fruitGeoms(s, physics).join('') +
          `</body>`,
      );
    } else if (s.stationType === 'printer' && (s.state === 'ready' || !s.state)) {
      const top = STATION_H + 0.0015;
      const pz = top + 0.008;
      out.push(
        `<body name="prop_${s.navTag}" pos="${f(s.x)} ${f(s.y)} ${f(pz)}">` +
          (physics ? `<freejoint name="prop_${s.navTag}_free"/>` : '') +
          printedItemGeoms(s, physics, 0, availableMeshes).join('') +
          `</body>`,
      );
    }
  }
  return out.join('\n    ');
}

export interface BuildSceneOptions {
  physics?: boolean;
  centreBoard?: boolean;
  availableMeshes?: string[] | Set<string> | Record<string, any>;
}

export function isMeshFileAvailable(
  fileName: string | undefined,
  available?: string[] | Set<string> | Record<string, any>,
): boolean {
  if (!fileName) return false;
  if (!available) return false;
  const cleanName = fileName.replace(/^models\//, '');
  if (Array.isArray(available)) {
    return available.some((a) => a === fileName || a === cleanName || a.replace(/^models\//, '') === cleanName);
  }
  if (available instanceof Set) {
    return available.has(fileName) || available.has(cleanName) || available.has(`models/${cleanName}`);
  }
  return !!(available[fileName] || available[cleanName] || available[`models/${cleanName}`]);
}

function customMeshGeoms(s: StationDef, physics: boolean, top: number, availableMeshes?: any, includeTag = true): string[] {
  const g: string[] = [];
  const solid = physics ? 'contype="3" conaffinity="3"' : 'contype="0" conaffinity="0" group="1"';
  const stl = s.outputStlName || s.stlName;
  if (stl && isMeshFileAvailable(stl, availableMeshes)) {
    const meshName = `mesh_st_${s.navTag}`;
    g.push(
      `<geom name="station_custom_stl_${s.navTag}" type="mesh" mesh="${meshName}" ` +
        `pos="0 0 ${f(top + 0.005)}" rgba="0.25 0.72 0.95 1" ${solid}/>`,
    );
  } else {
    // Clean procedural 3D item if custom mesh is not yet uploaded
    g.push(
      `<geom name="station_fallback_item_${s.navTag}" type="box" pos="0 0 ${f(top + 0.016)}" ` +
        `size="0.025 0.025 0.008" rgba="0.10 0.80 0.90 1" ${solid}/>`,
    );
  }
  if (s.propTag && includeTag) {
    g.push(...padTagGeoms(`ptag_${s.propTag}`, top + 0.024, 0.012));
  }
  return g;
}

/**
 * The props and specialized machinery, built from MuJoCo primitives & custom meshes.
 */
function propGeoms(s: StationDef, physics: boolean, availableMeshes?: any): string[] {
  const g: string[] = [];
  const solid = physics ? 'contype="3" conaffinity="3"' : 'contype="0" conaffinity="0" group="1"';
  const decor = 'contype="0" conaffinity="0" group="1"';
  const n = s.navTag;
  const top = STATION_H + 0.0015;
  const stType = s.stationType || 'simple';

  // If station has a custom STL specified
  if (s.stlName && stType === 'simple') {
    return customMeshGeoms(s, physics, top, availableMeshes);
  }

  // 1. 3D Printer Station: Pretend 3D printer with frame, bed, gantry, extruder, nozzle, and LED
  if (stType === 'printer') {
    const fw = 0.065, fh = 0.090, pw = 0.005;
    // 4 vertical frame pillars
    for (const [px, py] of [[-fw, -fw], [fw, -fw], [-fw, fw], [fw, fw]]) {
      g.push(
        `<geom name="printer_${n}_post_${px > 0 ? 'p' : 'n'}${py > 0 ? 'p' : 'n'}" type="box" ` +
          `pos="${f(px)} ${f(py)} ${f(top + fh / 2)}" size="${f(pw)} ${f(pw)} ${f(fh / 2)}" ` +
          `rgba="0.20 0.22 0.26 1" ${decor}/>`,
      );
    }
    // Top crossbars
    g.push(
      `<geom name="printer_${n}_top_x1" type="box" pos="0 ${f(-fw)} ${f(top + fh)}" size="${f(fw)} ${f(pw)} ${f(pw)}" rgba="0.20 0.22 0.26 1" ${decor}/>`,
      `<geom name="printer_${n}_top_x2" type="box" pos="0 ${f(fw)} ${f(top + fh)}" size="${f(fw)} ${f(pw)} ${f(pw)}" rgba="0.20 0.22 0.26 1" ${decor}/>`,
      `<geom name="printer_${n}_top_y1" type="box" pos="${f(-fw)} 0 ${f(top + fh)}" size="${f(pw)} ${f(fw)} ${f(pw)}" rgba="0.20 0.22 0.26 1" ${decor}/>`,
      `<geom name="printer_${n}_top_y2" type="box" pos="${f(fw)} 0 ${f(top + fh)}" size="${f(pw)} ${f(fw)} ${f(pw)}" rgba="0.20 0.22 0.26 1" ${decor}/>`,
    );
    // Heated print bed
    g.push(
      `<geom name="printer_${n}_bed" type="box" pos="0 0 ${f(top + 0.004)}" ` +
        `size="0.055 0.055 0.004" rgba="0.12 0.13 0.15 1" ${solid}/>`,
    );
    // Gantry beam + extruder carriage + nozzle
    const gz = s.state === 'printing' ? top + 0.035 : top + 0.075;
    g.push(
      `<geom name="printer_${n}_gantry" type="box" pos="0 0 ${f(gz)}" ` +
        `size="0.004 ${f(fw)} 0.005" rgba="0.45 0.48 0.52 1" ${decor}/>`,
      `<geom name="printer_${n}_extruder" type="box" pos="0 0 ${f(gz - 0.002)}" ` +
        `size="0.014 0.014 0.012" rgba="0.85 0.22 0.22 1" ${decor}/>`,
      `<geom name="printer_${n}_nozzle" type="cylinder" pos="0 0 ${f(gz - 0.014)}" ` +
        `size="0.003 0.004" rgba="0.95 0.75 0.15 1" ${decor}/>`,
    );
    // Status LCD / LED screen
    const screenColor = s.state === 'printing' ? '1 0.65 0.10 1' : '0.10 0.85 0.95 1';
    g.push(
      `<geom name="printer_${n}_screen" type="box" pos="${f(fw + 0.002)} 0 ${f(top + 0.030)}" ` +
        `size="0.002 0.018 0.012" rgba="${screenColor}" ${decor}/>`,
    );

    return g;
  }

  // 2. Deposit / Dispenser Station: Multi-item storage hopper
  if (stType === 'deposit') {
    const trayH = 0.012;
    // Sloped dispenser tray with metallic side rails
    g.push(
      `<geom name="deposit_${n}_tray" type="box" pos="0 0 ${f(top + trayH / 2)}" ` +
        `size="0.075 0.075 ${f(trayH / 2)}" rgba="0.30 0.35 0.42 1" ${solid}/>`,
      `<geom name="deposit_${n}_rail_l" type="box" pos="0 -0.072 ${f(top + trayH + 0.012)}" ` +
        `size="0.075 0.003 0.012" rgba="0.60 0.65 0.70 1" ${decor}/>`,
      `<geom name="deposit_${n}_rail_r" type="box" pos="0 0.072 ${f(top + trayH + 0.012)}" ` +
        `size="0.075 0.003 0.012" rgba="0.60 0.65 0.70 1" ${decor}/>`,
      `<geom name="deposit_${n}_stop" type="box" pos="0.072 0 ${f(top + trayH + 0.008)}" ` +
        `size="0.003 0.075 0.008" rgba="0.95 0.65 0.15 1" ${decor}/>`,
    );
    // Dispenser items (renders s.itemCount items, max 5)
    const count = Math.min(5, Math.max(0, s.itemCount ?? 3));
    for (let i = 0; i < count; i++) {
      const iy = (i - (count - 1) / 2) * 0.030;
      g.push(
        `<geom name="deposit_${n}_item_${i}" type="box" pos="0 ${f(iy)} ${f(top + trayH + 0.010)}" ` +
          `size="0.022 0.012 0.010" rgba="0.25 0.75 0.45 1" ${solid}/>`,
      );
    }
    if (count > 0 && s.propTag) {
      g.push(...padTagGeoms(`ptag_${s.propTag}`, top + trayH + 0.020, 0.010));
    }
    return g;
  }

  // 3. Magic Modification Station: Transmutation / transformation chamber
  if (stType === 'modifier') {
    const chR = 0.065, chH = 0.085;
    // Base scanner ring
    g.push(
      `<geom name="modifier_${n}_ring_b" type="cylinder" pos="0 0 ${f(top + 0.006)}" ` +
        `size="${f(chR + 0.006)} 0.006" rgba="0.22 0.25 0.30 1" ${solid}/>`,
    );
    // Glowing containment dome (translucent radiant purple)
    g.push(
      `<geom name="modifier_${n}_dome" type="cylinder" pos="0 0 ${f(top + chH / 2)}" ` +
        `size="${f(chR)} ${f(chH / 2)}" rgba="0.65 0.25 0.95 0.35" ${decor}/>`,
    );
    // Top magnetic emitter ring + glowing core
    g.push(
      `<geom name="modifier_${n}_ring_t" type="cylinder" pos="0 0 ${f(top + chH)}" ` +
        `size="${f(chR + 0.006)} 0.005" rgba="0.22 0.25 0.30 1" ${decor}/>`,
      `<geom name="modifier_${n}_core" type="sphere" pos="0 0 ${f(top + chH + 0.012)}" ` +
        `size="0.014" rgba="0.95 0.20 0.85 0.95" ${decor}/>`,
    );
    // Transformed output item inside chamber
    if ((s.outputStlName || s.stlName) && isMeshFileAvailable(s.outputStlName || s.stlName, availableMeshes)) {
      g.push(...customMeshGeoms(s, physics, top + 0.010, availableMeshes));
    } else {
      g.push(
        `<geom name="modifier_${n}_transmuted" type="box" pos="0 0 ${f(top + 0.015)}" ` +
          `size="0.024 0.024 0.015" rgba="0.85 0.35 0.95 1" ${solid}/>`,
      );
      if (s.propTag) {
        g.push(...padTagGeoms(`ptag_${s.propTag}`, top + 0.030, 0.012));
      }
    }
    return g;
  }

  // 4. Table Station: Spacious drop workbench / assembly surface
  if (stType === 'table') {
    // Wide hardwood/metallic worktable top (no pre-placed objects)
    g.push(
      `<geom name="table_${n}_top" type="box" pos="0 0 ${f(top + 0.006)}" ` +
        `size="0.115 0.115 0.006" rgba="0.75 0.58 0.40 1" ${solid}/>`,
    );
    return g;
  }

  // 5. Standard props
  switch (s.prop) {
    case 'bottle': {
      const r = 0.035, h = 0.11;
      g.push(
        `<geom name="bottle_${n}" type="cylinder" pos="0 0 ${f(top + h / 2)}" ` +
          `size="${f(r)} ${f(h / 2)}" rgba="0.30 0.55 0.85 0.85" ${solid}/>`,
        `<geom name="bottle_cap_${n}" type="cylinder" pos="0 0 ${f(top + h + 0.012)}" ` +
          `size="0.014 0.012" rgba="0.90 0.90 0.92 1" ${decor}/>`,
        `<geom name="bottle_neck_${n}" type="cylinder" pos="0 0 ${f(top + h + 0.001)}" ` +
          `size="0.016 0.010" rgba="0.30 0.55 0.85 0.85" ${decor}/>`,
      );
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
    case 'basket': {
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
    case 'none':
      break;
  }
  return g;
}

/** Every pedestal, its board top, its side nav tag, and whatever stands on it. */
function stationGeoms(physics: boolean, availableMeshes?: any): string[] {
  const g: string[] = [];
  const boardHalf = SQUARE_MM / 2 / 1000;
  const tagHalf = TAG_MM / 2 / 1000;
  for (const s of STATIONS) {
    const n = s.navTag;
    const yaw = stationYaw(s);
    if (s.isBoundary) {
      // Modular stop stand with warm brown base plate and sleek dark blue upright structure
      const uniqueSuffix = s.y < 0 ? 'start' : 'end';
      const tagZ = 0.070; // Aligned at top of upright face (56mm tag height, 95mm face + 6mm base, top at Z = 101mm)
      g.push(`<body name="boundary_post_${uniqueSuffix}" pos="${f(s.x)} ${f(s.y)} 0" euler="0 0 ${f(yaw)}">`);
      // 1. Base Plate in Warm Earthy Walnut Brown
      g.push(
        `<geom name="boundary_base_${uniqueSuffix}" type="mesh" mesh="boundary_base_plate" ` +
          `rgba="0.48 0.28 0.16 1" ` +
          (physics ? 'contype="3" conaffinity="3"' : 'contype="0" conaffinity="0" group="1"') +
          `/>`,
      );
      // 2. Upright Face Plate and Gusset Brackets in Sleek Dark Blue
      g.push(
        `<geom name="boundary_upright_${uniqueSuffix}" type="mesh" mesh="boundary_stand_upright" ` +
          `rgba="0.18 0.24 0.36 1" ` +
          (physics ? 'contype="3" conaffinity="3"' : 'contype="0" conaffinity="0" group="1"') +
          `/>`,
      );
      // 3. Rounded Stop Stand Tag Plate (46mm wide x 56mm high) with bold red borders & exploration text
      g.push(
        `<geom name="boundary_navtag_${uniqueSuffix}" type="box" ` +
          `pos="0.0186 0 ${f(tagZ)}" ` +
          `size="0.0005 0.023 0.028" ` +
          `rgba="1 1 1 1" contype="0" conaffinity="0" group="1"/>`,
      );
      g.push('</body>');
      continue;
    }
    // A body lets everything below be written in the station's own frame.
    g.push(`<body name="station_${n}" pos="${f(s.x)} ${f(s.y)} 0" euler="0 0 ${f(yaw)}">`);
    g.push(
      `<geom name="station_pedestal_${n}" type="box" pos="0 0 ${f(STATION_H / 2)}" ` +
        `size="${f(STATION_HALF)} ${f(STATION_HALF)} ${f(STATION_H / 2)}" ` +
        `rgba="0.35 0.36 0.40 1" ` +
        (physics ? 'contype="3" conaffinity="3"' : 'contype="0" conaffinity="0" group="1"') +
        `/>`,
    );
    // Board on the top face.
    g.push(
      `<geom name="station_board_${n}" type="box" pos="0 0 ${f(STATION_H + 0.0005)}" ` +
        `size="${f(boardHalf)} ${f(boardHalf)} 0.0005" rgba="0.85 0.80 0.65 1" ` +
        `contype="0" conaffinity="0" group="1"/>`,
    );
    // The calibration border only goes on the first station
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
    // The dual nav tag card on the +X side
    const tz = STATION_H - 0.045;
    // White sheet backing plate for the entire card (160mm wide x 60mm high)
    g.push(
      `<geom name="station_navtag_face_${n}" type="box" ` +
        `pos="${f(STATION_HALF + 0.0005)} 0 ${f(tz)}" ` +
        `size="0.0005 0.080 0.030" ` +
        `rgba="1 1 1 1" contype="0" conaffinity="0" group="1"/>`,
    );
    // Left marker (Tag 200) - on -Y side (viewer left)
    g.push(
      `<geom name="station_navtag_${n}" type="box" ` +
        `pos="${f(STATION_HALF + 0.0015)} -0.055 ${f(tz)}" ` +
        `size="0.0005 0.020 0.020" ` +
        `rgba="1 1 1 1" contype="0" conaffinity="0" group="1"/>`,
    );
    // Right marker (Tag 201) - on +Y side (viewer right)
    g.push(
      `<geom name="station_navtag_right_${n}" type="box" ` +
        `pos="${f(STATION_HALF + 0.0015)} 0.055 ${f(tz)}" ` +
        `size="0.0005 0.020 0.020" ` +
        `rgba="1 1 1 1" contype="0" conaffinity="0" group="1"/>`,
    );
    // Middle text plate carrying the station's label
    g.push(
      `<geom name="station_label_${n}" type="box" ` +
        `pos="${f(STATION_HALF + 0.0015)} 0 ${f(tz)}" ` +
        `size="0.0005 0.030 0.020" ` +
        `rgba="1 1 1 1" contype="0" conaffinity="0" group="1"/>`,
    );
    g.push(...propGeoms(s, physics, availableMeshes));
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
  opts: BuildSceneOptions = {},
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

  const station = stationGeoms(!!opts.physics, opts.availableMeshes).join('\n    ');

  // The real wrist-cam bracket (SO-ARM100 Optional/Wrist_Cam_Mount_32x32_UVC_Module,
  // with the fused gripper jaw trimmed off so it doesn't draw a second jaw over the
  // arm's own). Mocap, so it can ride the gripper frame without being welded in.
  const mount =
    `<body name="wrist_cam_mount" mocap="true" pos="0 0 -5">` +
    `<geom name="wrist_cam_mount_geom" type="mesh" mesh="wrist_cam_mount" ` +
    `rgba="0.55 0.57 0.62 1" contype="0" conaffinity="0" group="1"/></body>`;
  const injection =
    `\n    ${geoms.join('\n    ')}\n    ${station}\n    ${mount}\n    ` +
    `${floor}\n    ${block}\n    ${freePropBodies(!!opts.physics, opts.availableMeshes)}\n  </worldbody>`;
  const idx = baseXml.lastIndexOf('</worldbody>');
  if (idx < 0) throw new Error('No </worldbody> in base model XML');
  let out = baseXml.slice(0, idx) + injection + baseXml.slice(idx + '</worldbody>'.length);

  // STL is in millimetres, like the arm's own meshes.
  let customMeshes = '';
  for (const s of STATIONS) {
    const stl = s.outputStlName || s.stlName;
    if (stl && isMeshFileAvailable(stl, opts.availableMeshes)) {
      const cleanName = stl.replace(/^assets\//, '').replace(/^models\//, '');
      customMeshes += `    <mesh name="mesh_st_${s.navTag}" file="${cleanName}" scale="0.001 0.001 0.001"/>\n`;
    }
  }

  const meshDecl =
    '<mesh name="wrist_cam_mount" file="wrist_cam_mount.stl" scale="0.001 0.001 0.001"/>\n' +
    '    <mesh name="boundary_base_plate" file="boundary_base_plate.stl" scale="0.001 0.001 0.001"/>\n' +
    '    <mesh name="boundary_stand_upright" file="boundary_stand_upright.stl" scale="0.001 0.001 0.001"/>\n' +
    '    <mesh name="boundary_stop_stand" file="boundary_stop_stand.stl" scale="0.001 0.001 0.001"/>\n' +
    customMeshes +
    '  </asset>';
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
