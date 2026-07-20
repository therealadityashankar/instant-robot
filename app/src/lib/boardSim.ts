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

const f = (n: number) => n.toFixed(5);

/**
 * Inject the board (decorative geoms) and a mocap block body into a model's
 * `<worldbody>`. The block is a mocap body so its pose is set kinematically via
 * `data.mocap_pos` — no physics/free joint needed. Returns the modified XML.
 */
export function buildBoardSceneXml(baseXml: string): string {
  const tagHalf = TAG_MM / 2 / 1000;
  const boardHalf = SQUARE_MM / 2 / 1000;
  const intHalf = (SQUARE_MM / 2 - INSET_MM) / 1000;
  const [boardCx, boardCy] = boardToSim(SQUARE_MM / 2, SQUARE_MM / 2);

  const geoms: string[] = [];
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

  // Movable block as a mocap body (pose set via data.mocap_pos). Starts hidden
  // below the floor until a real detection places it.
  const block =
    `<body name="block" mocap="true" pos="${f(ROBOT_BASE_TIP_X + 0.15)} 0 -0.1">` +
    `<geom name="block_geom" type="box" ` +
    `size="${f(BLOCK_HALF_X)} ${f(BLOCK_HALF_Y)} ${f(BLOCK_HALF_Z)}" ` +
    `rgba="0.90 0.30 0.20 1" contype="0" conaffinity="0" group="1"/></body>`;

  const injection = `\n    ${geoms.join('\n    ')}\n    ${block}\n  </worldbody>`;
  const idx = baseXml.lastIndexOf('</worldbody>');
  if (idx < 0) throw new Error('No </worldbody> in base model XML');
  return baseXml.slice(0, idx) + injection + baseXml.slice(idx + '</worldbody>'.length);
}
