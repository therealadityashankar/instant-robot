// Generic Goal Quadrilateral Pick Calibration configuration and geometry helpers.
//
// Defines a target 4-corner quadrilateral with freely movable vertices, sides,
// and configurable tolerance error regions in the camera image plane.
// When an ArUco marker falls within this error region, the physical robot gripper
// is confirmed to be positioned directly above the tag for picking.

export type CornerPoint = [number, number];

export interface GoalQuadConfig {
  corners: [CornerPoint, CornerPoint, CornerPoint, CornerPoint]; // P0 (TL), P1 (TR), P2 (BR), P3 (BL)
  targetTagId: number; // Target ArUco marker ID (e.g. 201)
  tolerancePx?: number; // Allowed error border region / margin in pixels (default 14px)
  downStepMm?: number; // Stepwise descent distance in mm (default 6mm)
  savedAt?: string;
}

export const DEFAULT_GOAL_QUAD_SIM: GoalQuadConfig = {
  corners: [
    [280, 180], // P0 Top-Left
    [360, 180], // P1 Top-Right
    [360, 260], // P2 Bottom-Right
    [280, 260], // P3 Bottom-Left
  ],
  targetTagId: 201,
  tolerancePx: 14,
  downStepMm: 6,
};

export const DEFAULT_GOAL_QUAD_REAL: GoalQuadConfig = {
  corners: [
    [416, 198], // P0 Top-Left
    [504, 198], // P1 Top-Right
    [506, 285], // P2 Bottom-Right
    [414, 285], // P3 Bottom-Left
  ],
  targetTagId: 201,
  tolerancePx: 14,
  downStepMm: 6,
};

export const DEFAULT_GOAL_QUAD = DEFAULT_GOAL_QUAD_SIM;

const GOAL_QUAD_KEY_SIM = 'instant-robot:goal-quad-config:sim';
const GOAL_QUAD_KEY_REAL = 'instant-robot:goal-quad-config:real';
const GOAL_QUAD_KEY_LEGACY = 'instant-robot:goal-quad-config';

export function saveGoalQuad(cfg: GoalQuadConfig, mode: 'sim' | 'real' = 'sim') {
  try {
    const key = mode === 'real' ? GOAL_QUAD_KEY_REAL : GOAL_QUAD_KEY_SIM;
    localStorage.setItem(key, JSON.stringify(cfg));
  } catch (err) {
    console.warn('Failed to save goal quad config', err);
  }
}

export function loadGoalQuad(mode: 'sim' | 'real' = 'sim'): GoalQuadConfig {
  try {
    const key = mode === 'real' ? GOAL_QUAD_KEY_REAL : GOAL_QUAD_KEY_SIM;
    const defaultCfg = mode === 'real' ? DEFAULT_GOAL_QUAD_REAL : DEFAULT_GOAL_QUAD_SIM;
    const raw = localStorage.getItem(key) || (mode === 'real' ? null : localStorage.getItem(GOAL_QUAD_KEY_LEGACY));
    if (raw) {
      const parsed = JSON.parse(raw);
      if (Array.isArray(parsed.corners) && parsed.corners.length === 4) {
        return { ...defaultCfg, ...parsed };
      }
    }
    return { ...defaultCfg };
  } catch (err) {
    console.warn('Failed to load goal quad config', err);
  }
  return mode === 'real' ? { ...DEFAULT_GOAL_QUAD_REAL } : { ...DEFAULT_GOAL_QUAD_SIM };
}

/** Computes centroid [cx, cy] of a quadrilateral */
export function computeQuadCenter(c: [CornerPoint, CornerPoint, CornerPoint, CornerPoint]): [number, number] {
  return [
    (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4,
    (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4,
  ];
}

/** Computes the 4 edge lengths [Top, Right, Bottom, Left] of a quadrilateral in pixels */
export function computeQuadSideLengths(c: [CornerPoint, CornerPoint, CornerPoint, CornerPoint]): [number, number, number, number] {
  const s0 = Math.hypot(c[1][0] - c[0][0], c[1][1] - c[0][1]); // Top
  const s1 = Math.hypot(c[2][0] - c[1][0], c[2][1] - c[1][1]); // Right
  const s2 = Math.hypot(c[3][0] - c[2][0], c[3][1] - c[2][1]); // Bottom
  const s3 = Math.hypot(c[0][0] - c[3][0], c[0][1] - c[3][1]); // Left
  return [s0, s1, s2, s3];
}

/** Computes the mean side length (scale) of a quadrilateral in pixels */
export function computeQuadAvgSize(c: [CornerPoint, CornerPoint, CornerPoint, CornerPoint]): number {
  const [s0, s1, s2, s3] = computeQuadSideLengths(c);
  return (s0 + s1 + s2 + s3) / 4;
}

/** Computes the in-plane orientation angle (in degrees) of the top edge P0 -> P1 */
export function computeQuadOrientationDeg(c: [CornerPoint, CornerPoint, CornerPoint, CornerPoint]): number {
  return (Math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0]) * 180) / Math.PI;
}

/** Translates all 4 corners by (dx, dy) */
export function translateQuad(
  c: [CornerPoint, CornerPoint, CornerPoint, CornerPoint],
  dx: number,
  dy: number
): [CornerPoint, CornerPoint, CornerPoint, CornerPoint] {
  return [
    [c[0][0] + dx, c[0][1] + dy],
    [c[1][0] + dx, c[1][1] + dy],
    [c[2][0] + dx, c[2][1] + dy],
    [c[3][0] + dx, c[3][1] + dy],
  ];
}

/** Scales all 4 corners relative to the centroid by `factor` */
export function scaleQuad(
  c: [CornerPoint, CornerPoint, CornerPoint, CornerPoint],
  factor: number
): [CornerPoint, CornerPoint, CornerPoint, CornerPoint] {
  const [cx, cy] = computeQuadCenter(c);
  return c.map(([x, y]) => [
    cx + (x - cx) * factor,
    cy + (y - cy) * factor,
  ]) as [CornerPoint, CornerPoint, CornerPoint, CornerPoint];
}

/** Rotates all 4 corners around the centroid by `angleDeg` */
export function rotateQuad(
  c: [CornerPoint, CornerPoint, CornerPoint, CornerPoint],
  angleDeg: number
): [CornerPoint, CornerPoint, CornerPoint, CornerPoint] {
  const [cx, cy] = computeQuadCenter(c);
  const rad = (angleDeg * Math.PI) / 180;
  const cos = Math.cos(rad);
  const sin = Math.sin(rad);

  return c.map(([x, y]) => {
    const dx = x - cx;
    const dy = y - cy;
    return [
      cx + dx * cos - dy * sin,
      cy + dx * sin + dy * cos,
    ];
  }) as [CornerPoint, CornerPoint, CornerPoint, CornerPoint];
}

/**
 * Checks whether detected tag corners fall within the tolerance error region of the goal quad.
 * Returns { isInside: boolean, maxDist: number, avgDist: number }
 */
export function evaluateTolerance(
  goalCorners: [CornerPoint, CornerPoint, CornerPoint, CornerPoint],
  detectedCorners: [CornerPoint, CornerPoint, CornerPoint, CornerPoint],
  tolerancePx: number
): { isInside: boolean; maxDist: number; avgDist: number } {
  let sumDist = 0;
  let maxDist = 0;
  for (let i = 0; i < 4; i++) {
    const dist = Math.hypot(detectedCorners[i][0] - goalCorners[i][0], detectedCorners[i][1] - goalCorners[i][1]);
    sumDist += dist;
    if (dist > maxDist) maxDist = dist;
  }
  const avgDist = sumDist / 4;
  const isInside = maxDist <= tolerancePx;
  return { isInside, maxDist, avgDist };
}

// Backwards compatibility interfaces and aliases
export type GoalSquareConfig = GoalQuadConfig;
export const DEFAULT_GOAL_SQUARE = DEFAULT_GOAL_QUAD;
export const loadGoalSquare = loadGoalQuad;
export const saveGoalSquare = saveGoalQuad;

export interface PickCalibration {
  xOffsetMm: number;
  yOffsetMm: number;
  zOffsetMm: number;
  rollOffsetDeg: number;
  pitchOffsetDeg: number;
  yawOffsetDeg: number;
  dotSpacingMm?: number;
  targetTagId?: number;
  calibratedAt?: string;
}

export const DEFAULT_PICK_CALIBRATION: PickCalibration = {
  xOffsetMm: 0,
  yOffsetMm: 0,
  zOffsetMm: 0,
  rollOffsetDeg: 0,
  pitchOffsetDeg: 0,
  yawOffsetDeg: 0,
  dotSpacingMm: 25,
};

export function savePickCalibration(c: PickCalibration) {
  try {
    localStorage.setItem('instant-robot:pick-calibration', JSON.stringify(c));
  } catch {}
}

export function loadPickCalibration(): PickCalibration {
  try {
    const raw = localStorage.getItem('instant-robot:pick-calibration');
    if (raw) return { ...DEFAULT_PICK_CALIBRATION, ...JSON.parse(raw) };
  } catch {}
  return { ...DEFAULT_PICK_CALIBRATION };
}
