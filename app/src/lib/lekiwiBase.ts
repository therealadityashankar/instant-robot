// Empirical drive model for the LeKiwi 3-omniwheel base.
//
// The base has THREE natural drive directions, 120° apart: forward, back-left
// (+120°) and back-right (−120°). Each is produced by spinning one PAIR of wheels
// (the third idle). Rather than deriving anything from mount angles / radii, we
// store the raw wheel-speed pattern for each of those three directions plus
// rotation — measured directly by the "drive a pair, say which way it went"
// calibration. A drive command is just a blend of the four patterns.

export interface BaseConfig {
  /** Feetech servo ID for each wheel (indices 0,1,2 used by the patterns). */
  wheelIds: [number, number, number];
  /** Wheel command magnitude (raw servo units) at full deflection. */
  speed: number;
  /** Wheel-speed pattern for each primitive direction, + rotation. */
  forward: [number, number, number];
  backLeft: [number, number, number]; // +120°
  backRight: [number, number, number]; // −120°
  rotate: [number, number, number]; // CCW
}

// Sensible starting patterns; calibration overwrites them with what the real base does.
export const DEFAULT_BASE_CONFIG: BaseConfig = {
  wheelIds: [7, 8, 9],
  speed: 400,
  forward: [0, -1, 1],
  backLeft: [1, 0, -1],
  backRight: [1, -1, 0],
  rotate: [1, 1, 1],
};

// The three wheel PAIRS the calibration drives (one wheel idle each). The two
// active wheels spin opposite so the base translates.
export const CAL_PAIRS: [number, number, number][] = [
  [0, 1, -1], // wheels 1 & 2
  [1, 0, -1], // wheels 0 & 2
  [1, -1, 0], // wheels 0 & 1
];

// Unit ground directions of the three primitives (body frame: x fwd, y left).
export const PRIMITIVE_DIRS = {
  forward: [1, 0],
  backLeft: [-0.5, 0.8660254],
  backRight: [-0.5, -0.8660254],
} as const;

const HARD_LIMIT = 1023; // Feetech wheel-speed magnitude cap
const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));

/**
 * Blend the four primitive amounts (each roughly −1..1) into signed raw wheel
 * speeds keyed by servo ID. Driving a single primitive fires exactly its 2-wheel
 * pattern; combinations blend.
 */
export function wheelSpeeds(
  fwd: number,
  bl: number,
  br: number,
  turn: number,
  cfg: BaseConfig,
): Map<number, number> {
  const out = new Map<number, number>();
  for (let i = 0; i < 3; i++) {
    const w = fwd * cfg.forward[i] + bl * cfg.backLeft[i] + br * cfg.backRight[i] + turn * cfg.rotate[i];
    out.set(cfg.wheelIds[i], clamp(Math.round(w * cfg.speed), -HARD_LIMIT, HARD_LIMIT));
  }
  return out;
}

/**
 * Decompose a body velocity (vx forward, vy left) into the three primitive
 * amounts (min-norm; the primitives are 120° apart and redundant for 2-D). Used
 * by navigation, which wants an arbitrary direction rather than a single primitive.
 */
export function bodyToPrimitives(vx: number, vy: number): { fwd: number; bl: number; br: number } {
  const d = PRIMITIVE_DIRS;
  const k = 2 / 3;
  return {
    fwd: k * (vx * d.forward[0] + vy * d.forward[1]),
    bl: k * (vx * d.backLeft[0] + vy * d.backLeft[1]),
    br: k * (vx * d.backRight[0] + vy * d.backRight[1]),
  };
}
