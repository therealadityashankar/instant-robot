// Joint calibration data + fitting — port of calibrate_joints_real.py.
//
// The Python tool pre-renders MuJoCo reference images of each joint pose; that
// sim-rendering step is omitted here (it would require shipping the SO-101 model
// and the MuJoCo WASM runtime). Everything else — the per-joint reference
// targets, capturing the live servo reading, the per-joint linear fit
// (sim = scale·real + offset) and the sim→real conversion used for verification
// — is reproduced faithfully.

export interface JointPoint {
  label: string;
  simRad: number;
}

export interface JointDef {
  joint: string;
  index: number;
  /** Feetech servo ID on the bus (SO-101: joints are servos 1..6). */
  servoId: number;
  description: string;
  points: JointPoint[];
}

const HALF_PI = 1.5708;

export const CALIBRATION_PLAN: JointDef[] = [
  {
    joint: 'shoulder_pan',
    index: 0,
    servoId: 1,
    description: 'Base rotation — sweeps arm left / right',
    points: [
      { label: 'Centered (straight forward)', simRad: 0.0 },
      { label: 'Swept LEFT  ~90°', simRad: HALF_PI },
      { label: 'Swept RIGHT ~90°', simRad: -HALF_PI },
    ],
  },
  {
    joint: 'shoulder_lift',
    index: 1,
    servoId: 2,
    description: 'Shoulder tilt — raises / lowers the upper arm',
    points: [
      { label: 'Arm straight UP (vertical)', simRad: 0.0 },
      { label: 'Arm tilted FORWARD ~90°', simRad: HALF_PI },
    ],
  },
  {
    joint: 'elbow_flex',
    index: 2,
    servoId: 3,
    description: 'Elbow bend — folds the forearm',
    points: [
      { label: 'Elbow fully STRAIGHT', simRad: 0.0 },
      { label: 'Elbow bent ~90°', simRad: HALF_PI },
    ],
  },
  {
    joint: 'wrist_flex',
    index: 3,
    servoId: 4,
    description: 'Wrist pitch — tilts gripper up / down',
    points: [
      { label: 'Wrist NEUTRAL (level)', simRad: 0.0 },
      { label: 'Wrist flexed DOWN ~90°', simRad: HALF_PI },
    ],
  },
  {
    joint: 'wrist_roll',
    index: 4,
    servoId: 5,
    description: 'Wrist roll — rotates the gripper',
    points: [
      { label: 'No roll (level)', simRad: 0.0 },
      { label: 'Rolled 90° clockwise', simRad: HALF_PI },
      { label: 'Rolled 90° counter-clockwise', simRad: -HALF_PI },
    ],
  },
  {
    joint: 'gripper',
    index: 5,
    servoId: 6,
    description: 'Gripper open / close',
    points: [
      { label: 'Gripper FULLY OPEN', simRad: -0.17453 },
      { label: 'Gripper FULLY CLOSED', simRad: 1.74533 },
    ],
  },
];

export interface VerificationPose {
  label: string;
  /** [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper] */
  ctrlRad: number[];
}

export const VERIFICATION_POSES: VerificationPose[] = [
  { label: 'Home — arm straight up, gripper half-open', ctrlRad: [0.0, 0.0, 0.0, 0.0, 0.0, 0.5] },
  { label: 'Swept LEFT — arm still raised', ctrlRad: [0.8, 0.2, 0.2, 0.0, 0.0, 0.5] },
  { label: 'Swept RIGHT — arm still raised', ctrlRad: [-0.8, 0.2, 0.2, 0.0, 0.0, 0.5] },
];

export const JOINT_NAMES = CALIBRATION_PLAN.map((j) => j.joint);

/** Raw Feetech position range (12-bit encoder). Used to clip verification writes. */
export const SERVO_MIN = 0;
export const SERVO_MAX = 4095;

export interface JointFit {
  scale: number | null;
  offset: number | null;
  r2: number | null;
}

/** Least-squares fit of `sim = scale·real + offset`. Port of fit_linear(). */
export function fitLinear(realVals: number[], simVals: number[]): JointFit {
  const n = realVals.length;
  if (n < 2) return { scale: null, offset: null, r2: null };

  const meanX = realVals.reduce((a, b) => a + b, 0) / n;
  const meanY = simVals.reduce((a, b) => a + b, 0) / n;
  let cov = 0;
  let varX = 0;
  for (let i = 0; i < n; i++) {
    cov += (realVals[i] - meanX) * (simVals[i] - meanY);
    varX += (realVals[i] - meanX) ** 2;
  }
  const scale = varX === 0 ? 0 : cov / varX;
  const offset = meanY - scale * meanX;

  let ssRes = 0;
  let ssTot = 0;
  for (let i = 0; i < n; i++) {
    const pred = scale * realVals[i] + offset;
    ssRes += (simVals[i] - pred) ** 2;
    ssTot += (simVals[i] - meanY) ** 2;
  }
  const r2 = ssTot > 1e-12 ? 1 - ssRes / ssTot : 1;
  return { scale, offset, r2 };
}

export interface JointCalibration {
  index: number;
  servoId: number;
  points: Array<{ real: number; simRad: number }>;
  scale: number | null;
  offset: number | null;
  r2: number | null;
}

/**
 * Convert a target sim angle (rad) to a raw servo position using a joint's fit.
 * Mirrors the verification conversion in the Python tool: real = (sim - b) / m.
 */
export function simRadToServo(simRad: number, fit: JointFit): number | null {
  if (fit.scale == null || fit.offset == null || Math.abs(fit.scale) < 1e-9) return null;
  const real = (simRad - fit.offset) / fit.scale;
  return Math.min(SERVO_MAX, Math.max(SERVO_MIN, Math.round(real)));
}
