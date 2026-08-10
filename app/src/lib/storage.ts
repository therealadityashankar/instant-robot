// Persist calibrations in localStorage so they're reused by default across
// sessions. A supplied calibration file always overrides (and replaces) the
// stored one. All access is guarded — storage may be unavailable/full.

import type { JointCalibration } from './joints';
import type { Intrinsics } from './charuco';
import type { RobotId } from './robots';
import type { BaseConfig } from './lekiwiBase';

const JOINT_KEY = 'instant-robot:joint-calibration';
const INTRINSICS_KEY = 'instant-robot:camera-intrinsics';
const ROBOT_KEY = 'instant-robot:selected-robot';
const ARM_OFFSET_KEY = 'instant-robot:arm-offset'; // { [robotId]: [x,y,z] }
const BASE_CFG_KEY = 'instant-robot:base-config'; // LeKiwi wheel-drive config
const ARM_POSE_KEY = 'instant-robot:last-arm-pose'; // joints + EE target, for reload

function read<T>(key: string): T | null {
  try {
    const s = localStorage.getItem(key);
    return s ? (JSON.parse(s) as T) : null;
  } catch {
    return null;
  }
}

function write(key: string, value: unknown) {
  try {
    localStorage.setItem(key, JSON.stringify(value));
  } catch {
    /* storage unavailable or full — non-fatal */
  }
}

export function saveJointCalibration(c: Record<string, JointCalibration>) {
  write(JOINT_KEY, c);
}

export function loadJointCalibration(): Record<string, JointCalibration> | null {
  const d = read<Record<string, JointCalibration>>(JOINT_KEY);
  return d && typeof d === 'object' ? d : null;
}

export function saveIntrinsics(c: Intrinsics) {
  write(INTRINSICS_KEY, c);
}

export function loadIntrinsics(): Intrinsics | null {
  const d = read<Intrinsics>(INTRINSICS_KEY);
  if (d && Array.isArray(d.cameraMatrix) && d.cameraMatrix.length === 9) return d;
  return null;
}

export function saveRobotId(id: RobotId) {
  write(ROBOT_KEY, id);
}

export function loadRobotId(): RobotId | null {
  const d = read<RobotId>(ROBOT_KEY);
  return d === 'so101' || d === 'lekiwi' ? d : null;
}

export function saveArmOffset(id: RobotId, offset: [number, number, number]) {
  const all = read<Record<string, [number, number, number]>>(ARM_OFFSET_KEY) ?? {};
  all[id] = offset;
  write(ARM_OFFSET_KEY, all);
}

export function loadArmOffset(id: RobotId): [number, number, number] | null {
  const all = read<Record<string, [number, number, number]>>(ARM_OFFSET_KEY);
  const v = all?.[id];
  return Array.isArray(v) && v.length === 3 ? v : null;
}

export function saveBaseConfig(c: BaseConfig) {
  write(BASE_CFG_KEY, c);
}

export function loadBaseConfig(): Partial<BaseConfig> | null {
  const d = read<Partial<BaseConfig>>(BASE_CFG_KEY);
  return d && typeof d === 'object' ? d : null;
}

/** The arm's last pose: joint angles (rad) and the end-effector target it held. */
export interface ArmPose {
  joints: number[];
  target: [number, number, number];
  gripper: number;
}

export function saveArmPose(p: ArmPose) {
  write(ARM_POSE_KEY, p);
}

export function loadArmPose(): ArmPose | null {
  const d = read<ArmPose>(ARM_POSE_KEY);
  if (d && Array.isArray(d.joints) && d.joints.length >= 5 && Array.isArray(d.target)) return d;
  return null;
}
