// Persist calibrations in localStorage so they're reused by default across
// sessions. A supplied calibration file always overrides (and replaces) the
// stored one. All access is guarded — storage may be unavailable/full.

import type { JointCalibration } from './joints';

const BOARD_KEY = 'instant-robot:board-calibration';
const JOINT_KEY = 'instant-robot:joint-calibration';

export interface BoardCorrection {
  Sx: number;
  Bx: number;
  Sy: number;
  By: number;
}

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

export function saveBoardCalibration(c: BoardCorrection) {
  write(BOARD_KEY, { Sx: c.Sx, Bx: c.Bx, Sy: c.Sy, By: c.By });
}

export function loadBoardCalibration(): BoardCorrection | null {
  const d = read<BoardCorrection>(BOARD_KEY);
  if (d && ['Sx', 'Bx', 'Sy', 'By'].every((k) => typeof (d as any)[k] === 'number')) return d;
  return null;
}

export function saveJointCalibration(c: Record<string, JointCalibration>) {
  write(JOINT_KEY, c);
}

export function loadJointCalibration(): Record<string, JointCalibration> | null {
  const d = read<Record<string, JointCalibration>>(JOINT_KEY);
  return d && typeof d === 'object' ? d : null;
}
