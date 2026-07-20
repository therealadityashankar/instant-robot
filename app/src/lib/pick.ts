// Pick state machine — ports the navigation phases of run_real_ik.py's
// StateMachine + perform_pick into a frame-driven controller. Given the block's
// sim position and the current end-effector position, it yields the next IK
// target and gripper command; the Simulator drives IK toward that target each
// frame and streams the joints to the real arm.

export type PickPhase = 'idle' | 'approach' | 'descend' | 'grasp' | 'lift' | 'done';

export interface PickParams {
  hoverZ: number; // m above board to hover before descending
  graspZ: number; // m above board to close the gripper
  graspXOffset: number; // m +x offset applied while descending/grasping/lifting
  graspYOffset: number; // m +y offset applied while descending/grasping/lifting
  liftZ: number; // m above board for the final lift
  ikZOffset: number; // global vertical offset added to every IK target
  posThreshold: number; // m — how close the EE must be to advance
  graspFrames: number; // frames to hold while the gripper closes
  gripperOpen: number; // gripper joint angle (rad) when open
  gripperClose: number; // gripper joint angle (rad) when closed
}

export const DEFAULT_PICK: PickParams = {
  hoverZ: 0.1,
  graspZ: 0.01, // descend + grasp height above the board
  graspXOffset: 0.029, // reach 0.029 m forward when descending/grasping
  graspYOffset: 0, // lateral offset when descending/grasping
  liftZ: 0.12,
  ikZOffset: 0.1,
  posThreshold: 0.0175,
  graspFrames: 20,
  gripperOpen: -0.1745,
  gripperClose: 1.6, // tighter close (ctrlrange max is ~1.745)
};

export interface PickStep {
  target: [number, number, number];
  gripper: number;
  phase: PickPhase;
  done: boolean;
}

function dist(a: [number, number, number], b: [number, number, number]): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

/**
 * The IK target + gripper command for a single phase, without advancing —
 * used to drive to and *hold* at a chosen stage (manual stepping).
 */
export function phaseTarget(
  block: [number, number, number],
  phase: PickPhase,
  params: PickParams = DEFAULT_PICK,
): { target: [number, number, number]; gripper: number } {
  const [bx, by] = block;
  const p = params;
  const gx = bx + p.graspXOffset; // grasp x/y for descend/grasp/lift
  const gy = by + p.graspYOffset;
  switch (phase) {
    case 'approach':
      return { target: [bx, by, p.hoverZ + p.ikZOffset], gripper: p.gripperOpen };
    case 'descend':
      return { target: [gx, gy, p.graspZ + p.ikZOffset], gripper: p.gripperOpen };
    case 'grasp':
      return { target: [gx, gy, p.graspZ + p.ikZOffset], gripper: p.gripperClose };
    case 'lift':
      return { target: [gx, gy, p.liftZ + p.ikZOffset], gripper: p.gripperClose };
    default:
      return { target: block, gripper: p.gripperOpen };
  }
}

export class PickController {
  phase: PickPhase = 'idle';
  private block: [number, number, number] | null = null;
  private graspCounter = 0;
  private p: PickParams;

  constructor(params: Partial<PickParams> = {}) {
    this.p = { ...DEFAULT_PICK, ...params };
  }

  /** Update tuning params live (e.g. grasp depth). */
  setParams(partial: Partial<PickParams>) {
    this.p = { ...this.p, ...partial };
  }

  /** Begin a pick of the block at `blockSimPos` (world x,y,z). */
  start(blockSimPos: [number, number, number]) {
    this.block = blockSimPos;
    this.phase = 'approach';
    this.graspCounter = 0;
  }

  cancel() {
    this.phase = 'idle';
    this.block = null;
  }

  get active(): boolean {
    return this.phase !== 'idle' && this.phase !== 'done';
  }

  /**
   * Advance one frame. `eePos` is the current grasp-site world position.
   * Returns the IK target to drive toward and the gripper command this frame.
   */
  step(eePos: [number, number, number]): PickStep {
    const { p } = this;
    if (!this.block || this.phase === 'idle' || this.phase === 'done') {
      return { target: eePos, gripper: p.gripperOpen, phase: this.phase, done: this.phase === 'done' };
    }
    const [bx, by] = this.block;
    const gx = bx + p.graspXOffset; // grasp x/y for descend/grasp/lift
    const gy = by + p.graspYOffset;

    if (this.phase === 'approach') {
      const target: [number, number, number] = [bx, by, p.hoverZ + p.ikZOffset];
      if (dist(eePos, target) < p.posThreshold) this.phase = 'descend';
      return { target, gripper: p.gripperOpen, phase: 'approach', done: false };
    }
    if (this.phase === 'descend') {
      const target: [number, number, number] = [gx, gy, p.graspZ + p.ikZOffset];
      if (dist(eePos, target) < p.posThreshold) {
        this.phase = 'grasp';
        this.graspCounter = 0;
      }
      return { target, gripper: p.gripperOpen, phase: 'descend', done: false };
    }
    if (this.phase === 'grasp') {
      const target: [number, number, number] = [gx, gy, p.graspZ + p.ikZOffset];
      this.graspCounter += 1;
      if (this.graspCounter >= p.graspFrames) this.phase = 'lift';
      return { target, gripper: p.gripperClose, phase: 'grasp', done: false };
    }
    // lift
    const target: [number, number, number] = [gx, gy, p.liftZ + p.ikZOffset];
    if (dist(eePos, target) < p.posThreshold) this.phase = 'done';
    return { target, gripper: p.gripperClose, phase: 'lift', done: this.phase === 'done' };
  }
}
