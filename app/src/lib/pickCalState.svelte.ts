// State and runner bridge for Camera-View Goal Quadrilateral Pick Calibration.

import type { GoalQuadConfig } from './pickCalibration';

export type AutoNavMode = 'xy' | 'z' | 'all';

export class PickCalState {
  isNavigating = $state(false);
  isMovingToScan = $state(false);
  isNudging = $state(false);
  isPicking = $state(false);
  abortRequested = $state(false);
  activeMode = $state<AutoNavMode | null>(null);
  navStatus = $state<string | null>(null);
  navSuccess = $state<boolean | null>(null);

  // Live camera stream canvas source
  cameraSource = $state<CanvasImageSource | null>(null);

  // Live 2D detected marker corners: Map<tagId, [[x,y], [x,y], [x,y], [x,y]]>
  detectedCorners = $state<Map<number, [number, number][]>>(new Map());

  // Runners
  private autoNavRunner: ((
    goal: GoalQuadConfig,
    tagId: number,
    mode: AutoNavMode,
    stepDelayMs: number,
    stepGain: number,
    downStepMm: number,
    tolerancePx: number
  ) => Promise<{ success: boolean; msg: string }>) | null = null;
  private scanPosRunner: (() => Promise<{ success: boolean; msg: string }>) | null = null;
  private nudgeHeightRunner: ((deltaMm: number) => Promise<{ success: boolean; msg: string }>) | null = null;
  private pickRunner: ((downMm: number) => Promise<{ success: boolean; msg: string }>) | null = null;

  registerAutoNavRunner(
    fn: (
      goal: GoalQuadConfig,
      tagId: number,
      mode: AutoNavMode,
      stepDelayMs: number,
      stepGain: number,
      downStepMm: number,
      tolerancePx: number
    ) => Promise<{ success: boolean; msg: string }>
  ) {
    this.autoNavRunner = fn;
  }

  unregisterAutoNavRunner() {
    this.autoNavRunner = null;
  }

  registerScanPosRunner(fn: () => Promise<{ success: boolean; msg: string }>) {
    this.scanPosRunner = fn;
  }

  unregisterScanPosRunner() {
    this.scanPosRunner = null;
  }

  registerNudgeHeightRunner(fn: (deltaMm: number) => Promise<{ success: boolean; msg: string }>) {
    this.nudgeHeightRunner = fn;
  }

  unregisterNudgeHeightRunner() {
    this.nudgeHeightRunner = null;
  }

  registerPickRunner(fn: (downMm: number) => Promise<{ success: boolean; msg: string }>) {
    this.pickRunner = fn;
  }

  unregisterPickRunner() {
    this.pickRunner = null;
  }

  updateLiveCamera(source: CanvasImageSource | null, corners: Map<number, [number, number][]>) {
    this.cameraSource = source;
    this.detectedCorners = corners;
  }

  stopNavigation() {
    this.abortRequested = true;
    this.isNavigating = false;
    this.activeMode = null;
    this.navStatus = '⏹️ Auto-navigation stopped.';
  }

  async moveToScanPosition(): Promise<{ success: boolean; msg: string }> {
    if (!this.scanPosRunner) {
      this.navSuccess = false;
      this.navStatus = 'Scan position runner not ready. Ensure simulator/robot is active.';
      return { success: false, msg: this.navStatus };
    }

    this.isMovingToScan = true;
    this.navSuccess = null;
    this.navStatus = 'Moving arm to scan position above table…';

    try {
      const res = await this.scanPosRunner();
      this.navSuccess = res.success;
      this.navStatus = res.msg;
      return res;
    } catch (err: any) {
      const msg = err?.message || String(err);
      this.navSuccess = false;
      this.navStatus = `Move to scan position failed: ${msg}`;
      return { success: false, msg };
    } finally {
      this.isMovingToScan = false;
    }
  }

  async nudgeHeight(deltaMm: number): Promise<{ success: boolean; msg: string }> {
    if (!this.nudgeHeightRunner) {
      this.navSuccess = false;
      this.navStatus = 'Height nudge runner not ready. Ensure simulator/robot is active.';
      return { success: false, msg: this.navStatus };
    }

    this.isNudging = true;
    const dir = deltaMm < 0 ? 'downward' : 'upward';
    this.navSuccess = null;
    this.navStatus = `Moving arm ${Math.abs(deltaMm)}mm ${dir}…`;

    try {
      const res = await this.nudgeHeightRunner(deltaMm);
      this.navSuccess = res.success;
      this.navStatus = res.msg;
      return res;
    } catch (err: any) {
      const msg = err?.message || String(err);
      this.navSuccess = false;
      this.navStatus = `Nudge height failed: ${msg}`;
      return { success: false, msg };
    } finally {
      this.isNudging = false;
    }
  }

  async pickAndLift(downMm: number = 20): Promise<{ success: boolean; msg: string }> {
    if (!this.pickRunner) {
      this.navSuccess = false;
      this.navStatus = 'Pick runner not ready. Ensure simulator/robot is active.';
      return { success: false, msg: this.navStatus };
    }

    this.isPicking = true;
    this.navSuccess = null;
    this.navStatus = `Moving ${downMm}mm down, closing gripper, and lifting…`;

    try {
      const res = await this.pickRunner(downMm);
      this.navSuccess = res.success;
      this.navStatus = res.msg;
      return res;
    } catch (err: any) {
      const msg = err?.message || String(err);
      this.navSuccess = false;
      this.navStatus = `Pick and lift failed: ${msg}`;
      return { success: false, msg };
    } finally {
      this.isPicking = false;
    }
  }

  async autoNavigate(
    goal: GoalQuadConfig,
    tagId = 201,
    mode: AutoNavMode = 'all',
    stepDelayMs = 400,
    stepGain = 0.5,
    downStepMm = 6,
    tolerancePx = 14
  ): Promise<{ success: boolean; msg: string }> {
    if (!this.autoNavRunner) {
      this.navSuccess = false;
      this.navStatus = 'Auto-navigate runner not available. Ensure robot/simulator is running.';
      return { success: false, msg: this.navStatus };
    }

    this.abortRequested = false;
    this.isNavigating = true;
    this.activeMode = mode;
    this.navSuccess = null;
    const modeLabel = mode === 'xy' ? 'X-Y lateral' : mode === 'z' ? 'Z height' : 'Stepwise (X-Y → Descend)';
    this.navStatus = `Auto-navigating arm (${modeLabel}) to Tag #${tagId}…`;

    try {
      const res = await this.autoNavRunner(goal, tagId, mode, stepDelayMs, stepGain, downStepMm, tolerancePx);
      this.navSuccess = res.success;
      this.navStatus = res.msg;
      return res;
    } catch (err: any) {
      const msg = err?.message || String(err);
      this.navSuccess = false;
      this.navStatus = `Auto-navigation failed: ${msg}`;
      return { success: false, msg };
    } finally {
      this.isNavigating = false;
      this.activeMode = null;
    }
  }

  /**
   * Combined Stepwise Auto-Approach followed immediately by descent, grasp, and lift.
   */
  async autoApproachAndPick(
    goal: GoalQuadConfig,
    tagId = 201,
    stepDelayMs = 400,
    stepGain = 0.5,
    downStepMm = 6,
    tolerancePx = 14,
    pickDownDistanceMm = 20
  ): Promise<{ success: boolean; msg: string }> {
    this.abortRequested = false;
    this.navSuccess = null;
    this.navStatus = `Stepwise auto-approaching Tag #${tagId} before picking…`;

    // 1. Run stepwise visual alignment to goal quad
    const navRes = await this.autoNavigate(
      goal,
      tagId,
      'all',
      stepDelayMs,
      stepGain,
      downStepMm,
      tolerancePx
    );

    if (!navRes.success || this.abortRequested) {
      return navRes;
    }

    // 2. Descend a bit below, close gripper, and lift
    return await this.pickAndLift(pickDownDistanceMm);
  }
}

export const pickCalState = new PickCalState();
