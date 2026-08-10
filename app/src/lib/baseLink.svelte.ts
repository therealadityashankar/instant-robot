// Shared state for the arm's serial bus and the LeKiwi base wheels.
//
// The Simulator drives the base frame-to-frame, but the *calibration* UI lives in
// the Calibrate modal (a sibling of the Simulator, not a child), so the wheel
// config and the bus itself have to be reachable from both. Everything the two
// share lives here; anything only one of them needs stays where it is.

import { Robot } from './robot';
import { CAL_PAIRS, DEFAULT_BASE_CONFIG, type BaseConfig } from './lekiwiBase';
import { loadBaseConfig, saveBaseConfig } from './storage';

/** The one serial connection — arm servos (1-6) and wheels (7-9) share it. */
export const robot = new Robot();

/** How long each calibration pulse drives the wheels for. */
const TEST_MS = 1600;

export type PairDir =
  | 'forward'
  | 'backward'
  | 'backLeft'
  | 'frontRight'
  | 'backRight'
  | 'frontLeft';

class BaseLink {
  config = $state<BaseConfig>({ ...DEFAULT_BASE_CONFIG, ...(loadBaseConfig() ?? {}) });
  connected = $state(false);
  error = $state<string | null>(null);
  /** True while a calibration pulse is running — blocks overlapping tests. */
  testing = $state(false);

  persist() {
    saveBaseConfig({ ...this.config });
  }

  stopWheels() {
    const speeds = new Map(this.config.wheelIds.map((id) => [id, 0] as [number, number]));
    return robot.syncWriteWheelSpeed(speeds);
  }

  /** Restore the default drive patterns, keeping wheel IDs and speed. */
  resetCalibration() {
    const c = this.config;
    c.forward = [...DEFAULT_BASE_CONFIG.forward] as [number, number, number];
    c.backLeft = [...DEFAULT_BASE_CONFIG.backLeft] as [number, number, number];
    c.backRight = [...DEFAULT_BASE_CONFIG.backRight] as [number, number, number];
    c.rotate = [...DEFAULT_BASE_CONFIG.rotate] as [number, number, number];
    this.persist();
  }

  /**
   * Drive one wheel pattern for a fixed pulse so the user can watch which way the
   * base actually slid, then label it. Each of the three pairs (one wheel idle)
   * slides the base along one of its primitive directions.
   */
  testPattern(pattern: number[]) {
    if (this.testing) return;
    if (!this.connected) {
      this.error = 'Connect the wheels first.';
      return;
    }
    this.error = null;
    this.testing = true;
    const speeds = new Map(
      this.config.wheelIds.map(
        (id, i) => [id, Math.round(pattern[i] * this.config.speed)] as [number, number],
      ),
    );
    robot.syncWriteWheelSpeed(speeds).catch((e) => {
      this.error = e instanceof Error ? e.message : String(e);
    });
    window.setTimeout(() => {
      this.testing = false;
      this.stopWheels().catch(() => {});
    }, TEST_MS);
  }

  testPair(i: number) {
    this.testPattern(CAL_PAIRS[i]);
  }

  testRotate() {
    this.testPattern([1, 1, 1]);
  }

  /** Record which primitive direction the pair the user just watched maps to. */
  labelPair(i: number, dir: PairDir) {
    const p = CAL_PAIRS[i];
    const neg = [-p[0], -p[1], -p[2]] as [number, number, number];
    const pos = [...p] as [number, number, number];
    const c = this.config;
    if (dir === 'forward') c.forward = pos;
    else if (dir === 'backward') c.forward = neg;
    else if (dir === 'backLeft') c.backLeft = pos;
    else if (dir === 'frontRight') c.backLeft = neg;
    else if (dir === 'backRight') c.backRight = pos;
    else c.backRight = neg;
    this.persist();
  }

  labelRotate(ccw: boolean) {
    this.config.rotate = (ccw ? [1, 1, 1] : [-1, -1, -1]) as [number, number, number];
    this.persist();
  }
}

export const baseLink = new BaseLink();
