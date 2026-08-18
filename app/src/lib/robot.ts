// Thin wrapper around feetech.js (WebSerial control of Feetech SCS/STS servos)
// that mirrors the small slice of the Python RealRobot API the joint-calibration
// flow needs: connect, read a servo, drive servos, torque on/off.
//
// The arm and the LeKiwi wheels share ONE serial bus. Every Feetech command is a
// write-then-read on that single port, so two overlapping calls make the second
// try to lock a stream reader the first still holds ("Cannot get a new reader…").
// Because position streaming (arm) and wheel-speed streaming (base) both fire each
// animation frame, we funnel every SDK access through a one-at-a-time queue so the
// bus is only ever touched by a single in-flight operation.

import { ScsServoSDK } from 'feetech.js';

/**
 * The slice of servo-bus behaviour every backend (local WebSerial, remote
 * WebRTC) must provide. `baseLink.svelte.ts` holds one of these behind a
 * facade so the rest of the app — IK streaming, wheel drive, calibration —
 * never has to know whether the bus is a USB cable or a peer connection to a
 * Pi across the room.
 */
export interface RobotLike {
  connected: boolean;
  connect(): Promise<void>;
  disconnect(): Promise<void>;
  readPosition(servoId: number): Promise<number>;
  syncReadPositions(servoIds: number[]): Promise<Map<number, number>>;
  setTorque(servoIds: number[], enable: boolean): Promise<void>;
  syncWritePositions(targets: Map<number, number>): Promise<'success'>;
  setWheelMode(servoIds: number[]): Promise<void>;
  syncWriteWheelSpeed(speeds: Map<number, number>): Promise<'success'>;
  writeWheelSpeed(servoId: number, speed: number): Promise<'success'>;
}

export class Robot implements RobotLike {
  private sdk = new ScsServoSDK();
  connected = false;
  // Serializes all bus I/O: each call chains after the previous one settles.
  private queue: Promise<unknown> = Promise.resolve();

  private exclusive<T>(fn: () => Promise<T>): Promise<T> {
    const run = this.queue.then(fn, fn);
    // Keep the chain alive regardless of individual success/failure.
    this.queue = run.then(
      () => undefined,
      () => undefined,
    );
    return run;
  }

  /** Prompt for the USB serial device and open the bus. */
  async connect(): Promise<void> {
    await this.exclusive(() => this.sdk.connect());
    this.connected = true;
  }

  async disconnect(): Promise<void> {
    try {
      await this.exclusive(() => this.sdk.disconnect());
    } finally {
      this.connected = false;
    }
  }

  /** Read one servo's raw position (0–4095). */
  readPosition(servoId: number): Promise<number> {
    return this.exclusive(() => this.sdk.readPosition(servoId));
  }

  /** Read several servos at once; returns {servoId -> position}. */
  syncReadPositions(servoIds: number[]): Promise<Map<number, number>> {
    return this.exclusive(() => this.sdk.syncReadPositions(servoIds));
  }

  setTorque(servoIds: number[], enable: boolean): Promise<void> {
    return this.exclusive(async () => {
      for (const id of servoIds) {
        await this.sdk.writeTorqueEnable(id, enable);
      }
    });
  }

  /** Command target positions; `targets` maps servoId -> raw position. */
  syncWritePositions(targets: Map<number, number>): Promise<'success'> {
    return this.exclusive(() => this.sdk.syncWritePositions(targets));
  }

  /** Switch servos into continuous-rotation (wheel) mode — for the base motors. */
  setWheelMode(servoIds: number[]): Promise<void> {
    return this.exclusive(async () => {
      for (const id of servoIds) {
        await this.sdk.setWheelMode(id);
      }
    });
  }

  /** Command wheel speeds; `speeds` maps servoId -> signed raw speed. */
  syncWriteWheelSpeed(speeds: Map<number, number>): Promise<'success'> {
    return this.exclusive(() => this.sdk.syncWriteWheelSpeed(speeds));
  }

  /** Spin one wheel at a signed raw speed (for calibration/test). */
  writeWheelSpeed(servoId: number, speed: number): Promise<'success'> {
    return this.exclusive(() => this.sdk.writeWheelSpeed(servoId, speed));
  }
}
