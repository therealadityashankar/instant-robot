// Thin wrapper around feetech.js (WebSerial control of Feetech SCS/STS servos)
// that mirrors the small slice of the Python RealRobot API the joint-calibration
// flow needs: connect, read a servo, drive servos, torque on/off.

import { ScsServoSDK } from 'feetech.js';

export class Robot {
  private sdk = new ScsServoSDK();
  connected = false;

  /** Prompt for the USB serial device and open the bus. */
  async connect(): Promise<void> {
    await this.sdk.connect();
    this.connected = true;
  }

  async disconnect(): Promise<void> {
    try {
      await this.sdk.disconnect();
    } finally {
      this.connected = false;
    }
  }

  /** Read one servo's raw position (0–4095). */
  readPosition(servoId: number): Promise<number> {
    return this.sdk.readPosition(servoId);
  }

  /** Read several servos at once; returns {servoId -> position}. */
  syncReadPositions(servoIds: number[]): Promise<Map<number, number>> {
    return this.sdk.syncReadPositions(servoIds);
  }

  async setTorque(servoIds: number[], enable: boolean): Promise<void> {
    for (const id of servoIds) {
      await this.sdk.writeTorqueEnable(id, enable);
    }
  }

  /** Command target positions; `targets` maps servoId -> raw position. */
  syncWritePositions(targets: Map<number, number>): Promise<'success'> {
    return this.sdk.syncWritePositions(targets);
  }
}
