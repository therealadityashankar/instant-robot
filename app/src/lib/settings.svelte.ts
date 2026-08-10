// Global preferences that belong to the whole app rather than to one panel, so
// they can live behind the header's Settings button instead of cluttering the
// controls you use while driving the robot.

class Settings {
  open = $state(false); // is the Settings modal showing?

  /** Cap on how fast any arm joint moves, degrees/second. Applies everywhere. */
  maxArmSpeedDeg = $state(35);

  // ── camera bandwidth ───────────────────────────────────────────────────────
  // Two USB cameras can exceed what the bus will carry at once; these are the
  // two levers for that.
  /** Connecting one camera releases the other. */
  exclusiveCam = $state(true);
  /** Requested capture width; height follows 4:3. Applies on next connect. */
  camResW = $state(640);

  // ── arm mounting offset ────────────────────────────────────────────────────
  // Where the arm sits on the base (m). Owned by the Simulator (it's per-robot
  // and persisted per-robot), mirrored here so Settings can edit it without the
  // whole robot model having to move into a store.
  armOffset = $state<[number, number, number] | null>(null);
  setArmOffset: ((axis: 0 | 1 | 2, value: number) => void) | null = null;
}

export const settings = new Settings();
