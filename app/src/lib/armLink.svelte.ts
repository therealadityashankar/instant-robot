// Shared connection state for the arm's servo bus.
//
// The connect/disconnect *logic* lives in Simulator.svelte (it also has to set
// torque, seed the sim from the live pose and probe for wheels), but the button
// belongs in the app header. So the Simulator registers its handlers here on
// mount and keeps `connected` in sync; the header just reads the state and calls
// whatever is registered.
//
// Two ways in — local (WebSerial, same machine) or remote (WebRTC, a Pi
// somewhere else) — so the header shows a small chooser instead of connecting
// straight away. `chooserOpen` is owned here so it survives regardless of which
// view is mounted.

import type { RemoteRobotOptions } from './remoteRobot';

class ArmLink {
  connected = $state(false);
  busy = $state(false);
  error = $state<string | null>(null);
  /** True while the header is showing the local/remote picker. */
  chooserOpen = $state(false);
  /** Registered by the Simulator once it's ready; null until then. */
  connectLocal: (() => Promise<void>) | null = $state(null);
  connectRemote: ((opts: RemoteRobotOptions) => Promise<void>) | null = $state(null);
  disconnect: (() => Promise<void>) | null = $state(null);

  async run(fn: (() => Promise<void>) | null) {
    if (this.busy || !fn) return;
    this.busy = true;
    this.error = null;
    try {
      await fn();
    } finally {
      this.busy = false;
    }
  }

  /** Header button: connected -> disconnect; not connected -> show the picker. */
  toggle() {
    if (this.busy) return;
    if (this.connected) {
      this.run(this.disconnect);
    } else {
      this.chooserOpen = true;
    }
  }

  pickLocal() {
    this.chooserOpen = false;
    this.run(this.connectLocal);
  }

  pickRemote(opts: RemoteRobotOptions) {
    this.chooserOpen = false;
    this.run(this.connectRemote ? () => this.connectRemote!(opts) : null);
  }
}

export const armLink = new ArmLink();
