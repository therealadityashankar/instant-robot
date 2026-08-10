// Shared connection state for the arm's servo bus.
//
// The connect/disconnect *logic* lives in Simulator.svelte (it also has to set
// torque, seed the sim from the live pose and probe for wheels), but the button
// belongs in the app header. So the Simulator registers its handlers here on
// mount and keeps `connected` in sync; the header just reads the state and calls
// whatever is registered.

class ArmLink {
  connected = $state(false);
  busy = $state(false);
  error = $state<string | null>(null);
  /** Registered by the Simulator once it's ready; null until then. */
  connect: (() => Promise<void>) | null = $state(null);
  disconnect: (() => Promise<void>) | null = $state(null);

  /** Run connect or disconnect, whichever applies, guarding against re-entry. */
  async toggle() {
    if (this.busy) return;
    const fn = this.connected ? this.disconnect : this.connect;
    if (!fn) return;
    this.busy = true;
    try {
      await fn();
    } finally {
      this.busy = false;
    }
  }
}

export const armLink = new ArmLink();
