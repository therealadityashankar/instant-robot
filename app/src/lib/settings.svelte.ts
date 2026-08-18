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

  // ── OpenRouter AI Assistant ────────────────────────────────────────────────
  openrouterApiKey = $state(
    typeof localStorage !== 'undefined' ? localStorage.getItem('openrouter_api_key') || '' : ''
  );
  openrouterModel = $state(
    typeof localStorage !== 'undefined' ? localStorage.getItem('openrouter_model') || 'google/gemini-3.7-flash' : 'google/gemini-3.7-flash'
  );

  setOpenrouterApiKey(key: string) {
    this.openrouterApiKey = key;
    if (typeof localStorage !== 'undefined') {
      localStorage.setItem('openrouter_api_key', key);
    }
  }

  setOpenrouterModel(model: string) {
    this.openrouterModel = model;
    if (typeof localStorage !== 'undefined') {
      localStorage.setItem('openrouter_model', model);
    }
  }
}

export const settings = new Settings();

