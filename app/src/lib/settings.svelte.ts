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

  // ── Navigation & Approach Tuning ──────────────────────────────────────────
  /** Standoff distance for OCR / Station Scanning (meters). */
  navScanStandoff = $state(0.34);
  /** Forward drive time (seconds) to dock at station during "Go here" (0.0 - 12.0s). */
  navWorkForwardSec = $state(
    typeof localStorage !== 'undefined' && localStorage.getItem('nav_work_forward_sec') !== null
      ? parseFloat(localStorage.getItem('nav_work_forward_sec') || '8.5') || 8.5
      : 8.5
  );
  /** Backwards compatibility alias for navWorkForwardSec */
  get navWorkStandoff(): number {
    return this.navWorkForwardSec;
  }
  set navWorkStandoff(val: number) {
    this.setNavWorkForwardSec(val);
  }

  setNavWorkForwardSec(sec: number) {
    this.navWorkForwardSec = sec;
    if (typeof localStorage !== 'undefined') {
      localStorage.setItem('nav_work_forward_sec', String(sec));
    }
  }
  /** Additional forward crawl drive time (seconds) after visual servo lock. */
  navExtraForwardDurationSec = $state(0.0);
  /** Speed units for additional forward crawl (0.05 - 0.40). */
  navExtraForwardSpeed = $state(0.15);
  /** Max apparent tag size (pixel height) before forward stop cutoff. */
  maxTagSizePx = $state(140);
  /** Target ratio of ArUco marker side width to camera frame width for standoff regulation (0.02 - 0.15). */
  tagCameraRatio = $state(
    typeof localStorage !== 'undefined'
      ? parseFloat(localStorage.getItem('tag_camera_ratio') || '0.05') || 0.05
      : 0.05
  );

  /** Target tag side-size in pixels (at 640px base camera resolution). */
  get scanTagSidePx(): number {
    return Math.round(this.tagCameraRatio * 640);
  }
  set scanTagSidePx(px: number) {
    this.setTagCameraRatio(px / 640);
  }

  setTagCameraRatio(ratio: number) {
    this.tagCameraRatio = ratio;
    if (typeof localStorage !== 'undefined') {
      localStorage.setItem('tag_camera_ratio', String(ratio));
    }
  }

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
