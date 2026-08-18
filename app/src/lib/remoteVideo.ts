// Holds the latest JPEG frame from each remote camera, decoded into a canvas.
//
// The Pi sends whole JPEGs down an unreliable datachannel (see remoteProtocol.ts).
// Everything downstream in the app — ArUco detection, the preview panels — already
// accepts any `CanvasImageSource`, so decoding each frame onto a canvas lets the
// remote cameras take exactly the same path a local webcam does. No branch in the
// detector, no second pipeline.
//
// Decoding is `createImageBitmap`, which runs off the main thread; frames that
// arrive while one is still decoding are dropped rather than queued. On a lossy
// channel a queue would only ever accumulate stale frames — the newest frame is
// the only one worth having.

export class RemoteCameraSink {
  /** One canvas per camera index, created lazily as frames arrive. */
  private canvases = new Map<number, HTMLCanvasElement>();
  private decoding = new Set<number>();
  /** Bumped on every decoded frame so callers can tell "new frame" from "same frame". */
  frameCount = new Map<number, number>();
  /** Called after each successful decode, so the UI can react (e.g. flip a flag). */
  onFrame: ((camera: number) => void) | null = null;

  /** The canvas for a camera, or null if it hasn't produced a decodable frame yet. */
  canvas(camera: number): HTMLCanvasElement | null {
    const c = this.canvases.get(camera);
    return c && c.width > 0 ? c : null;
  }

  has(camera: number): boolean {
    return this.canvas(camera) !== null;
  }

  /** Number of cameras that have produced at least one frame. */
  get activeCount(): number {
    return [...this.canvases.values()].filter((c) => c.width > 0).length;
  }

  async push(camera: number, jpeg: Uint8Array): Promise<void> {
    if (this.decoding.has(camera)) return; // drop rather than queue — see header
    this.decoding.add(camera);
    try {
      // Copy: the incoming view aliases the datachannel's buffer, which the next
      // message may reuse before the async decode finishes.
      const blob = new Blob([jpeg.slice()], { type: 'image/jpeg' });
      const bmp = await createImageBitmap(blob);
      let canvas = this.canvases.get(camera);
      if (!canvas) {
        canvas = document.createElement('canvas');
        this.canvases.set(camera, canvas);
      }
      if (canvas.width !== bmp.width || canvas.height !== bmp.height) {
        canvas.width = bmp.width;
        canvas.height = bmp.height;
      }
      const ctx = canvas.getContext('2d');
      ctx?.drawImage(bmp, 0, 0);
      bmp.close();
      this.frameCount.set(camera, (this.frameCount.get(camera) ?? 0) + 1);
      this.onFrame?.(camera);
    } catch {
      // A corrupt frame is expected occasionally on a lossy channel; the next
      // one supersedes it, so there's nothing useful to do here.
    } finally {
      this.decoding.delete(camera);
    }
  }

  clear(): void {
    this.canvases.clear();
    this.decoding.clear();
    this.frameCount.clear();
  }
}
