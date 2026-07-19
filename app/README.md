# instant-robot · app

A browser (Svelte + TypeScript) port of [`calibration/calibrate_board.py`](../calibration/calibrate_board.py).

It opens your webcam, detects the printed ArUco board's border tags, computes a
homography to a canonical bird's-eye view, and runs the same 4-corner perspective
calibration as the Python tool — all client-side. ArUco detection, `findHomography`
and `warpPerspective` are provided by [OpenCV.js](https://docs.opencv.org/) (loaded
from a CDN in `index.html`).

## Run

```bash
cd app
npm install
npm run dev      # open the printed localhost URL
```

`npm run build` produces a static bundle in `dist/`.

> Camera access needs a secure context — `localhost` (dev) or HTTPS in production.

## Using it

- **Left panel** — raw camera feed with detected tags and the board outline.
- **Right panel** — rectified bird's-eye view with a mm grid and detected objects.

### Calibration

1. Press <kbd>C</kbd> (or *Start calibration*).
2. Place one block (tag ID 101 by default) at each interior corner in order
   **TL → TR → BL → BR**, its corner touching the interior corner.
3. Press <kbd>Space</kbd> to record each corner.
4. After 4 corners a per-axis linear correction (`Sx, Bx, Sy, By`) is fit and shown.
   *Download calibration* saves it as JSON (the browser equivalent of the Python
   tool's `camera_calibration.npz`).

Board and block dimensions are editable in the sidebar (defaults match the Python
script's argparse defaults).

## Layout

| File | Purpose |
| --- | --- |
| `src/lib/board.ts` | Board geometry & constants (`boardTagCentres`, offsets). |
| `src/lib/geometry.ts` | Pure helpers: bordered-tag footprint, linear fit. |
| `src/lib/homography.ts` | OpenCV.js wrappers: detection, homography, transforms. |
| `src/lib/render.ts` | Canvas rendering of the raw & rectified panels. |
| `src/App.svelte` | Camera loop, calibration state machine, UI. |
