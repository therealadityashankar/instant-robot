# instant-robot · app

A browser (Svelte + TypeScript) port of the SO-101 calibration tools —
[`calibration/calibrate_board.py`](../calibration/calibrate_board.py) and
[`calibration/calibrate_joints_real.py`](../calibration/calibrate_joints_real.py).
Everything runs client-side. Three tabs:

- **Calibrate** — opens your webcam, detects the printed ArUco board's border
  tags, computes a homography to a canonical bird's-eye view, and runs the same
  4-corner perspective calibration as the Python tool.
- **Test calibration** — loads a saved board calibration and overlays each block's
  detected vs. corrected footprint live on the rectified view.
- **Joint calibration** — connects to the arm's six Feetech servos over WebSerial
  and runs the joint-by-joint linear calibration (`sim = scale·real + offset`),
  with optional verification poses.
- **Simulator (IK)** — loads the SO-101 MJCF into MuJoCo (WASM), renders it with
  three.js, and solves inverse kinematics so you can drag an end-effector target
  and watch the arm reach it. A `sim ⇄ real` toggle streams the solved joint
  angles to the physical servos (through a loaded joint calibration). In real
  mode it also paints the calibration board into the scene, detects a block via
  the overhead camera, copies its position into the sim, and runs a
  **pick** (approach → descend → grasp → lift) on both the sim and the real arm.

ArUco detection, `findHomography` and `warpPerspective` are provided by OpenCV.js
via the [`@techstark/opencv-js`](https://www.npmjs.com/package/@techstark/opencv-js)
npm package — a build compiled *with* the contrib `aruco` module (the stock
docs.opencv.org build omits it), bundled locally so nothing loads cross-origin.
Servo I/O uses [`feetech.js`](https://www.npmjs.com/package/feetech.js) (the
[bambot](https://github.com/timqian/bambot) WebSerial SDK).

## Inverse kinematics & the simulator

The Simulator tab uses the MuJoCo WASM bindings
([`mujoco-js`](https://www.npmjs.com/package/mujoco-js) on npm — a
**single-threaded** build of the official DeepMind bindings) — not for physics,
but as a shared kinematics + rendering backend. Single-threaded matters: the
pthread builds busy-spin the browser's main thread (it can't `Atomics.wait`) and
peg the CPU, so we avoid them.

- **IK** is a position-only damped-least-squares loop
  ([`lib/ik.ts`](src/lib/ik.ts)) built on `mj_jacSite` / `mj_forward` — the same
  scheme dm_control's `qpos_from_site_pose` uses. There is no single "solve IK"
  call in MuJoCo (or its Python API); this is the standard Jacobian approach.
- **Rendering** ([`lib/mujocoRender.ts`](src/lib/mujocoRender.ts)) builds one
  three.js mesh per MuJoCo geom straight from the *compiled* model (primitives
  from `geom_size`, meshes from `mesh_vert`/`mesh_face` — no STL re-parsing) and
  positions them each frame from `geom_xpos`/`geom_xmat`.
- **Testability** — because IK runs against MuJoCo's own FK, `mj_forward` is an
  independent ground-truth oracle. [`test/ik.test.ts`](test/ik.test.ts) loads the
  real model headlessly (no browser, no canvas) and asserts the solver reaches
  FK-generated targets within 1 mm. Run with `npm test`.

The MuJoCo runtime (`mujoco_wasm.js`, with the wasm embedded — a single file) is
served from [`public/mujoco/`](public/mujoco/) and loaded at runtime, because
Vite can't bundle the Emscripten glue's `import.meta.url` lookup. Model files live
in [`public/models/so101/`](public/models/so101/). Being single-threaded, it needs
no `SharedArrayBuffer` and no cross-origin-isolation headers, so it deploys as
plain static hosting.

> **Memory note:** MuJoCo's accessor *methods* (`data.site(id)`, `model.body(id)`)
> return C++ handles that are **not** garbage-collected and must be `.delete()`'d.
> Never call them in a hot loop — read the flat views instead (`data.site_xpos`,
> `data.geom_xpos`, `data.qpos`), which is what the IK and renderer do.

## Joint calibration notes

The Python tool pre-renders MuJoCo reference images of each joint pose. That sim
step is **not** reproduced here — it would require shipping the SO-101 model and
the MuJoCo WASM runtime — so the browser tab shows each pose's target angle
numerically instead. The capture → per-joint linear fit → JSON export → verification
flow is otherwise faithful. Servos are addressed by ID 1–6 in joint order
(`shoulder_pan` → `gripper`); WebSerial needs a Chromium-based browser.

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
| `src/lib/joints.ts` | Joint plan, verification poses, linear fit, sim→servo conversion. |
| `src/lib/robot.ts` | `feetech.js` wrapper: connect, read/write servos, torque. |
| `src/lib/mujocoSession.ts` | MuJoCo WASM loader + FS model mounting (browser & Node). |
| `src/lib/ik.ts` | Jacobian (damped-least-squares) IK on a MuJoCo model. |
| `src/lib/mujocoRender.ts` | three.js renderer built from MuJoCo geom data. |
| `src/lib/boardSim.ts` | Board⇄sim coordinate mapping + injects board/block geoms into the model. |
| `src/lib/pick.ts` | Pick state machine (approach/descend/grasp/lift) → IK targets. |
| `test/ik.test.ts`, `test/boardSim.test.ts`, `test/pick.test.ts` | Headless tests (`npm test`). |
| `src/App.svelte` | Camera loop, board-calibration state machine, tab shell. |
| `src/JointCalibration.svelte` | Joint-calibration tab: connect, capture, fit, verify. |
| `src/Simulator.svelte` | Simulator tab: MuJoCo viewer, IK target, sim/real toggle. |
