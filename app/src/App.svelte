<script lang="ts">
  import { onMount } from 'svelte';
  import './app.css';
  import { loadCv, type Cv } from './lib/cv';
  import {
    boardTagCentres,
    OUT_W,
    OUT_H,
    type TagCentres,
  } from './lib/board';
  import {
    createDetector,
    detectMarkers,
    computeHomography,
    tagInteriorPos,
    type Detector,
  } from './lib/homography';
  import { drawRawPanel, drawRectifiedPanel } from './lib/render';
  import { fitLinear2d, formatFitReport, type LinearFit } from './lib/geometry';
  import { DEFAULT_PARAMS, type BoardParams, type CalibState } from './lib/types';
  import JointCalibration from './JointCalibration.svelte';
  import Simulator from './Simulator.svelte';

  let params = $state<BoardParams>({ ...DEFAULT_PARAMS });

  let cvReady = $state(false);
  let errorMsg = $state<string | null>(null);
  let running = $state(false);
  let statusText = $state('Loading OpenCV.js…');
  let statusClass = $state<'ok' | 'warn' | 'bad'>('warn');
  let report = $state<string | null>(null);

  // ── Mode: calibrate vs. test an existing calibration ──────────────────────
  type Correction = { Sx: number; Bx: number; Sy: number; By: number };
  let mode = $state<'calibrate' | 'test' | 'joints' | 'sim'>('calibrate');
  let loadedCorr = $state<Correction | null>(null);
  let loadedSource = $state<string | null>(null);
  interface Readout {
    id: number;
    rawX: number;
    rawY: number;
    calX: number;
    calY: number;
  }
  let readout = $state<Readout[]>([]);

  let rawCanvas: HTMLCanvasElement;
  let rectCanvas: HTMLCanvasElement;
  let video: HTMLVideoElement;

  let cv: Cv;
  let detector: Detector;
  let stream: MediaStream | null = null;
  let rafId = 0;

  // ── Derived board geometry ────────────────────────────────────────────────
  function interiorInsetMm() {
    return 2 * params.tagMm + params.gapMm;
  }
  function interiorSizeMm() {
    return params.squareMm - 2 * interiorInsetMm();
  }
  function innerTagsStart() {
    return params.nOuter * 2 + (params.nOuter - 2) * 2;
  }
  function cornerPositions(): Array<[number, number]> {
    const hw = params.blockW / 2;
    const hd = params.blockD / 2;
    const iW = interiorSizeMm();
    const iH = interiorSizeMm();
    return [
      [hw, hd],
      [iW - hw, hd],
      [hw, iH - hd],
      [iW - hw, iH - hd],
    ];
  }

  const calib: CalibState = $state({
    active: false,
    done: false,
    step: 0,
    data: [],
    donePx: [],
    cornerPositions: cornerPositions(),
    blockTag: params.blockTag,
    blockW: params.blockW,
    blockD: params.blockD,
  });

  let lastCorners: Map<number, any> = new Map();
  let lastH: any = null;
  let lastHValid = false;

  // ── Calibration control ───────────────────────────────────────────────────
  function toggleCalibration() {
    const start = !calib.active || calib.done;
    calib.active = start;
    calib.done = false;
    calib.step = 0;
    calib.data = [];
    calib.donePx = [];
    calib.cornerPositions = cornerPositions();
    calib.blockTag = params.blockTag;
    calib.blockW = params.blockW;
    calib.blockD = params.blockD;
    if (!start) report = null;
  }

  function recordCorner() {
    if (!calib.active || calib.done) return;
    const step = calib.step;
    if (!lastHValid || !lastH) {
      statusText = `[${'TLTRBLBR'.slice(step * 2, step * 2 + 2)}] No homography — need more border tags`;
      return;
    }
    const corners = lastCorners.get(params.blockTag);
    if (!corners) {
      statusText = `Tag ID ${params.blockTag} not detected`;
      return;
    }
    const scale = OUT_W / params.squareMm;
    const [ox, oy] = tagInteriorPos(cv, corners, lastH, scale, interiorInsetMm());
    const [trueX, trueY] = calib.cornerPositions[step];
    calib.data.push([trueX, trueY, ox, oy, params.blockHeight]);
    const insetPx = Math.round(interiorInsetMm() * scale);
    calib.donePx.push([insetPx + trueX * scale, insetPx + trueY * scale]);
    calib.step += 1;

    if (calib.step >= 4) {
      calib.active = false;
      calib.done = true;
      const fit = fitLinear2d(calib.data);
      report = formatFitReport(fit);
      lastFit = fit;
    }
  }

  let lastFit: LinearFit | null = $state(null);

  function downloadCalibration() {
    if (!lastFit) return;
    const payload = {
      Sx: lastFit.Sx,
      Bx: lastFit.Bx,
      Sy: lastFit.Sy,
      By: lastFit.By,
      rms: lastFit.rms,
      measurements: lastFit.measurements,
      params,
    };
    const blob = new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'camera_calibration.json';
    a.click();
    URL.revokeObjectURL(url);
  }

  function onKey(e: KeyboardEvent) {
    if (e.target instanceof HTMLInputElement) return;
    if (e.key === 'c' || e.key === 'C') toggleCalibration();
    else if (e.key === ' ') {
      e.preventDefault();
      recordCorner();
    }
  }

  // ── Main loop ─────────────────────────────────────────────────────────────
  function processFrame(
    grabCtx: CanvasRenderingContext2D,
    src: any,
    gray: any,
    tagCentresMm: TagCentres,
  ) {
    // Grab the current video frame into the reused RGBA Mat.
    const { width, height } = grabCtx.canvas;
    grabCtx.drawImage(video, 0, 0, width, height);
    const frame = grabCtx.getImageData(0, 0, width, height);
    src.data.set(frame.data);
    cv.cvtColor(src, gray, cv.COLOR_RGBA2GRAY);

    const cornersDict = detectMarkers(cv, detector, gray);
    const { H, inliers } = computeHomography(cv, cornersDict, tagCentresMm, params.squareMm);

    lastCorners = cornersDict;
    if (lastH) lastH.delete();
    lastH = H;
    lastHValid = !!H;

    const rawCtx = rawCanvas.getContext('2d')!;
    const rectCtx = rectCanvas.getContext('2d')!;
    drawRawPanel(cv, rawCtx, video, cornersDict, tagCentresMm, H, params.squareMm);
    const testOverlay =
      mode === 'test' && loadedCorr
        ? { corr: loadedCorr, blockW: params.blockW, blockD: params.blockD }
        : null;
    drawRectifiedPanel(
      cv, rectCtx, src, H, params.squareMm, params.tagMm, params.gapMm,
      cornersDict, tagCentresMm, innerTagsStart(), calib, testOverlay,
    );

    updateStatus(cornersDict, tagCentresMm, inliers, !!H);

    if (mode === 'test') computeReadout(cornersDict, H, tagCentresMm);
    else if (readout.length) readout = [];
  }

  /** In test mode, apply the loaded correction to every detected object tag. */
  function computeReadout(
    cornersDict: Map<number, any>,
    H: any | null,
    tagCentresMm: TagCentres,
  ) {
    if (!H || !loadedCorr) {
      if (readout.length) readout = [];
      return;
    }
    const scale = OUT_W / params.squareMm;
    const inset = interiorInsetMm();
    const rows: Readout[] = [];
    for (const [id, corners] of cornersDict) {
      if (tagCentresMm.has(id)) continue; // border tags aren't objects
      if (id < 100 || id >= 200) continue;
      const [rawX, rawY] = tagInteriorPos(cv, corners, H, scale, inset);
      rows.push({
        id,
        rawX,
        rawY,
        calX: loadedCorr.Sx * rawX + loadedCorr.Bx,
        calY: loadedCorr.Sy * rawY + loadedCorr.By,
      });
    }
    rows.sort((a, b) => a.id - b.id);
    readout = rows;
  }

  function useCurrentFit() {
    if (!lastFit) return;
    loadedCorr = { Sx: lastFit.Sx, Bx: lastFit.Bx, Sy: lastFit.Sy, By: lastFit.By };
    loadedSource = 'current session fit';
  }

  async function loadCalibrationFile(e: Event) {
    const input = e.target as HTMLInputElement;
    const file = input.files?.[0];
    if (!file) return;
    try {
      const data = JSON.parse(await file.text());
      const { Sx, Bx, Sy, By } = data;
      if ([Sx, Bx, Sy, By].some((v) => typeof v !== 'number')) {
        throw new Error('missing Sx/Bx/Sy/By');
      }
      loadedCorr = { Sx, Bx, Sy, By };
      loadedSource = file.name;
    } catch (err) {
      errorMsg = `Could not read calibration: ${err instanceof Error ? err.message : String(err)}`;
    }
    input.value = '';
  }

  function updateStatus(
    cornersDict: Map<number, any>,
    tagCentresMm: TagCentres,
    inliers: number,
    hasH: boolean,
  ) {
    let nBorder = 0;
    let nObj = 0;
    for (const id of cornersDict.keys()) {
      if (tagCentresMm.has(id)) nBorder++;
      else if (id >= 100 && id < 200) nObj++;
    }

    if (calib.done) {
      statusText = 'Calibration complete — download below. C=redo';
      statusClass = 'ok';
    } else if (calib.active) {
      const step = calib.step;
      const name = ['TL', 'TR', 'BL', 'BR'][step];
      const [tx, ty] = calib.cornerPositions[step];
      const hasTag = cornersDict.has(params.blockTag) && hasH;
      let obs = '';
      if (hasTag) {
        const scale = OUT_W / params.squareMm;
        const [ox, oy] = tagInteriorPos(
          cv, cornersDict.get(params.blockTag), lastH, scale, interiorInsetMm(),
        );
        obs = `  obs=(${ox.toFixed(1)},${oy.toFixed(1)})`;
      }
      statusText = `Corner ${step + 1}/4: place block at ${name} — tag→(${tx.toFixed(1)},${ty.toFixed(1)})mm${obs}  SPACE=record  C=abort`;
      statusClass = hasTag ? 'ok' : 'warn';
    } else {
      statusText = `Border: ${nBorder}/${tagCentresMm.size}  Inliers: ${hasH ? inliers : 0}  Objects: ${nObj}  ${hasH ? '[OK]' : '[NO HOMOGRAPHY]'}  C=calibrate`;
      statusClass = hasH ? 'ok' : 'bad';
    }
  }

  async function start() {
    try {
      cv = await loadCv();
      cvReady = true;
      detector = createDetector(cv);

      stream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: 'environment' },
      });
      video.srcObject = stream;
      await video.play();

      const w = video.videoWidth || 640;
      const h = video.videoHeight || 480;
      rawCanvas.width = w;
      rawCanvas.height = h;
      rectCanvas.width = OUT_W;
      rectCanvas.height = OUT_H;

      // Offscreen canvas used to pull frames out of the <video> each tick.
      const grab = document.createElement('canvas');
      grab.width = w;
      grab.height = h;
      const grabCtx = grab.getContext('2d', { willReadFrequently: true })!;
      const src = new cv.Mat(h, w, cv.CV_8UC4);
      const gray = new cv.Mat();
      running = true;

      const loop = () => {
        if (!running) return;
        // The camera panels are unmounted on the joints/sim tabs — skip the frame.
        if (mode === 'joints' || mode === 'sim' || !rawCanvas || !rectCanvas) {
          rafId = requestAnimationFrame(loop);
          return;
        }
        const tagCentresMm = boardTagCentres(
          params.squareMm, params.tagMm, params.gapMm, params.nOuter, params.nInner,
        );
        try {
          processFrame(grabCtx, src, gray, tagCentresMm);
        } catch (err) {
          console.error(err);
        }
        rafId = requestAnimationFrame(loop);
      };
      loop();
    } catch (err) {
      errorMsg = err instanceof Error ? err.message : String(err);
      statusText = 'Failed to start: ' + errorMsg;
      statusClass = 'bad';
    }
  }

  onMount(() => {
    window.addEventListener('keydown', onKey);
    start();
    return () => {
      running = false;
      cancelAnimationFrame(rafId);
      window.removeEventListener('keydown', onKey);
      stream?.getTracks().forEach((t) => t.stop());
      if (lastH) lastH.delete();
    };
  });
</script>

<div class="app">
  <h1>instant-robot · board calibration</h1>
  <p class="subtitle">
    Browser port of <code>calibration/calibrate_board.py</code> — live homography viewer &amp;
    4-corner perspective calibration.
  </p>

  <!-- Hidden source element; frames are read from it into OpenCV each tick. -->
  <video bind:this={video} playsinline muted style="display:none"></video>

  <div class="tabs">
    <button class:active={mode === 'calibrate'} onclick={() => (mode = 'calibrate')}>
      Calibrate
    </button>
    <button class:active={mode === 'test'} onclick={() => (mode = 'test')}>
      Test calibration
    </button>
    <button class:active={mode === 'joints'} onclick={() => (mode = 'joints')}>
      Joint calibration
    </button>
    <button class:active={mode === 'sim'} onclick={() => (mode = 'sim')}>
      Simulator (IK)
    </button>
  </div>

  {#if mode === 'joints'}
    <JointCalibration />
  {:else if mode === 'sim'}
    <Simulator />
  {:else}
  <div class="layout">
    <div>
      <div class="panels">
        <canvas bind:this={rawCanvas} width="640" height="480"></canvas>
        <canvas bind:this={rectCanvas} width={OUT_W} height={OUT_H}></canvas>
      </div>

      <div class="status {statusClass}">{statusText}</div>

      {#if mode === 'calibrate'}
        <div class="controls">
          <button class="primary" onclick={toggleCalibration} disabled={!running}>
            {calib.active ? 'Abort calibration' : calib.done ? 'Redo calibration' : 'Start calibration (C)'}
          </button>
          <button onclick={recordCorner} disabled={!calib.active}>Record corner (Space)</button>
          <button onclick={downloadCalibration} disabled={!lastFit}>Download calibration</button>
        </div>

        {#if report}
          <pre class="report">{report}</pre>
        {/if}
      {:else}
        <div class="controls">
          <button class="primary" onclick={useCurrentFit} disabled={!lastFit}>
            Use current session fit
          </button>
          <label class="file-btn">
            Load calibration JSON…
            <input type="file" accept="application/json,.json" onchange={loadCalibrationFile} />
          </label>
        </div>

        {#if loadedCorr}
          <div class="status ok">
            Active correction ({loadedSource}):
            X' = {loadedCorr.Sx.toFixed(4)}·x {loadedCorr.Bx >= 0 ? '+' : '−'} {Math.abs(loadedCorr.Bx).toFixed(2)},
            Y' = {loadedCorr.Sy.toFixed(4)}·y {loadedCorr.By >= 0 ? '+' : '−'} {Math.abs(loadedCorr.By).toFixed(2)}
          </div>

          <table class="readout">
            <thead>
              <tr><th>Tag</th><th>Raw X</th><th>Raw Y</th><th>Corrected X</th><th>Corrected Y</th></tr>
            </thead>
            <tbody>
              {#if readout.length === 0}
                <tr><td colspan="5" class="empty">No object tags detected…</td></tr>
              {:else}
                {#each readout as r (r.id)}
                  <tr>
                    <td>{r.id}</td>
                    <td>{r.rawX.toFixed(1)}</td>
                    <td>{r.rawY.toFixed(1)}</td>
                    <td class="cal">{r.calX.toFixed(1)}</td>
                    <td class="cal">{r.calY.toFixed(1)}</td>
                  </tr>
                {/each}
              {/if}
            </tbody>
          </table>
        {:else}
          <p class="hint">
            Load a calibration to test — either <em>Use current session fit</em> (after running a
            calibration in the Calibrate tab) or upload a previously downloaded
            <code>camera_calibration.json</code>. Then hold a tagged block on the board to see its
            raw and corrected interior position (mm) live.
          </p>
        {/if}
      {/if}

      {#if errorMsg}
        <div class="status bad">Error: {errorMsg}</div>
      {/if}
    </div>

    <div class="sidebar">
      <h2>Board parameters</h2>
      <div class="params">
        <label for="squareMm">Square (mm)</label>
        <input id="squareMm" type="number" bind:value={params.squareMm} />
        <label for="tagMm">Tag (mm)</label>
        <input id="tagMm" type="number" bind:value={params.tagMm} />
        <label for="gapMm">Gap (mm)</label>
        <input id="gapMm" type="number" bind:value={params.gapMm} />
        <label for="nOuter">n outer</label>
        <input id="nOuter" type="number" bind:value={params.nOuter} />
        <label for="nInner">n inner</label>
        <input id="nInner" type="number" bind:value={params.nInner} />
        <label for="blockTag">Block tag ID</label>
        <input id="blockTag" type="number" bind:value={params.blockTag} />
        <label for="blockHeight">Block height (mm)</label>
        <input id="blockHeight" type="number" bind:value={params.blockHeight} />
        <label for="blockW">Block width (mm)</label>
        <input id="blockW" type="number" bind:value={params.blockW} />
        <label for="blockD">Block depth (mm)</label>
        <input id="blockD" type="number" bind:value={params.blockD} />
      </div>

      {#if mode === 'calibrate'}
        <h2 style="margin-top:1rem">How to calibrate</h2>
        <p class="hint">
          Press <kbd>C</kbd> to begin. Place one block (tag ID {params.blockTag}) at each interior
          corner in order TL → TR → BL → BR, its corner touching the interior corner, then press
          <kbd>Space</kbd> to record. After 4 corners a per-axis linear fit is computed and can be
          downloaded as JSON.
        </p>
      {:else}
        <h2 style="margin-top:1rem">How to test</h2>
        <p class="hint">
          Load a calibration, then move a tagged block (any bordered ID 100–199) around the board.
          The table shows each tag's raw detected interior position and the position after applying
          the linear correction — corrected values should track the block's true mm coordinates.
        </p>
      {/if}
      {#if !cvReady}
        <p class="hint">Waiting for OpenCV.js…</p>
      {/if}
    </div>
  </div>
  {/if}
</div>
