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

  let params = $state<BoardParams>({ ...DEFAULT_PARAMS });

  let cvReady = $state(false);
  let errorMsg = $state<string | null>(null);
  let running = $state(false);
  let statusText = $state('Loading OpenCV.js…');
  let statusClass = $state<'ok' | 'warn' | 'bad'>('warn');
  let report = $state<string | null>(null);

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
  function processFrame(cap: any, src: any, gray: any, tagCentresMm: TagCentres) {
    cap.read(src);
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
    drawRectifiedPanel(
      cv, rectCtx, src, H, params.squareMm, params.tagMm, params.gapMm,
      cornersDict, tagCentresMm, innerTagsStart(), calib,
    );

    updateStatus(cornersDict, tagCentresMm, inliers, !!H);
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

      const cap = new cv.VideoCapture(video);
      const src = new cv.Mat(h, w, cv.CV_8UC4);
      const gray = new cv.Mat();
      running = true;

      const loop = () => {
        if (!running) return;
        const tagCentresMm = boardTagCentres(
          params.squareMm, params.tagMm, params.gapMm, params.nOuter, params.nInner,
        );
        try {
          processFrame(cap, src, gray, tagCentresMm);
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

  <div class="layout">
    <div>
      <div class="panels">
        <canvas bind:this={rawCanvas} width="640" height="480"></canvas>
        <canvas bind:this={rectCanvas} width={OUT_W} height={OUT_H}></canvas>
      </div>

      <div class="status {statusClass}">{statusText}</div>

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

      <h2 style="margin-top:1rem">How to calibrate</h2>
      <p class="hint">
        Press <kbd>C</kbd> to begin. Place one block (tag ID {params.blockTag}) at each interior
        corner in order TL → TR → BL → BR, its corner touching the interior corner, then press
        <kbd>Space</kbd> to record. After 4 corners a per-axis linear fit is computed and can be
        downloaded as JSON.
      </p>
      {#if !cvReady}
        <p class="hint">Waiting for OpenCV.js…</p>
      {/if}
    </div>
  </div>
</div>
