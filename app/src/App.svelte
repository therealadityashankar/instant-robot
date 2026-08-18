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
    type Detector,
  } from './lib/homography';
  import { drawRawPanel, drawRectifiedPanel } from './lib/render';
  import { DEFAULT_PARAMS, type BoardParams, type CalibState } from './lib/types';
  import { saveIntrinsics, loadIntrinsics, loadRemoteConfig, saveRemoteConfig } from './lib/storage';
  import { solvePnpTvec, meanStd } from './lib/pose';
  import { boardPoseFromTags, blockBoardXY } from './lib/detect3d';
  import { BORDERED_IDS } from './lib/board';
  import {
    createCharucoBoard,
    createCharucoDetector,
    detectView,
    calibrateIntrinsics,
    type CharucoView,
    type Intrinsics,
  } from './lib/charuco';
  import JointCalibration from './JointCalibration.svelte';
  import Simulator from './Simulator.svelte';
  import { armLink } from './lib/armLink.svelte';
  import { settings } from './lib/settings.svelte';
  import SettingsModal from './SettingsModal.svelte';
  import BaseCalibration from './BaseCalibration.svelte';

  let params = $state<BoardParams>({ ...DEFAULT_PARAMS });

  // ── Remote-connect chooser (local USB vs. WebRTC to a Pi) ─────────────────
  // One field, "room@token" — split back into the pair connectRemote below.
  const savedRemote = loadRemoteConfig();
  let remoteCombined = $state(savedRemote ? `${savedRemote.room}@${savedRemote.token}` : '');
  let connectBtnEl: HTMLButtonElement | null = $state(null);
  let connectModalEl: HTMLDivElement | null = $state(null);
  const remoteParsed = $derived.by(() => {
    const at = remoteCombined.indexOf('@');
    if (at <= 0 || at === remoteCombined.length - 1) return null;
    return { room: remoteCombined.slice(0, at).trim(), token: remoteCombined.slice(at + 1).trim() };
  });
  function connectRemote() {
    if (!remoteParsed) return;
    saveRemoteConfig(remoteParsed);
    armLink.pickRemote({ signalOrigin: remoteWsOrigin(), room: remoteParsed.room, token: remoteParsed.token });
  }
  function remoteWsOrigin(): string {
    return `${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}`;
  }
  function closeConnectChooser() {
    armLink.chooserOpen = false;
    connectBtnEl?.focus(); // dialog closed -> focus goes back to what opened it
  }
  // Move focus into the dialog the moment it opens, as a screen reader expects.
  $effect(() => {
    if (armLink.chooserOpen) connectModalEl?.querySelector<HTMLElement>('[data-autofocus]')?.focus();
  });

  let cvReady = $state(false);
  let errorMsg = $state<string | null>(null);
  let running = $state(false);
  let statusText = $state('Loading OpenCV.js…');
  let statusClass = $state<'ok' | 'warn' | 'bad'>('warn');

  // ── Calibration modal: camera intrinsics / detect / joint calibration ─────
  let calibrateOpen = $state(false);
  let calStep = $state<'intrinsics' | 'test' | 'joints' | 'base'>('intrinsics');

  // ── Camera intrinsics (ChArUco) ───────────────────────────────────────────
  let intrinsics = $state<Intrinsics | null>(loadIntrinsics());
  let charucoBoard: any = null;
  let charucoDetector: any = null;
  let charucoViews: CharucoView[] = [];
  let charucoViewCount = $state(0);
  let charucoLiveCorners = $state(0);
  let intrinsicsMsg = $state<string | null>(null);
  interface Readout {
    id: number;
    x: number; // board-mm X (solvePnP)
    y: number; // board-mm Y (solvePnP)
  }
  let readout = $state<Readout[]>([]);

  // ── Detection-noise logger ────────────────────────────────────────────────
  const MEASURE_N = 200;
  let measuring = $state(false);
  let measureLeft = $state(0);
  let measureResult = $state<string | null>(null);
  let markerMm = $state(15); // block ArUco marker size (black square, not white border)
  const samp = {
    bx: [] as number[], // board-mm X (the actual output)
    by: [] as number[], // board-mm Y
    cz: [] as number[], // camera-frame depth (informational)
  };

  function startMeasure() {
    if (!intrinsics) return;
    samp.bx = [];
    samp.by = [];
    samp.cz = [];
    measureLeft = MEASURE_N;
    measureResult = null;
    measuring = true;
  }

  /** Each frame while measuring: record the selected block's board-mm position. */
  function recordSample(cornersDict: Map<number, any>) {
    if (!intrinsics) return;
    const corners = cornersDict.get(params.blockTag);
    if (!corners || !BORDERED_IDS.has(params.blockTag)) return;
    const tagCentresMm = boardTagCentres(
      params.squareMm, params.tagMm, params.gapMm, params.nOuter, params.nInner,
    );
    const pose = boardPoseFromTags(cv, cornersDict, tagCentresMm, params.tagMm, intrinsics, camW, camH);
    if (!pose) return;
    const xy = blockBoardXY(cv, corners, markerMm, intrinsics, camW, camH, pose);
    if (!xy) return;
    samp.bx.push(xy.x);
    samp.by.push(xy.y);
    const t = solvePnpTvec(cv, corners, markerMm, camW, camH, intrinsics);
    if (t) samp.cz.push(t[2]);

    measureLeft -= 1;
    if (measureLeft <= 0) finishMeasure();
  }

  function finishMeasure() {
    measuring = false;
    const f = (v: number) => (Number.isFinite(v) ? v.toFixed(2) : '—');
    const f3 = (v: number) => (Number.isFinite(v) ? v.toFixed(3) : '—');
    const bx = meanStd(samp.bx);
    const by = meanStd(samp.by);
    const cz = meanStd(samp.cz);
    measureResult =
      `Samples: ${samp.bx.length}   intrinsics RMS ${intrinsics?.rms.toFixed(2) ?? '—'}px, marker ${markerMm}mm\n\n` +
      `solvePnP board position (board mm) — what actually drives picking\n` +
      `  X: mean ${f(bx.mean)}  std ±${f3(bx.std)} mm\n` +
      `  Y: mean ${f(by.mean)}  std ±${f3(by.std)} mm\n\n` +
      `Camera depth (informational): mean ${f(cz.mean)}mm  std ±${f3(cz.std)}mm\n` +
      `(depth mean ≈ camera-to-board distance; std is the noisy axis we discard)`;
  }

  // ── ChArUco intrinsic-calibration frame processing ────────────────────────
  function processCharucoFrame(grabCtx: CanvasRenderingContext2D) {
    if (!charucoBoard) {
      charucoBoard = createCharucoBoard(cv);
      charucoDetector = createCharucoDetector(cv, charucoBoard);
    }
    const { width, height } = grabCtx.canvas;
    grabCtx.drawImage(video, 0, 0, width, height);
    camSrc.data.set(grabCtx.getImageData(0, 0, width, height).data);
    cv.cvtColor(camSrc, camGray, cv.COLOR_RGBA2GRAY);

    const ctx = rawCanvas.getContext('2d')!;
    ctx.drawImage(video, 0, 0, rawCanvas.width, rawCanvas.height);

    // Detect + draw the charuco corners as a live preview. Also pull out the
    // marker detections so we can tell "no markers found" from "markers found
    // but corners didn't interpolate".
    const corners = new cv.Mat();
    const ids = new cv.Mat();
    const markerCorners = new cv.MatVector();
    const markerIds = new cv.Mat();
    let nMarkers = 0;
    try {
      charucoDetector.detectBoard(camGray, corners, ids, markerCorners, markerIds);
      // OpenCV.js returns 1×N Mats — count with total(), not rows.
      charucoLiveCorners = ids.total?.() || 0;
      nMarkers = markerIds.total?.() || 0;
      // Draw detected marker outlines (amber) …
      ctx.strokeStyle = 'rgb(217,119,6)';
      ctx.lineWidth = 2;
      for (let m = 0; m < markerCorners.size(); m++) {
        const mc = markerCorners.get(m);
        const md = mc.data32F as Float32Array;
        ctx.beginPath();
        ctx.moveTo(md[0], md[1]);
        for (let k = 1; k < 4; k++) ctx.lineTo(md[k * 2], md[k * 2 + 1]);
        ctx.closePath();
        ctx.stroke();
        mc.delete();
      }
      // … and charuco corners (green).
      const d = corners.data32F as Float32Array;
      ctx.fillStyle = 'rgb(34,197,94)';
      for (let i = 0; i < charucoLiveCorners; i++) {
        ctx.beginPath();
        ctx.arc(d[i * 2], d[i * 2 + 1], 4, 0, Math.PI * 2);
        ctx.fill();
      }
    } catch (e) {
      charucoLiveCorners = 0;
      intrinsicsMsg = 'detectBoard error: ' + (e instanceof Error ? e.message : String(e));
    } finally {
      corners.delete();
      ids.delete();
      markerCorners.delete();
      markerIds.delete();
    }
    statusText = `Markers: ${nMarkers}   ChArUco corners: ${charucoLiveCorners}   (need the board in view)`;
    statusClass = charucoLiveCorners >= 6 ? 'ok' : nMarkers > 0 ? 'warn' : 'bad';
  }

  function captureCharucoView() {
    if (!charucoDetector) return;
    const v = detectView(cv, charucoDetector, charucoBoard, camGray);
    if (!v) {
      intrinsicsMsg = 'No board detected — get the full ChArUco board in frame.';
      return;
    }
    charucoViews.push(v);
    charucoViewCount = charucoViews.length;
    intrinsicsMsg = `Captured view ${charucoViewCount} (${v.n} corners). Tilt/move the board and capture again.`;
  }

  function runIntrinsicsCalibration() {
    if (charucoViews.length < 5) {
      intrinsicsMsg = `Need at least 5 views from different angles (have ${charucoViews.length}).`;
      return;
    }
    try {
      const intr = calibrateIntrinsics(cv, charucoViews, camW, camH);
      intrinsics = intr;
      saveIntrinsics(intr);
      const fx = intr.cameraMatrix[0];
      const fy = intr.cameraMatrix[4];
      intrinsicsMsg =
        `Calibrated from ${charucoViews.length} views — RMS reprojection ${intr.rms.toFixed(3)} px\n` +
        `fx=${fx.toFixed(1)}  fy=${fy.toFixed(1)}  saved. solvePnP now uses these intrinsics.`;
    } catch (e) {
      intrinsicsMsg = 'Calibration failed: ' + (e instanceof Error ? e.message : String(e));
    }
  }

  function resetIntrinsicsViews() {
    charucoViews = [];
    charucoViewCount = 0;
    intrinsicsMsg = null;
  }

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
    // Homography is kept ONLY for the bird's-eye visualization (no calibration).
    drawRawPanel(cv, rawCtx, video, cornersDict, tagCentresMm, H, params.squareMm);
    drawRectifiedPanel(
      cv, rectCtx, src, H, params.squareMm, params.tagMm, params.gapMm,
      cornersDict, tagCentresMm, innerTagsStart(), calib, null,
    );

    updateStatus(cornersDict, tagCentresMm, inliers, !!H);

    if (calStep === 'test') computeReadout(cornersDict, tagCentresMm);
    else if (readout.length) readout = [];

    if (measuring) recordSample(cornersDict);
  }

  /** solvePnP board-frame positions for every detected object tag (board mm). */
  function computeReadout(cornersDict: Map<number, any>, tagCentresMm: TagCentres) {
    if (!intrinsics) {
      if (readout.length) readout = [];
      return;
    }
    const pose = boardPoseFromTags(cv, cornersDict, tagCentresMm, params.tagMm, intrinsics, camW, camH);
    if (!pose) {
      if (readout.length) readout = [];
      return;
    }
    const rows: Readout[] = [];
    for (const [id, corners] of cornersDict) {
      if (tagCentresMm.has(id)) continue; // border tags aren't objects
      if (id < 100 || id >= 200) continue;
      const xy = blockBoardXY(cv, corners, markerMm, intrinsics, camW, camH, pose);
      if (xy) rows.push({ id, x: xy.x, y: xy.y });
    }
    rows.sort((a, b) => a.id - b.id);
    readout = rows;
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

    if (!intrinsics) {
      statusText = 'Calibrate the camera (ChArUco) first — needed for solvePnP positions.';
      statusClass = 'bad';
    } else {
      statusText = `Border tags: ${nBorder}/${tagCentresMm.size}  Objects: ${nObj}  ${hasH ? '[board OK]' : '[need ≥4 border tags]'}`;
      statusClass = hasH ? 'ok' : 'warn';
    }
    void inliers;
  }

  let camSrc: any = null;
  let camGray: any = null;
  let camW = 640;
  let camH = 480;

  // The camera runs only while the calibration modal is open (on a camera step),
  // so it's released for the Simulator's real-mode detection when the modal closes.
  async function startCamera() {
    if (running) return;
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
      camW = w;
      camH = h;
      const grab = document.createElement('canvas');
      grab.width = w;
      grab.height = h;
      const grabCtx = grab.getContext('2d', { willReadFrequently: true })!;
      camSrc = new cv.Mat(h, w, cv.CV_8UC4);
      camGray = new cv.Mat();
      running = true;

      const loop = () => {
        if (!running) return;
        // Skip when the camera panels aren't mounted (joints step / closed).
        if (calStep === 'joints' || !rawCanvas || !rectCanvas) {
          rafId = requestAnimationFrame(loop);
          return;
        }
        if (rawCanvas.width !== w) {
          rawCanvas.width = w;
          rawCanvas.height = h;
          rectCanvas.width = OUT_W;
          rectCanvas.height = OUT_H;
        }
        try {
          if (calStep === 'intrinsics') {
            processCharucoFrame(grabCtx);
          } else {
            const tagCentresMm = boardTagCentres(
              params.squareMm, params.tagMm, params.gapMm, params.nOuter, params.nInner,
            );
            processFrame(grabCtx, camSrc, camGray, tagCentresMm);
          }
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

  function stopCamera() {
    running = false;
    cancelAnimationFrame(rafId);
    stream?.getTracks().forEach((t) => t.stop());
    stream = null;
    try {
      camSrc?.delete();
      camGray?.delete();
    } catch {
      /* ignore */
    }
    camSrc = null;
    camGray = null;
  }

  onMount(() => {
    return () => {
      stopCamera();
      if (lastH) lastH.delete();
    };
  });

  // Start the camera when the calibration modal opens; release it when it closes.
  $effect(() => {
    if (calibrateOpen) startCamera();
    else stopCamera();
  });
</script>

<div class="app">
  {#if armLink.error}
    <div class="toperror">Servo bus: {armLink.error}</div>
  {/if}

  <!-- Hidden source element; frames are read from it into OpenCV each tick. -->
  <video bind:this={video} playsinline muted style="display:none"></video>

  <Simulator onOpenCalibrate={() => (calibrateOpen = true)} />
</div>

<SettingsModal />

{#if armLink.chooserOpen}
  <div
    class="modal-backdrop connectbackdrop"
    role="button"
    tabindex="-1"
    onclick={closeConnectChooser}
    onkeydown={(e) => e.key === 'Escape' && closeConnectChooser()}
  >
    <div
      bind:this={connectModalEl}
      class="modal connectmodal"
      role="dialog"
      aria-modal="true"
      aria-labelledby="connect-modal-title"
      tabindex="-1"
      onclick={(e) => e.stopPropagation()}
    >
      <div class="modal-head">
        <h2 id="connect-modal-title">Connect robot</h2>
        <button class="close" aria-label="Close" onclick={closeConnectChooser}>✕</button>
      </div>
      <div class="modal-body connectbody">
        <button data-autofocus class="connectsq" onclick={() => armLink.pickLocal()}>
          <svg
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            stroke-width="2"
            stroke-linecap="round"
            stroke-linejoin="round"
          >
            <path d="M12 22v-5" /><path d="M9 8V2" /><path d="M15 8V2" />
            <path d="M18 8v5a4 4 0 0 1-4 4h-4a4 4 0 0 1-4-4V8Z" />
          </svg>
          Local (USB)
        </button>
        <div class="connector"><span>or</span></div>
        <div class="connectremote">
          <label for="connect-combined">Room connection code</label>
          <input id="connect-combined" type="password" bind:value={remoteCombined} />
          <button class="connectbtn" disabled={!remoteParsed} onclick={connectRemote}> Connect </button>
        </div>
      </div>
    </div>
  </div>
{/if}

{#if calibrateOpen}
  <div
    class="modal-backdrop"
    role="button"
    tabindex="-1"
    onclick={() => (calibrateOpen = false)}
    onkeydown={(e) => e.key === 'Escape' && (calibrateOpen = false)}
  >
    <div class="modal" role="dialog" tabindex="-1" onclick={(e) => e.stopPropagation()}>
      <div class="modal-head">
        <div class="calsteps">
          <button class:active={calStep === 'intrinsics'} onclick={() => (calStep = 'intrinsics')}>
            1 · Camera (ChArUco)
          </button>
          <button class:active={calStep === 'test'} onclick={() => (calStep = 'test')}>
            2 · Detect
          </button>
          <button class:active={calStep === 'joints'} onclick={() => (calStep = 'joints')}>
            3 · Joints
          </button>
          <button class:active={calStep === 'base'} onclick={() => (calStep = 'base')}>
            4 · Base
          </button>
        </div>
        <button class="close" onclick={() => (calibrateOpen = false)}>✕</button>
      </div>

      <div class="modal-body">
        {#if calStep === 'joints'}
          <JointCalibration />
        {:else if calStep === 'base'}
          <BaseCalibration />
        {:else}
          <div class="layout">
    <div>
      <div class="panels">
        <canvas bind:this={rawCanvas} width="640" height="480"></canvas>
        <canvas bind:this={rectCanvas} width={OUT_W} height={OUT_H}></canvas>
      </div>

      <div class="status {statusClass}">{statusText}</div>

      {#if calStep === 'intrinsics'}
        <div class="controls">
          <button class="primary" onclick={captureCharucoView} disabled={charucoLiveCorners < 6}>
            Capture view ({charucoViewCount})
          </button>
          <button onclick={runIntrinsicsCalibration} disabled={charucoViewCount < 5}>
            Calibrate camera
          </button>
          <button onclick={resetIntrinsicsViews} disabled={charucoViewCount === 0}>Reset</button>
        </div>
        {#if intrinsicsMsg}
          <pre class="report">{intrinsicsMsg}</pre>
        {:else if intrinsics}
          <div class="status ok">
            Using saved intrinsics (RMS {intrinsics.rms.toFixed(3)} px). Recalibrate to replace.
          </div>
        {/if}
      {:else}
        <!-- Detect step: solvePnP board-frame positions -->
        {#if !intrinsics}
          <div class="status bad">Calibrate the camera (ChArUco step) first.</div>
        {/if}
        <div class="controls">
          <button onclick={startMeasure} disabled={!intrinsics || measuring}>
            {measuring ? `Measuring… ${measureLeft}` : `Measure noise (block ${params.blockTag})`}
          </button>
        </div>

        {#if measureResult}
          <pre class="report">{measureResult}</pre>
        {/if}

        <table class="readout">
          <thead>
            <tr><th>Tag</th><th>Board X (mm)</th><th>Board Y (mm)</th></tr>
          </thead>
          <tbody>
            {#if readout.length === 0}
              <tr><td colspan="3" class="empty">No object tags detected…</td></tr>
            {:else}
              {#each readout as r (r.id)}
                <tr>
                  <td>{r.id}</td>
                  <td class="cal">{r.x.toFixed(1)}</td>
                  <td class="cal">{r.y.toFixed(1)}</td>
                </tr>
              {/each}
            {/if}
          </tbody>
        </table>
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

      {#if calStep === 'intrinsics'}
        <h2 style="margin-top:1rem">Camera calibration (ChArUco)</h2>
        <p class="hint">
          Print <code>printables/charuco_board.pdf</code> (7×5, DICT_5X5_100). Hold the whole board in
          view and <strong>Capture view</strong> from several angles/distances (≥5, more is better —
          tilt it each time). Then <strong>Calibrate camera</strong> runs OpenCV's
          <code>calibrateCameraExtended</code> and saves the intrinsics. Lower RMS (px) is better.
          This is required — block positions are computed via solvePnP.
        </p>
      {:else}
        <h2 style="margin-top:1rem">Detect</h2>
        <p class="hint">
          Move a tagged block (bordered ID 100–199) around the board. Each row shows its position in
          board millimetres, computed from <strong>solvePnP</strong> against the ChArUco intrinsics
          (block height / camera tilt handled natively — no perspective calibration needed).
        </p>
      {/if}
      {#if !cvReady}
        <p class="hint">Waiting for OpenCV.js…</p>
      {/if}
    </div>
  </div>
        {/if}
      </div>
    </div>
  </div>
{/if}
