<script lang="ts">
  import { onMount, onDestroy } from 'svelte';
  import {
    loadMujocoModule,
    mountModel,
    type ModelFiles,
    type Session,
  } from './lib/mujocoSession';
  import { IKSolver } from './lib/ik';
  import { MujocoRenderer } from './lib/mujocoRender';
  import { Robot } from './lib/robot';
  import { CALIBRATION_PLAN, simRadToServo, type JointCalibration } from './lib/joints';
  import { buildBoardSceneXml, interiorToSim, SQUARE_MM, INSET_MM } from './lib/boardSim';
  import { phaseTarget, DEFAULT_PICK, type PickPhase } from './lib/pick';
  import { loadCv, type Cv } from './lib/cv';
  import { OUT_W, boardTagCentres, BORDERED_IDS, type TagCentres } from './lib/board';
  import {
    createDetector,
    detectMarkers,
    computeHomography,
    tagInteriorPos,
    type Detector,
  } from './lib/homography';

  const XML = 'so101_new_calib.xml';
  const ASSETS = [
    'waveshare_mounting_plate_so101_v2.stl',
    'sts3215_03a_v1.stl',
    'motor_holder_so101_base_v1.stl',
    'wrist_roll_follower_so101_v1.stl',
    'moving_jaw_so101_v1.stl',
    'base_motor_holder_so101_v1.stl',
    'upper_arm_so101_v1.stl',
    'wrist_roll_pitch_so101_v2.stl',
    'under_arm_so101_v1.stl',
    'rotation_pitch_so101_v1.stl',
    'motor_holder_so101_wrist_v1.stl',
    'sts3215_03a_no_horn_v1.stl',
    'base_so101_v2.stl',
  ];
  const ARM_DOFS = [0, 1, 2, 3, 4];
  // Extra 90° added to wrist_roll (dof 4) for the on-screen pose ONLY — the real
  // servo still gets the raw IK-solved angle.
  const WRIST_ROLL_OFFSET = Math.PI / 2;
  const GRASP_SITE = 'graspframe';

  let canvas: HTMLCanvasElement;
  let wrap: HTMLDivElement;

  let status = $state('Loading MuJoCo…');
  let ready = $state(false);
  let errorMsg = $state<string | null>(null);

  let target = $state<[number, number, number]>([0.25, 0, 0.2]);
  let ikOk = $state(true);
  let ikErrMm = $state(0);
  let jointAngles = $state<number[]>([0, 0, 0, 0, 0]);
  let gripperRange = $state<[number, number]>([-0.1745, 1.7453]); // [open, closed]

  // ── sim / real ─────────────────────────────────────────────────────────────
  let mode = $state<'sim' | 'real'>('sim');
  const robot = new Robot();
  let realConnected = $state(false);
  let jointCal = $state<Record<string, JointCalibration> | null>(null);
  let jointCalName = $state<string | null>(null);
  let realError = $state<string | null>(null);
  let lastSent = 0;

  // ── pick + board detection ──────────────────────────────────────────────────
  let pickPhase = $state<PickPhase>('idle');
  let blockTag = $state(101);
  let gripperCmd = $state(-0.1745); // current gripper angle (rad)

  // Stage selector: '' = off (manual sliders), 'auto' = run the full sequence,
  // or a single phase to drive to and hold there.
  type Stage = '' | 'approach' | 'descend' | 'grasp' | 'lift';
  let stage = $state<Stage>('');
  // Last known sim position of the selected block (from detection).
  let pickBlock = $state<[number, number, number] | null>(null);
  // Configurable grasp offsets (m). Depth = descend/grasp height (can be negative).
  let graspDepth = $state(DEFAULT_PICK.graspZ);
  let graspX = $state(DEFAULT_PICK.graspXOffset);
  let graspY = $state(DEFAULT_PICK.graspYOffset);

  // Board perspective correction (from the Test-calibration tab's JSON).
  let boardCorr = $state<{ Sx: number; Bx: number; Sy: number; By: number } | null>(null);
  let boardCorrName = $state<string | null>(null);

  // Latest detected block positions (interior mm, corrected) keyed by tag id.
  let detected = $state<Map<number, [number, number]>>(new Map());
  let detectMsg = $state<string | null>(null);

  // Camera detection plumbing (only runs in real mode once started).
  let cv: Cv | null = null;
  let detector: Detector | null = null;
  let video: HTMLVideoElement;
  let camStream: MediaStream | null = null;
  let grabCtx: CanvasRenderingContext2D | null = null;
  let srcMat: any = null;
  let grayMat: any = null;
  let tagCentresMm: TagCentres = new Map();
  let lastDetect = 0;

  let session: Session | null = null;
  let solver: IKSolver | null = null;
  let renderer: MujocoRenderer | null = null;
  let raf = 0;

  function setBlockMocap(pos: [number, number, number]) {
    if (!session) return;
    const mp = session.data.mocap_pos as Float64Array;
    mp[0] = pos[0];
    mp[1] = pos[1];
    mp[2] = pos[2];
  }

  onMount(async () => {
    try {
      const mj = await loadMujocoModule();
      status = 'Loading SO-101 model…';
      // Fetch the base model, paint the calibration board + a mocap block into
      // it, then mount the augmented scene.
      const base = await (await fetch(`/models/so101/${XML}`)).text();
      const files: ModelFiles = { 'scene.xml': buildBoardSceneXml(base) };
      for (const name of ASSETS) {
        const res = await fetch(`/models/so101/assets/${name}`);
        files[`assets/${name}`] = new Uint8Array(await res.arrayBuffer());
      }
      session = mountModel(mj, files, 'scene.xml');
      solver = new IKSolver(mj, session.model, session.data, GRASP_SITE);
      target = solver.sitePosition();

      // Gripper ctrlrange (joint index 5) for the gripper slider.
      const cr = Array.from(session.model.actuator_ctrlrange as ArrayLike<number>);
      if (cr[10] < cr[11]) gripperRange = [cr[10], cr[11]];

      renderer = new MujocoRenderer(canvas, mj, session.model, session.data);
      renderer.setTarget(target);
      fitCanvas();
      ready = true;
      status = 'Ready';
      loop();
    } catch (e) {
      errorMsg = e instanceof Error ? e.message : String(e);
      status = 'Failed to load';
    }
  });

  function fitCanvas() {
    if (!renderer || !wrap) return;
    const w = wrap.clientWidth;
    const h = Math.round(w * 0.66);
    renderer.resize(w, h);
  }

  function loop() {
    if (!solver || !renderer || !session) return;
    const q = session.data.qpos as Float64Array;

    // A selected stage overrides the manual target with that phase's target.
    let solveTarget = target;
    if (stage !== '' && pickBlock) {
      // Drive to the chosen stage and hold there.
      const s = phaseTarget(pickBlock, stage, {
        ...DEFAULT_PICK,
        graspZ: graspDepth,
        graspXOffset: graspX,
        graspYOffset: graspY,
      });
      solveTarget = s.target;
      gripperCmd = s.gripper;
      pickPhase = stage;
      target = s.target;
    } else if (stage === '') {
      pickPhase = 'idle';
    }

    q[5] = gripperCmd; // gripper is driven directly
    const res = solver.solve(solveTarget, { dofIndices: ARM_DOFS, maxIters: 5 });
    ikOk = res.ok;
    ikErrMm = res.error * 1000;
    jointAngles = res.qpos;

    // Display-only fixups (restored afterwards so they don't affect IK or the
    // streamed command): wrist_roll gets +90°, and the gripper open/close is
    // flipped because the model's gripper geometry renders inverted vs the real arm.
    const solvedRoll = q[4];
    q[4] = solvedRoll + WRIST_ROLL_OFFSET;
    q[5] = gripperRange[0] + gripperRange[1] - gripperCmd;
    session.forward();
    renderer.setTarget(solveTarget);
    renderer.update();
    q[4] = solvedRoll;
    q[5] = gripperCmd;

    maybeDetect();
    maybeStreamToRobot();
    raf = requestAnimationFrame(loop);
  }

  async function maybeStreamToRobot() {
    if (mode !== 'real' || !realConnected || !jointCal) return;
    const now = performance.now();
    if (now - lastSent < 66) return; // ~15 Hz
    lastSent = now;
    const targets = new Map<number, number>();
    // Arm joints from IK, gripper from the pick/gripper command.
    CALIBRATION_PLAN.forEach((def, i) => {
      const cal = jointCal![def.joint];
      if (!cal) return;
      const angle = i < 5 ? jointAngles[i] : gripperCmd;
      const servo = simRadToServo(angle, cal);
      if (servo != null) targets.set(def.servoId, servo);
    });
    if (targets.size) {
      try {
        await robot.syncWritePositions(targets);
      } catch (e) {
        realError = e instanceof Error ? e.message : String(e);
      }
    }
  }

  // ── Camera detection (real mode) ────────────────────────────────────────────
  async function startCamera() {
    if (camStream) return; // already acquired
    try {
      if (!cv) {
        cv = await loadCv();
        detector = createDetector(cv);
        tagCentresMm = boardTagCentres(SQUARE_MM, 16, 2, 10, 8);
      }
      camStream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: 'environment' },
      });
      video.srcObject = camStream;
      await video.play();
      const w = video.videoWidth || 640;
      const h = video.videoHeight || 480;
      const grab = document.createElement('canvas');
      grab.width = w;
      grab.height = h;
      grabCtx = grab.getContext('2d', { willReadFrequently: true });
      srcMat = new cv.Mat(h, w, cv.CV_8UC4);
      grayMat = new cv.Mat();
    } catch (e) {
      detectMsg = 'Camera error: ' + (e instanceof Error ? e.message : String(e));
    }
  }

  function stopCamera() {
    camStream?.getTracks().forEach((t) => t.stop());
    camStream = null;
    try {
      srcMat?.delete();
      grayMat?.delete();
    } catch {
      /* ignore */
    }
    srcMat = null;
    grayMat = null;
    grabCtx = null;
  }

  function maybeDetect() {
    if (mode !== 'real' || !cv || !detector || !grabCtx || !srcMat || !session) return;
    const now = performance.now();
    if (now - lastDetect < 100) return; // ~10 Hz
    lastDetect = now;

    const { width, height } = grabCtx.canvas;
    grabCtx.drawImage(video, 0, 0, width, height);
    srcMat.data.set(grabCtx.getImageData(0, 0, width, height).data);
    cv.cvtColor(srcMat, grayMat, cv.COLOR_RGBA2GRAY);
    const corners = detectMarkers(cv, detector, grayMat);
    const { H } = computeHomography(cv, corners, tagCentresMm, SQUARE_MM);
    const m = new Map<number, [number, number]>();
    if (H) {
      const scale = OUT_W / SQUARE_MM;
      for (const [id, c] of corners) {
        if (tagCentresMm.has(id) || !BORDERED_IDS.has(id)) continue;
        const [x, y] = tagInteriorPos(cv, c, H, scale, INSET_MM);
        // Apply the board perspective correction when loaded.
        const cx = boardCorr ? boardCorr.Sx * x + boardCorr.Bx : x;
        const cy = boardCorr ? boardCorr.Sy * y + boardCorr.By : y;
        m.set(id, [cx, cy]);
      }
      H.delete();
    }
    detected = m;
    // Track the selected block live in the sim.
    const d = m.get(blockTag);
    if (d) {
      pickBlock = interiorToSim(d[0], d[1]);
      setBlockMocap(pickBlock);
    }
  }

  function onStageChange(next: Stage) {
    detectMsg = null;
    if (next === '') {
      stage = '';
      pickPhase = 'idle';
      return;
    }
    // Need a block position to target.
    const d = detected.get(blockTag);
    if (d) pickBlock = interiorToSim(d[0], d[1]);
    if (!pickBlock) {
      detectMsg = `Block ${blockTag} not detected on the board.`;
      stage = '';
      return;
    }
    stage = next;
  }

  async function connectRobot() {
    realError = null;
    try {
      await robot.connect();
      realConnected = true;
      await robot.setTorque(
        CALIBRATION_PLAN.map((j) => j.servoId),
        true,
      );
    } catch (e) {
      realError = e instanceof Error ? e.message : String(e);
    }
  }

  async function disconnectRobot() {
    try {
      await robot.setTorque(CALIBRATION_PLAN.map((j) => j.servoId), false);
    } catch {
      /* ignore */
    }
    await robot.disconnect().catch(() => {});
    realConnected = false;
  }

  async function loadJointCal(e: Event) {
    const input = e.target as HTMLInputElement;
    const file = input.files?.[0];
    if (!file) return;
    try {
      jointCal = JSON.parse(await file.text());
      jointCalName = file.name;
    } catch (err) {
      realError = 'Bad calibration file: ' + (err instanceof Error ? err.message : String(err));
    }
    input.value = '';
  }

  async function loadBoardCal(e: Event) {
    const input = e.target as HTMLInputElement;
    const file = input.files?.[0];
    if (!file) return;
    try {
      const d = JSON.parse(await file.text());
      const { Sx, Bx, Sy, By } = d;
      if ([Sx, Bx, Sy, By].some((v) => typeof v !== 'number')) throw new Error('missing Sx/Bx/Sy/By');
      boardCorr = { Sx, Bx, Sy, By };
      boardCorrName = file.name;
    } catch (err) {
      detectMsg = 'Bad board calibration: ' + (err instanceof Error ? err.message : String(err));
    }
    input.value = '';
  }

  const onResize = () => fitCanvas();
  $effect(() => {
    window.addEventListener('resize', onResize);
    return () => window.removeEventListener('resize', onResize);
  });

  // Start/stop the camera as real mode is entered/left.
  $effect(() => {
    if (mode === 'real') startCamera();
    else {
      stopCamera();
      stage = '';
      pickPhase = 'idle';
    }
  });

  onDestroy(() => {
    cancelAnimationFrame(raf);
    stopCamera();
    if (realConnected) disconnectRobot();
    solver?.dispose();
    renderer?.dispose();
    session?.dispose();
  });
</script>

<div class="sim">
  <div class="viewer" bind:this={wrap}>
    <canvas bind:this={canvas}></canvas>
    {#if !ready}<div class="overlay">{status}</div>{/if}
    {#if errorMsg}<div class="overlay err">{errorMsg}</div>{/if}
  </div>

  <div class="panel">
    <div class="modeswitch">
      <button class:active={mode === 'sim'} onclick={() => (mode = 'sim')}>Sim only</button>
      <button class:active={mode === 'real'} onclick={() => (mode = 'real')}>Drive real arm</button>
    </div>

    <h2>End-effector target</h2>
    <div class="sliders">
      {#each ['x', 'y', 'z'] as axis, i (axis)}
        <label>{axis}</label>
        <input
          type="range"
          min={i === 1 ? -0.35 : i === 2 ? 0 : -0.1}
          max={i === 1 ? 0.35 : 0.45}
          step="0.005"
          bind:value={target[i]}
        />
        <span class="val">{target[i].toFixed(3)}</span>
      {/each}
      <label>grip</label>
      <input
        type="range"
        min={gripperRange[0]}
        max={gripperRange[1]}
        step="0.01"
        bind:value={gripperCmd}
      />
      <span class="val">
        {Math.round(((gripperCmd - gripperRange[0]) / (gripperRange[1] - gripperRange[0])) * 100)}%
      </span>
    </div>

    <div class="ikstatus {ikOk ? 'ok' : 'bad'}">
      {ikOk ? 'reached' : 'unreached'} · error {ikErrMm.toFixed(1)} mm
    </div>

    <table class="angles">
      <thead><tr><th>joint</th><th>rad</th><th>deg</th></tr></thead>
      <tbody>
        {#each ['pan', 'lift', 'elbow', 'wrist_flex', 'wrist_roll'] as name, i (name)}
          {@const a = jointAngles[i]}
          <tr>
            <td>{name}</td>
            <td>{a != null ? a.toFixed(3) : '—'}</td>
            <td>{a != null ? ((a * 180) / Math.PI).toFixed(1) : '—'}</td>
          </tr>
        {/each}
      </tbody>
    </table>

    {#if mode === 'real'}
      <div class="realbox">
        <h2>Real arm</h2>
        <p class="hint">
          Solved angles stream to the Feetech servos at ~15 Hz. Needs a joint calibration
          (from the Joint calibration tab) to map sim radians → servo ticks.
        </p>
        <div class="controls">
          {#if !realConnected}
            <button class="primary" onclick={connectRobot}>Connect servos…</button>
          {:else}
            <button onclick={disconnectRobot}>Disconnect</button>
          {/if}
          <label class="file-btn">
            {jointCalName ?? 'Load joint_calibration.json…'}
            <input type="file" accept="application/json,.json" onchange={loadJointCal} />
          </label>
        </div>
        {#if realConnected && !jointCal}
          <div class="status warn">Connected — load a joint calibration to start driving.</div>
        {:else if realConnected && jointCal}
          <div class="status ok">Driving arm live from the simulator.</div>
        {/if}
        {#if realError}<div class="status bad">Error: {realError}</div>{/if}
      </div>

      <div class="realbox">
        <h2>Pick a block</h2>
        <p class="hint">
          The overhead camera detects the block's ArUco tag → board position, which is copied into
          the sim; the arm then approaches, descends, grasps and lifts. Needs the board perspective
          calibration (from the Test-calibration tab) for accurate placement.
        </p>
        <div class="controls">
          <label class="file-btn">
            {boardCorrName ?? 'Load board calibration…'}
            <input type="file" accept="application/json,.json" onchange={loadBoardCal} />
          </label>
        </div>
        <div class="pickrow">
          <label for="blocktag">Block tag ID</label>
          <input id="blocktag" type="number" bind:value={blockTag} />
          <label for="stage">Stage</label>
          <select
            id="stage"
            value={stage}
            disabled={!realConnected || !jointCal}
            onchange={(e) => onStageChange(e.currentTarget.value as Stage)}
          >
            <option value="">— off (manual) —</option>
            <option value="approach">1 · Approach (hover)</option>
            <option value="descend">2 · Descend</option>
            <option value="grasp">3 · Grasp (close)</option>
            <option value="lift">4 · Lift</option>
          </select>
        </div>
        <div class="pickrow">
          <label for="depth">Grasp depth (m)</label>
          <input id="depth" type="number" step="0.005" bind:value={graspDepth} />
        </div>
        <div class="pickrow">
          <label for="gx">Grasp X offset (m)</label>
          <input id="gx" type="number" step="0.005" bind:value={graspX} />
        </div>
        <div class="pickrow">
          <label for="gy">Grasp Y offset (m)</label>
          <input id="gy" type="number" step="0.005" bind:value={graspY} />
        </div>
        <div class="status {detected.has(blockTag) ? 'ok' : 'warn'}">
          {#if stage !== ''}
            holding at <strong>{stage}</strong>
          {:else if detected.has(blockTag)}
            block {blockTag} detected {boardCorr ? '(corrected)' : '(uncorrected — load board calibration)'}
          {:else}
            block {blockTag} not detected — hold it on the board in view of the camera
          {/if}
        </div>
        {#if detectMsg}<div class="status bad">{detectMsg}</div>{/if}
      </div>
    {/if}
  </div>
</div>

<!-- Hidden camera source for block detection in real mode. -->
<video bind:this={video} playsinline muted style="display:none"></video>

<style>
  .sim {
    display: grid;
    grid-template-columns: 1fr 300px;
    gap: 1.25rem;
    align-items: start;
  }
  @media (max-width: 980px) {
    .sim {
      grid-template-columns: 1fr;
    }
  }
  .viewer {
    position: relative;
    width: 100%;
  }
  .viewer canvas {
    width: 100%;
    display: block;
    border: 1px solid #2a2f38;
    border-radius: 8px;
  }
  .overlay {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    color: #8b93a1;
    font-size: 0.95rem;
  }
  .overlay.err {
    color: #f87171;
    padding: 1rem;
    text-align: center;
  }
  h2 {
    font-size: 0.95rem;
    margin: 1rem 0 0.5rem;
  }
  .modeswitch {
    display: flex;
    gap: 0.25rem;
  }
  .modeswitch button {
    flex: 1;
  }
  .modeswitch button.active {
    background: #1f6feb;
    border-color: #1f6feb;
  }
  .sliders {
    display: grid;
    grid-template-columns: auto 1fr auto;
    gap: 0.4rem 0.6rem;
    align-items: center;
  }
  .sliders label {
    font-family: ui-monospace, monospace;
    color: #8b93a1;
  }
  .sliders .val {
    font-family: ui-monospace, monospace;
    font-size: 0.85rem;
    width: 3.5rem;
    text-align: right;
  }
  .ikstatus {
    margin-top: 0.6rem;
    font-family: ui-monospace, monospace;
    font-size: 0.85rem;
  }
  .ikstatus.ok {
    color: #4ade80;
  }
  .ikstatus.bad {
    color: #fbbf24;
  }
  table.angles {
    width: 100%;
    margin-top: 0.75rem;
    border-collapse: collapse;
    font-family: ui-monospace, monospace;
    font-size: 0.82rem;
  }
  table.angles th,
  table.angles td {
    text-align: right;
    padding: 0.25rem 0.5rem;
    border-bottom: 1px solid #23282f;
  }
  table.angles th:first-child,
  table.angles td:first-child {
    text-align: left;
    color: #8b93a1;
  }
  .file-btn {
    display: inline-flex;
    align-items: center;
    background: #262b33;
    border: 1px solid #3a414c;
    border-radius: 6px;
    padding: 0.45rem 0.9rem;
    font-size: 0.85rem;
    cursor: pointer;
  }
  .file-btn input {
    display: none;
  }
  .pickrow {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    margin: 0.6rem 0;
  }
  .pickrow label {
    font-size: 0.85rem;
    color: #8b93a1;
  }
  .pickrow input[type='number'] {
    width: 4.5rem;
    background: #1c2027;
    color: #e6e6e6;
    border: 1px solid #3a414c;
    border-radius: 4px;
    padding: 0.25rem 0.4rem;
  }
  .pickrow select {
    flex: 1;
    background: #1c2027;
    color: #e6e6e6;
    border: 1px solid #3a414c;
    border-radius: 4px;
    padding: 0.28rem 0.4rem;
  }
</style>
