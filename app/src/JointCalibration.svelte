<script lang="ts">
  import { onDestroy } from 'svelte';
  import { Robot } from './lib/robot';
  import {
    CALIBRATION_PLAN,
    VERIFICATION_POSES,
    JOINT_NAMES,
    fitLinear,
    simRadToServo,
    type JointCalibration,
  } from './lib/joints';
  import { saveJointCalibration, loadJointCalibration } from './lib/storage';

  const robot = new Robot();

  let connected = $state(false);
  let error = $state<string | null>(null);
  let phase = $state<'idle' | 'calibrating' | 'done'>('idle');

  // A work queue of plan indices to calibrate (all six for a full run, or a
  // single index when redoing one joint). `qi` is the position within it.
  let queue = $state<number[]>([]);
  let qi = $state(0);
  let pi = $state(0); // point index within the current joint
  let liveReal = $state<number | null>(null);

  // Captured samples for the joint currently being calibrated.
  let realVals: number[] = [];
  let simVals: number[] = [];

  // Finished per-joint calibrations, keyed by joint name. Defaults to the last
  // saved calibration (a supplied file overrides it).
  const savedJoints = loadJointCalibration();
  let calibration = $state<Record<string, JointCalibration>>(savedJoints ?? {});
  let loadedName = $state<string | null>(savedJoints ? 'saved calibration' : null);

  let polling = false;
  let pollTimer: ReturnType<typeof setTimeout> | null = null;

  const jointDef = $derived(CALIBRATION_PLAN[queue[qi]] ?? null);
  const point = $derived(jointDef?.points[pi] ?? null);

  function deg(rad: number) {
    return (rad * 180) / Math.PI;
  }

  async function pollLoop() {
    if (!polling) return;
    const def = CALIBRATION_PLAN[queue[qi]];
    if (connected && def && phase === 'calibrating') {
      try {
        liveReal = await robot.readPosition(def.servoId);
      } catch {
        liveReal = null;
      }
    }
    pollTimer = setTimeout(pollLoop, 150);
  }

  async function connect() {
    error = null;
    try {
      await robot.connect();
      connected = true;
      polling = true;
      pollLoop();
      // If a calibration was loaded up front, keep it and land on the per-joint
      // screen so specific joints can be corrected; otherwise sweep all six.
      if (Object.keys(calibration).length > 0) phase = 'done';
      else startQueue(CALIBRATION_PLAN.map((_, i) => i));
    } catch (e) {
      error = e instanceof Error ? e.message : String(e);
    }
  }

  /** Load an existing joint_calibration.json to start from / correct. */
  async function loadCalibrationFile(e: Event) {
    const input = e.target as HTMLInputElement;
    const file = input.files?.[0];
    if (!file) return;
    try {
      const parsed = JSON.parse(await file.text());
      if (typeof parsed !== 'object' || parsed === null) throw new Error('not an object');
      calibration = parsed as Record<string, JointCalibration>;
      loadedName = file.name;
      saveJointCalibration(calibration); // supplied file becomes the new default
      // Loaded outside an active sweep → show the per-joint table to correct from.
      if (connected && phase === 'calibrating' && queue.length > 1) {
        // don't interrupt an in-progress full sweep
      } else if (connected) {
        phase = 'done';
      }
    } catch (err) {
      error = 'Bad calibration file: ' + (err instanceof Error ? err.message : String(err));
    }
    input.value = '';
  }

  /** Begin calibrating the given plan indices, in order. */
  function startQueue(indices: number[]) {
    if (testing) disableTest(); // torque off so the arm can be posed by hand
    queue = indices;
    qi = 0;
    pi = 0;
    realVals = [];
    simVals = [];
    phase = 'calibrating';
  }

  /** Re-calibrate a single joint, leaving all other joints' fits intact. */
  function recalibrateJoint(planIndex: number) {
    startQueue([planIndex]);
  }

  async function disconnect() {
    polling = false;
    if (pollTimer) clearTimeout(pollTimer);
    await robot.disconnect().catch(() => {});
    connected = false;
    phase = 'idle';
  }

  function finishJoint() {
    const def = CALIBRATION_PLAN[queue[qi]];
    // Only overwrite the fit if at least one point was captured; a fully-skipped
    // joint keeps whatever calibration it already had.
    if (realVals.length > 0) {
      const fit = fitLinear(realVals, simVals);
      calibration = {
        ...calibration,
        [def.joint]: {
          index: def.index,
          servoId: def.servoId,
          points: realVals.map((r, i) => ({ real: r, simRad: simVals[i] })),
          ...fit,
        },
      };
      saveJointCalibration(calibration);
    }
    realVals = [];
    simVals = [];
    if (qi + 1 >= queue.length) {
      phase = 'done';
    } else {
      qi += 1;
      pi = 0;
    }
  }

  function advancePoint() {
    const def = CALIBRATION_PLAN[queue[qi]];
    if (pi + 1 >= def.points.length) finishJoint();
    else pi += 1;
  }

  function capture() {
    if (phase !== 'calibrating' || liveReal == null || !point) return;
    realVals.push(liveReal);
    simVals.push(point.simRad);
    advancePoint();
  }

  function skip() {
    if (phase !== 'calibrating') return;
    advancePoint();
  }

  function restart() {
    calibration = {};
    if (connected) startQueue(CALIBRATION_PLAN.map((_, i) => i));
    else phase = 'idle';
  }

  function downloadCalibration() {
    const blob = new Blob([JSON.stringify(calibration, null, 2)], {
      type: 'application/json',
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'joint_calibration.json';
    a.click();
    URL.revokeObjectURL(url);
  }

  // ── Test drive: command each joint to a sim angle via its fit ───────────────
  // This checks the calibration end-to-end — a commanded angle should move the
  // real joint to that angle. Requires torque ON, so it's opt-in.
  let testing = $state(false);
  let testVals = $state<Record<string, number>>({});
  let testError = $state<string | null>(null);
  let writing = false;
  const pending = new Set<string>();

  /** Slider range (rad) for a joint — the span it was calibrated over, padded. */
  function jointSimRange(joint: string): [number, number] {
    const c = calibration[joint];
    if (c && c.points.length) {
      const rs = c.points.map((p) => p.simRad);
      let lo = Math.min(...rs);
      let hi = Math.max(...rs);
      if (hi - lo < 0.2) {
        lo -= 0.3;
        hi += 0.3;
      }
      return [lo, hi];
    }
    return [-Math.PI / 2, Math.PI / 2];
  }

  function testServoIds(): number[] {
    return Object.values(calibration).map((c) => c.servoId);
  }

  async function enableTest() {
    testError = null;
    try {
      await robot.setTorque(testServoIds(), true);
      for (const j of Object.keys(calibration)) if (testVals[j] == null) testVals[j] = 0;
      testing = true;
    } catch (e) {
      testError = e instanceof Error ? e.message : String(e);
    }
  }

  async function disableTest() {
    testing = false;
    await robot.setTorque(testServoIds(), false).catch(() => {});
  }

  /** Coalescing writer: queues joint moves and flushes without overlapping I/O. */
  async function driveJoint(joint: string) {
    if (!testing) return;
    pending.add(joint);
    if (writing) return;
    writing = true;
    try {
      while (pending.size) {
        const map = new Map<number, number>();
        for (const j of [...pending]) {
          pending.delete(j);
          const cal = calibration[j];
          if (!cal) continue;
          const servo = simRadToServo(testVals[j], cal);
          if (servo != null) map.set(cal.servoId, servo);
        }
        if (map.size) await robot.syncWritePositions(map);
      }
    } catch (e) {
      testError = e instanceof Error ? e.message : String(e);
    } finally {
      writing = false;
    }
  }

  // ── Verification ───────────────────────────────────────────────────────────
  let verifyStatus = $state<string | null>(null);
  let verifyBusy = $state(false);

  function poseTargets(ctrlRad: number[]): Map<number, number> {
    const targets = new Map<number, number>();
    JOINT_NAMES.forEach((name, i) => {
      const cal = calibration[name];
      if (!cal) return;
      const servo = simRadToServo(ctrlRad[i], cal);
      if (servo != null) targets.set(cal.servoId, servo);
    });
    return targets;
  }

  async function runPose(index: number) {
    if (verifyBusy) return;
    const pose = VERIFICATION_POSES[index];
    const targets = poseTargets(pose.ctrlRad);
    if (targets.size === 0) {
      verifyStatus = 'No fitted joints available for this pose.';
      return;
    }
    verifyBusy = true;
    verifyStatus = `Moving to “${pose.label}”…`;
    try {
      const ids = [...targets.keys()];
      await robot.setTorque(ids, true);
      const current = await robot.syncReadPositions(ids);
      const steps = 60;
      for (let s = 1; s <= steps; s++) {
        const t = s / steps;
        const interp = new Map<number, number>();
        for (const [id, target] of targets) {
          const from = current.get(id) ?? target;
          interp.set(id, Math.round(from + t * (target - from)));
        }
        await robot.syncWritePositions(interp);
        await new Promise((r) => setTimeout(r, 30));
      }
      verifyStatus = `At “${pose.label}”. Confirm it matches, then disable torque.`;
    } catch (e) {
      verifyStatus = 'Move failed: ' + (e instanceof Error ? e.message : String(e));
    } finally {
      verifyBusy = false;
    }
  }

  async function releaseTorque() {
    try {
      await robot.setTorque(
        CALIBRATION_PLAN.map((j) => j.servoId),
        false,
      );
      verifyStatus = 'Torque disabled — arm is free to move by hand.';
    } catch (e) {
      verifyStatus = 'Failed to disable torque: ' + (e instanceof Error ? e.message : String(e));
    }
  }

  function onKey(e: KeyboardEvent) {
    if (e.target instanceof HTMLInputElement) return;
    if (phase !== 'calibrating') return;
    if (e.key === ' ') {
      e.preventDefault();
      capture();
    } else if (e.key === 's' || e.key === 'S') {
      skip();
    }
  }

  $effect(() => {
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  });

  onDestroy(() => {
    polling = false;
    if (pollTimer) clearTimeout(pollTimer);
    robot.disconnect().catch(() => {});
  });
</script>

<div class="joints">
  {#if !connected}
    <div class="connect-card">
      <h2>Connect to the arm</h2>
      <p class="hint">
        The SO-101's six Feetech servos are driven over WebSerial via
        <code>feetech.js</code>. Plug the arm in, click connect, and pick the USB serial device.
        Servos are addressed by ID 1–6 in joint order (shoulder_pan → gripper).
      </p>
      <p class="hint">
        Torque stays <strong>off</strong> during capture so you can pose the arm by hand — exactly
        like the Python tool. (Reference sim images from <code>calibrate_joints_real.py</code> are
        not rendered here; the target angle for each pose is shown numerically instead.)
      </p>
      <div class="controls">
        <button class="primary" onclick={connect}>Connect servos…</button>
        <label class="file-btn">
          {loadedName ?? 'Load existing calibration…'}
          <input type="file" accept="application/json,.json" onchange={loadCalibrationFile} />
        </label>
      </div>
      {#if loadedName}
        <div class="status ok">
          Loaded {Object.keys(calibration).length} joint(s) from {loadedName}. After connecting,
          redo just the joints you want to correct.
        </div>
      {/if}
      {#if error}<div class="status bad">Error: {error}</div>{/if}
    </div>
  {:else}
    <div class="toolbar">
      <span class="pill ok">● connected</span>
      <button onclick={disconnect}>Disconnect</button>
      <button onclick={restart} disabled={phase === 'idle'}>Restart</button>
      <label class="file-btn" class:disabled={phase === 'calibrating'}>
        Load calibration…
        <input
          type="file"
          accept="application/json,.json"
          onchange={loadCalibrationFile}
          disabled={phase === 'calibrating'}
        />
      </label>
      <button onclick={downloadCalibration} disabled={Object.keys(calibration).length === 0}>
        Download joint_calibration.json
      </button>
    </div>

    {#if phase === 'calibrating' && jointDef && point}
      <div class="capture">
        <div class="progress">
          {queue.length === 1 ? 'Recalibrating' : `Joint ${qi + 1}/${queue.length}`} · point
          {pi + 1}/{jointDef.points.length}
        </div>
        <h2>{jointDef.joint}</h2>
        <p class="desc">{jointDef.description} <span class="dim">(servo ID {jointDef.servoId})</span></p>

        <div class="target">
          <div class="target-label">Move to: <strong>{point.label}</strong></div>
          <div class="target-angle">
            {deg(point.simRad).toFixed(1)}° <span class="dim">({point.simRad.toFixed(4)} rad)</span>
          </div>
        </div>

        <div class="live">
          live servo reading:
          <span class="live-val">{liveReal != null ? liveReal : '—'}</span>
        </div>

        <div class="controls">
          <button class="primary" onclick={capture} disabled={liveReal == null}>
            Capture (Space)
          </button>
          <button onclick={skip}>Skip (S)</button>
        </div>
        <p class="hint">Only <strong>{jointDef.joint}</strong> matters — other joints can be anywhere.</p>
      </div>
    {/if}

    {#if connected}
      <div class="results">
        <h2>Per-joint calibration</h2>
        <p class="hint">Redo any single joint without repeating the others.</p>
        <table class="readout">
          <thead>
            <tr><th>Joint</th><th>Servo</th><th>Scale</th><th>Offset</th><th>R²</th><th>pts</th><th></th></tr>
          </thead>
          <tbody>
            {#each CALIBRATION_PLAN as def, i (def.joint)}
              {@const c = calibration[def.joint]}
              {@const active = phase === 'calibrating' && queue[qi] === i}
              <tr class:activerow={active}>
                <td>{def.joint}</td>
                <td>{def.servoId}</td>
                <td class="cal">{c?.scale != null ? c.scale.toFixed(5) : '—'}</td>
                <td class="cal">{c?.offset != null ? c.offset.toFixed(4) : '—'}</td>
                <td>{c?.r2 != null ? c.r2.toFixed(4) : '—'}</td>
                <td>{c?.points.length ?? 0}</td>
                <td>
                  {#if active}
                    <span class="dim">capturing…</span>
                  {:else}
                    <button class="mini" onclick={() => recalibrateJoint(i)} disabled={phase === 'calibrating'}>
                      {c ? 'Redo' : 'Calibrate'}
                    </button>
                  {/if}
                </td>
              </tr>
            {/each}
          </tbody>
        </table>
      </div>
    {/if}

    {#if connected && phase !== 'calibrating' && Object.keys(calibration).length > 0}
      <div class="results">
        <h2>Test drive <span class="dim">(moves the robot)</span></h2>
        <p class="hint warn-text">
          ⚠ Commands each joint to a target angle through its fit — the real joint should reach that
          angle. Enables torque. Keep the workspace clear and a hand near the power switch.
        </p>
        <div class="controls">
          {#if !testing}
            <button class="primary" onclick={enableTest}>Enable torque &amp; test</button>
          {:else}
            <button onclick={disableTest}>Disable torque</button>
          {/if}
        </div>

        {#if testing}
          <div class="jsliders">
            {#each CALIBRATION_PLAN as def (def.joint)}
              {#if calibration[def.joint]}
                {@const range = jointSimRange(def.joint)}
                <label title={def.joint}>{def.joint}</label>
                <input
                  type="range"
                  min={range[0]}
                  max={range[1]}
                  step="0.01"
                  bind:value={testVals[def.joint]}
                  oninput={() => driveJoint(def.joint)}
                />
                <span class="val">{deg(testVals[def.joint] ?? 0).toFixed(0)}°</span>
                <span class="tick">→ {simRadToServo(testVals[def.joint] ?? 0, calibration[def.joint]) ?? '—'}</span>
              {/if}
            {/each}
          </div>
        {/if}
        {#if testError}<div class="status bad">Error: {testError}</div>{/if}
      </div>
    {/if}

    {#if phase === 'done'}
      <div class="verify">
        <h2>Verification <span class="dim">(optional — moves the robot)</span></h2>
        <p class="hint warn-text">
          ⚠ These buttons enable torque and drive the arm to test poses using your fit. Keep the
          workspace clear and a hand near the power switch. Movement is interpolated over ~1.8 s.
        </p>
        <div class="controls">
          {#each VERIFICATION_POSES as pose, i (pose.label)}
            <button onclick={() => runPose(i)} disabled={verifyBusy}>{pose.label}</button>
          {/each}
          <button onclick={releaseTorque}>Disable torque</button>
        </div>
        {#if verifyStatus}<div class="status">{verifyStatus}</div>{/if}
      </div>
    {/if}
  {/if}
</div>

<style>
  .joints {
    max-width: 760px;
  }
  .connect-card,
  .capture,
  .results,
  .verify {
    background: #1c2027;
    border: 1px solid #2a2f38;
    border-radius: 8px;
    padding: 1rem 1.2rem;
    margin-bottom: 1rem;
  }
  h2 {
    margin: 0 0 0.5rem;
    font-size: 1.05rem;
  }
  .toolbar {
    display: flex;
    gap: 0.5rem;
    align-items: center;
    margin-bottom: 1rem;
    flex-wrap: wrap;
  }
  .pill {
    font-size: 0.8rem;
    padding: 0.25rem 0.6rem;
    border-radius: 999px;
    border: 1px solid #2a2f38;
  }
  .pill.ok {
    color: #4ade80;
    border-color: #1f7a1f;
  }
  .progress {
    color: #8b93a1;
    font-size: 0.85rem;
    margin-bottom: 0.25rem;
  }
  .desc {
    color: #b8bfc9;
    margin: 0 0 1rem;
  }
  .dim {
    color: #8b93a1;
  }
  .target {
    background: #0f1114;
    border: 1px solid #2a2f38;
    border-radius: 6px;
    padding: 0.8rem 1rem;
    margin-bottom: 0.9rem;
  }
  .target-label {
    font-size: 1rem;
  }
  .target-angle {
    font-family: ui-monospace, monospace;
    font-size: 1.4rem;
    margin-top: 0.25rem;
    color: #ffd24a;
  }
  .live {
    font-family: ui-monospace, monospace;
    margin-bottom: 0.9rem;
  }
  .live-val {
    font-size: 1.3rem;
    color: #4ade80;
    margin-left: 0.4rem;
  }
  .warn-text {
    color: #ffb454;
  }
  .mini {
    padding: 0.2rem 0.6rem;
    font-size: 0.78rem;
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
  .file-btn.disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }
  .file-btn input {
    display: none;
  }
  tr.activerow {
    background: rgba(31, 111, 235, 0.12);
  }
  tr.activerow td {
    color: #7cc0ff;
  }
  .jsliders {
    display: grid;
    grid-template-columns: 7rem 1fr 2.5rem 3.5rem;
    gap: 0.4rem 0.6rem;
    align-items: center;
    margin-top: 0.75rem;
  }
  .jsliders label {
    font-family: ui-monospace, monospace;
    font-size: 0.8rem;
    color: #b8bfc9;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .jsliders .val {
    font-family: ui-monospace, monospace;
    font-size: 0.85rem;
    text-align: right;
  }
  .jsliders .tick {
    font-family: ui-monospace, monospace;
    font-size: 0.78rem;
    color: #4ade80;
    text-align: right;
  }
</style>
