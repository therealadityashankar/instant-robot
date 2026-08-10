<script lang="ts">
  // Wheel identity + drive-direction calibration for the LeKiwi base. Lives in the
  // Calibrate modal alongside the camera and joint calibrations, since it's the
  // same kind of thing: a one-off setup step whose result is saved and reused.
  import { baseLink } from './lib/baseLink.svelte';

  const c = $derived(baseLink.config);
</script>

<h3>Wheel motors</h3>
<p class="hint">
  The 3 omniwheel motors share the arm's serial bus and are detected automatically when you
  connect the motors — a chain of more than 6 servos means a base is attached. Change these only
  if your wiring differs.
</p>
<div class="pickrow">
  <span>Wheel servo IDs</span>
  {#each c.wheelIds as _id, i (i)}
    <input
      type="number"
      style="width:3.4rem"
      value={c.wheelIds[i]}
      onchange={(e) => {
        c.wheelIds[i] = +e.currentTarget.value;
        baseLink.persist();
      }}
    />
  {/each}
  {#if baseLink.connected}
    <button onclick={() => baseLink.stopWheels()}>Stop</button>
  {/if}
</div>
<div class="status {baseLink.connected ? 'ok' : 'warn'}">
  {baseLink.connected ? 'wheels live' : 'no wheels detected — connect the motors first'}
</div>

<h3>Drive directions</h3>
<p class="hint">
  The base has 3 drive directions 120° apart: <strong>forward</strong>, <strong>back-left</strong>
  (+120°) and <strong>back-right</strong> (−120°). Each is one pair of wheels. Hit <em>Test</em> on
  a pair, watch which of those the base slid toward, and click it — the greyed row is the opposite
  direction, if it went the other way. Then the rotation test. Give it clear floor space.
</p>
{#each [0, 1, 2] as i (i)}
  <div class="pickrow">
    <button
      style="min-width:4.5rem"
      disabled={!baseLink.connected || baseLink.testing}
      onclick={() => baseLink.testPair(i)}
    >
      {baseLink.testing ? '…' : `▶ Pair ${i + 1}`}
    </button>
    <div class="btnwrap">
      <button disabled={baseLink.testing} onclick={() => baseLink.labelPair(i, 'forward')}>Forward</button>
      <button disabled={baseLink.testing} onclick={() => baseLink.labelPair(i, 'backLeft')}>Back-left</button>
      <button disabled={baseLink.testing} onclick={() => baseLink.labelPair(i, 'backRight')}>Back-right</button>
      <button class="ghost" disabled={baseLink.testing} onclick={() => baseLink.labelPair(i, 'backward')}>Backward</button>
      <button class="ghost" disabled={baseLink.testing} onclick={() => baseLink.labelPair(i, 'frontRight')}>Front-right</button>
      <button class="ghost" disabled={baseLink.testing} onclick={() => baseLink.labelPair(i, 'frontLeft')}>Front-left</button>
    </div>
  </div>
{/each}
<div class="pickrow">
  <button
    style="min-width:4.5rem"
    disabled={!baseLink.connected || baseLink.testing}
    onclick={() => baseLink.testRotate()}
  >
    {baseLink.testing ? '…' : '▶ Rotate'}
  </button>
  <div class="btnwrap">
    <button disabled={baseLink.testing} onclick={() => baseLink.labelRotate(true)}>Turned left (CCW)</button>
    <button disabled={baseLink.testing} onclick={() => baseLink.labelRotate(false)}>Turned right (CW)</button>
  </div>
</div>
<div class="pickrow">
  <button onclick={() => baseLink.resetCalibration()} title="restore default drive patterns">
    Reset to defaults
  </button>
</div>
<div class="status {baseLink.connected ? 'ok' : 'warn'}">
  fwd [{c.forward.join(',')}] · bk-L [{c.backLeft.join(',')}] · bk-R [{c.backRight.join(',')}] ·
  rot [{c.rotate.join(',')}]
</div>
{#if baseLink.error}<div class="status bad">{baseLink.error}</div>{/if}

<style>
  h3 {
    margin: 1.1rem 0 0.3rem;
    font-size: 0.95rem;
  }
  .hint {
    margin: 0 0 0.5rem;
    font-size: 0.78rem;
    opacity: 0.75;
    line-height: 1.45;
  }
  .pickrow {
    display: flex;
    align-items: center;
    gap: 0.4rem;
    margin-bottom: 0.4rem;
    flex-wrap: wrap;
  }
  .btnwrap {
    display: flex;
    gap: 0.3rem;
    flex-wrap: wrap;
  }
  .status {
    margin: 0.4rem 0;
    padding: 0.35rem 0.6rem;
    border-radius: 6px;
    font-size: 0.78rem;
  }
  .status.ok {
    background: #ecfdf5;
    color: #065f46;
  }
  .status.warn {
    background: #fffbeb;
    color: #92400e;
  }
  .status.bad {
    background: #fef2f2;
    color: #991b1b;
  }
  button.ghost {
    opacity: 0.55;
  }
</style>
