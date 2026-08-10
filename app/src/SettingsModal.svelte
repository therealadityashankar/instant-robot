<script lang="ts">
  // App-wide preferences, kept out of the driving controls. Opened from the header.
  import { settings } from './lib/settings.svelte';
  import { baseLink } from './lib/baseLink.svelte';
</script>

{#if settings.open}
  <!-- svelte-ignore a11y_click_events_have_key_events, a11y_no_static_element_interactions -->
  <div class="backdrop" onclick={() => (settings.open = false)}>
    <!-- svelte-ignore a11y_click_events_have_key_events, a11y_no_static_element_interactions -->
    <div class="modal" role="dialog" tabindex="-1" onclick={(e) => e.stopPropagation()}>
      <header>
        <h2>Settings</h2>
        <button class="close" onclick={() => (settings.open = false)}>✕</button>
      </header>

      <h3>Arm</h3>
      <div class="row">
        <label for="setspeed">Max joint speed</label>
        <input id="setspeed" type="range" min="5" max="180" step="5" bind:value={settings.maxArmSpeedDeg} />
        <span class="val">{settings.maxArmSpeedDeg}°/s</span>
      </div>
      <p class="hint">
        Caps every commanded arm motion — manual target, pick steps, shelf stages, rest and board
        view alike — so nothing ever snaps. Lower it if the real arm moves too abruptly.
      </p>

      <h3>Base</h3>
      <div class="row">
        <label for="setwheel">Wheel speed</label>
        <input
          id="setwheel"
          type="number"
          step="25"
          bind:value={baseLink.config.speed}
          onchange={() => baseLink.persist()}
        />
        <span class="val">units</span>
      </div>
      <p class="hint">
        Raw Feetech wheel speed used for driving and for the direction-calibration pulses. Wheel
        IDs and drive directions live under <em>Calibrate → Base</em>.
      </p>

      {#if settings.armOffset}
        <h3>Arm mounting</h3>
        <div class="row">
          <label for="armoffx">Arm on base (m)</label>
          <span class="triple">
            {#each [0, 1, 2] as ax (ax)}
              <input
                id={ax === 0 ? 'armoffx' : undefined}
                type="number"
                step="0.005"
                value={settings.armOffset[ax]}
                onchange={(e) => settings.setArmOffset?.(ax as 0 | 1 | 2, +e.currentTarget.value)}
              />
            {/each}
          </span>
          <span class="val">x y z</span>
        </div>
        <p class="hint">
          Where the arm sits on the LeKiwi base. Saved per robot. The arm's mount <em>rotation</em>
          is a fixed mesh correction and isn't adjustable.
        </p>
      {/if}

      <h3>Cameras</h3>
      <div class="row">
        <label for="setexcl">One camera at a time</label>
        <input id="setexcl" type="checkbox" bind:checked={settings.exclusiveCam} />
        <span class="val"></span>
      </div>
      <div class="row">
        <label for="setres">Capture width</label>
        <select id="setres" bind:value={settings.camResW}>
          <option value={320}>320 (lowest bandwidth)</option>
          <option value={640}>640</option>
          <option value={1280}>1280 (most detail)</option>
        </select>
        <span class="val">px</span>
      </div>
      <p class="hint">
        Two USB cameras can be more than the bus will carry at once. With <em>one at a time</em> on,
        connecting either camera releases the other. Capture size applies on the next connect — hit
        <em>Refresh</em> under the camera view after changing it. Smaller frames mean coarser tag
        detection at distance, so drop it only as far as you need.
      </p>
    </div>
  </div>
{/if}

<style>
  .backdrop {
    position: fixed;
    inset: 0;
    background: rgba(17, 24, 39, 0.45);
    display: flex;
    align-items: center;
    justify-content: center;
    z-index: 50;
    padding: 1rem;
  }
  .modal {
    background: var(--panel, #fff);
    color: inherit;
    border-radius: 12px;
    padding: 1.1rem 1.3rem 1.4rem;
    width: min(34rem, 100%);
    max-height: 90vh;
    overflow: auto;
    box-shadow: 0 20px 50px rgba(0, 0, 0, 0.25);
  }
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 0.5rem;
  }
  h2 {
    margin: 0;
    font-size: 1.1rem;
  }
  h3 {
    margin: 1.1rem 0 0.4rem;
    font-size: 0.95rem;
  }
  .close {
    border: none;
    background: transparent;
    font-size: 1rem;
    cursor: pointer;
  }
  .row {
    display: grid;
    grid-template-columns: 10rem 1fr 4rem;
    align-items: center;
    gap: 0.5rem;
    margin-bottom: 0.35rem;
  }
  .val {
    font-variant-numeric: tabular-nums;
    font-size: 0.8rem;
    opacity: 0.8;
  }
  .triple {
    display: flex;
    gap: 0.3rem;
  }
  .triple input { width: 100%; min-width: 0; }
  .hint {
    margin: 0.3rem 0 0;
    font-size: 0.78rem;
    opacity: 0.75;
    line-height: 1.45;
  }
</style>
