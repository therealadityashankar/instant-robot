<script lang="ts">
  // Circular 3-omniwheel drive triad for LeKiwi base.
  // 3-sector donut representing forward (W), back-left (A), back-right (D),
  // with geometric arrows, letters to the right of each arrow,
  // and force percentage on the left side of each arrow.
  // Shift/Reverse toggle in the center hub.
  import { onMount } from 'svelte';

  let {
    onmove,
    baseForces,
  }: {
    onmove: (fwd: number, bl: number, br: number) => void;
    baseForces?: { fwd: number; bl: number; br: number };
  } = $props();

  type Dir = 'fwd' | 'bl' | 'br';
  const KEY: Record<string, Dir> = { w: 'fwd', a: 'bl', d: 'br' };
  let held = $state<Record<Dir, boolean>>({ fwd: false, bl: false, br: false });
  let shiftKey = $state(false); // Shift held on keyboard
  let toggleRev = $state(false); // On-screen reverse toggle in center
  const reverse = $derived(shiftKey || toggleRev);

  const forceFwd = $derived(Math.abs(baseForces?.fwd ?? 0));
  const forceBl = $derived(Math.abs(baseForces?.bl ?? 0));
  const forceBr = $derived(Math.abs(baseForces?.br ?? 0));

  const pctFwd = $derived(Math.round((held.fwd ? 1 : forceFwd) * 100));
  const pctBl = $derived(Math.round((held.bl ? 1 : forceBl) * 100));
  const pctBr = $derived(Math.round((held.br ? 1 : forceBr) * 100));

  function editable(t: EventTarget | null) {
    const el = t as HTMLElement | null;
    return !!el && (el.isContentEditable || ['INPUT', 'TEXTAREA', 'SELECT'].includes(el.tagName));
  }

  function press(d: Dir, e: PointerEvent) {
    e.preventDefault();
    (e.currentTarget as HTMLElement).setPointerCapture?.(e.pointerId);
    held[d] = true;
  }

  const release = (d: Dir) => {
    held[d] = false;
  };

  function toggleCenterRev(e: MouseEvent) {
    e.preventDefault();
    toggleRev = !toggleRev;
  }

  onMount(() => {
    const kd = (e: KeyboardEvent) => {
      if (e.key === 'Shift') { shiftKey = true; return; }
      if (editable(e.target)) return;
      const d = KEY[e.key.toLowerCase()];
      if (d) { held[d] = true; e.preventDefault(); }
    };
    const ku = (e: KeyboardEvent) => {
      if (e.key === 'Shift') { shiftKey = false; return; }
      const d = KEY[e.key.toLowerCase()];
      if (d) { held[d] = false; e.preventDefault(); }
    };
    const reset = () => { held = { fwd: false, bl: false, br: false }; shiftKey = false; };
    window.addEventListener('keydown', kd);
    window.addEventListener('keyup', ku);
    window.addEventListener('blur', reset);
    let raf = 0;
    const tick = () => {
      const s = reverse ? -1 : 1;
      onmove(held.fwd ? s : 0, held.bl ? s : 0, held.br ? s : 0);
      raf = requestAnimationFrame(tick);
    };
    tick();
    return () => {
      window.removeEventListener('keydown', kd);
      window.removeEventListener('keyup', ku);
      window.removeEventListener('blur', reset);
      cancelAnimationFrame(raf);
    };
  });
</script>

<div class="triad-donut-wrap" class:rev={reverse}>
  <svg
    viewBox="0 0 120 120"
    class="donut-svg"
    aria-label="3-Omniwheel Drive Controls"
  >
    <!-- Background outer ring -->
    <circle cx="60" cy="60" r="57" class="outer-glow" />

    <!-- Top Sector: Forward (W) -->
    <!-- svelte-ignore a11y_click_events_have_key_events -->
    <g
      class="sector-group"
      class:active={held.fwd || forceFwd > 0.03}
      role="button"
      tabindex="0"
      aria-label="Drive forward (W)"
      onpointerdown={(e) => press('fwd', e)}
      onpointerup={() => release('fwd')}
      onpointerleave={() => release('fwd')}
      onpointercancel={() => release('fwd')}
    >
      <path
        class="sector-path"
        d="M 12.51 30.32 A 56 56 0 0 1 107.49 30.32 L 79.50 47.81 A 23 23 0 0 0 40.50 47.81 Z"
      />
      <!-- Identical Geometric Arrow (Up) -->
      <g transform="translate(60, 60) rotate(0)">
        {#if !reverse && (baseForces?.fwd ?? 0) >= 0}
          <line x1="0" y1="-28" x2="0" y2="-40" class="arrow-shaft" />
          <polygon points="-4.5,-39 0,-47 4.5,-39" class="arrow-head" />
        {:else}
          <line x1="0" y1="-46" x2="0" y2="-34" class="arrow-shaft" />
          <polygon points="-4.5,-35 0,-27 4.5,-35" class="arrow-head" />
        {/if}
        <!-- Alphabet letter to the RIGHT of the arrow -->
        <text x="14" y="-38" class="sector-key">W</text>
        <!-- Movement force percentage to the LEFT of the arrow -->
        {#if pctFwd > 0}
          <text x="-14" y="-38" class="sector-pct">{pctFwd}%</text>
        {/if}
      </g>
    </g>

    <!-- Bottom-Right Sector: Back-Right (D) -->
    <!-- svelte-ignore a11y_click_events_have_key_events -->
    <g
      class="sector-group"
      class:active={held.br || forceBr > 0.03}
      role="button"
      tabindex="0"
      aria-label="Drive back-right (D)"
      onpointerdown={(e) => press('br', e)}
      onpointerup={() => release('br')}
      onpointerleave={() => release('br')}
      onpointercancel={() => release('br')}
    >
      <path
        class="sector-path"
        d="M 109.44 33.71 A 56 56 0 0 1 61.95 115.97 L 60.80 82.99 A 23 23 0 0 0 80.31 49.20 Z"
      />
      <!-- Identical Geometric Arrow (Bottom-Right, 120 deg) -->
      <g transform="translate(60, 60) rotate(120)">
        {#if !reverse && (baseForces?.br ?? 0) >= 0}
          <line x1="0" y1="-28" x2="0" y2="-40" class="arrow-shaft" />
          <polygon points="-4.5,-39 0,-47 4.5,-39" class="arrow-head" />
        {:else}
          <line x1="0" y1="-46" x2="0" y2="-34" class="arrow-shaft" />
          <polygon points="-4.5,-35 0,-27 4.5,-35" class="arrow-head" />
        {/if}
        <!-- Alphabet letter to the RIGHT of the arrow -->
        <text x="14" y="-38" class="sector-key" transform="rotate(-120, 14, -38)">D</text>
        <!-- Movement force percentage to the LEFT of the arrow -->
        {#if pctBr > 0}
          <text x="-14" y="-38" class="sector-pct" transform="rotate(-120, -14, -38)">{pctBr}%</text>
        {/if}
      </g>
    </g>

    <!-- Bottom-Left Sector: Back-Left (A) -->
    <!-- svelte-ignore a11y_click_events_have_key_events -->
    <g
      class="sector-group"
      class:active={held.bl || forceBl > 0.03}
      role="button"
      tabindex="0"
      aria-label="Drive back-left (A)"
      onpointerdown={(e) => press('bl', e)}
      onpointerup={() => release('bl')}
      onpointerleave={() => release('bl')}
      onpointercancel={() => release('bl')}
    >
      <path
        class="sector-path"
        d="M 58.05 115.97 A 56 56 0 0 1 10.56 33.71 L 39.69 49.20 A 23 23 0 0 0 59.20 82.99 Z"
      />
      <!-- Identical Geometric Arrow (Bottom-Left, 240 deg) -->
      <g transform="translate(60, 60) rotate(240)">
        {#if !reverse && (baseForces?.bl ?? 0) >= 0}
          <line x1="0" y1="-28" x2="0" y2="-40" class="arrow-shaft" />
          <polygon points="-4.5,-39 0,-47 4.5,-39" class="arrow-head" />
        {:else}
          <line x1="0" y1="-46" x2="0" y2="-34" class="arrow-shaft" />
          <polygon points="-4.5,-35 0,-27 4.5,-35" class="arrow-head" />
        {/if}
        <!-- Alphabet letter to the RIGHT of the arrow -->
        <text x="14" y="-38" class="sector-key" transform="rotate(-240, 14, -38)">A</text>
        <!-- Movement force percentage to the LEFT of the arrow -->
        {#if pctBl > 0}
          <text x="-14" y="-38" class="sector-pct" transform="rotate(-240, -14, -38)">{pctBl}%</text>
        {/if}
      </g>
    </g>

    <!-- Center Hub: Shift / Reverse Button -->
    <!-- svelte-ignore a11y_click_events_have_key_events -->
    <g
      class="center-hub"
      class:rev-active={reverse}
      role="button"
      tabindex="0"
      aria-label="Toggle Reverse (Shift)"
      onclick={toggleCenterRev}
    >
      <circle cx="60" cy="60" r="19" class="center-circle" />
      <text x="60" y="55" class="center-shift-text">SHIFT</text>
      <text x="60" y="67" class="center-sub-text">{reverse ? 'REV ✓' : 'REV ⇄'}</text>
    </g>
  </svg>
</div>

<style>
  .triad-donut-wrap {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    user-select: none;
    touch-action: none;
  }

  .donut-svg {
    width: 6.8rem;
    height: 6.8rem;
    overflow: visible;
    filter: drop-shadow(0 2px 6px rgba(0, 0, 0, 0.35));
  }

  .outer-glow {
    fill: none;
    stroke: rgba(0, 0, 0, 0.4);
    stroke-width: 1.5;
  }

  /* Donut 120-degree sector buttons */
  .sector-group {
    cursor: pointer;
    transition: transform 0.08s ease;
  }

  .sector-path {
    fill: #f1f5f9;
    stroke: #0f172a;
    stroke-width: 1.8;
    transition: fill 0.08s ease, stroke 0.08s ease;
  }

  .sector-group:hover .sector-path {
    fill: #e2e8f0;
  }

  .sector-group.active .sector-path {
    fill: #f59e0b;
    stroke: #000000;
  }

  /* Identical Solid Black Arrows */
  .arrow-shaft {
    stroke: #000000;
    stroke-width: 3.2px;
    stroke-linecap: round;
    pointer-events: none;
  }

  .arrow-head {
    fill: #000000;
    stroke: #000000;
    stroke-width: 1px;
    stroke-linejoin: round;
    pointer-events: none;
  }

  /* Letter to the right of arrow */
  .sector-key {
    fill: #000000;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    font-size: 11px;
    font-weight: 900;
    text-anchor: middle;
    dominant-baseline: central;
    pointer-events: none;
  }

  /* Percentage of max movement force to the left of arrow */
  .sector-pct {
    fill: #000000;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace;
    font-size: 8px;
    font-weight: 900;
    text-anchor: middle;
    dominant-baseline: central;
    pointer-events: none;
  }

  /* Center Hub: Shift / Reverse */
  .center-hub {
    cursor: pointer;
  }

  .center-circle {
    fill: #ffffff;
    stroke: #0f172a;
    stroke-width: 2;
    transition: fill 0.12s ease, stroke 0.12s ease;
  }

  .center-hub:hover .center-circle {
    fill: #f8fafc;
  }

  .center-hub.rev-active .center-circle {
    fill: #f97316;
    stroke: #000000;
  }

  .center-shift-text {
    fill: #000000;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace;
    font-size: 8px;
    font-weight: 900;
    letter-spacing: 0.5px;
    text-anchor: middle;
    dominant-baseline: central;
    pointer-events: none;
  }

  .center-sub-text {
    fill: #000000;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    font-size: 7.5px;
    font-weight: 700;
    text-anchor: middle;
    dominant-baseline: central;
    pointer-events: none;
  }

  .triad-donut-wrap.rev .sector-path {
    stroke: #ea580c;
  }
</style>
