<script lang="ts">
  // Loading overlay adapted from the roboter project's LoadingScreen — falling,
  // tumbling line-art tech icons behind a card with an animated robot head and a
  // progress bar. Icons are inline lucide SVGs (matching roboter's lucide-react).
  let { message = 'Loading…', percent = null }: { message?: string; percent?: number | null } =
    $props();

  // lucide icon inner paths (24×24 viewBox, stroke = currentColor).
  const BOT =
    '<path d="M12 8V4H8"/><rect width="16" height="12" x="4" y="8" rx="2"/><path d="M2 14h2"/><path d="M20 14h2"/><path d="M15 13v2"/><path d="M9 13v2"/>';
  const ICONS = [
    BOT,
    '<rect width="16" height="16" x="4" y="4" rx="2"/><rect width="6" height="6" x="9" y="9" rx="1"/><path d="M15 2v2"/><path d="M15 20v2"/><path d="M2 15h2"/><path d="M2 9h2"/><path d="M20 15h2"/><path d="M20 9h2"/><path d="M9 2v2"/><path d="M9 20v2"/>',
    '<path d="M4 14a1 1 0 0 1-.78-1.63l9.9-10.2a.5.5 0 0 1 .86.46l-1.92 6.02A1 1 0 0 0 13 10h7a1 1 0 0 1 .78 1.63l-9.9 10.2a.5.5 0 0 1-.86-.46l1.92-6.02A1 1 0 0 0 11 14z"/>',
    '<path d="M21 8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16Z"/><path d="m3.3 7 8.7 5 8.7-5"/><path d="M12 22V12"/>',
    '<rect width="18" height="18" x="3" y="3" rx="2"/><path d="M3 9h18"/><path d="M3 15h18"/><path d="M9 3v18"/><path d="M15 3v18"/>',
    '<rect width="18" height="18" x="3" y="3" rx="2"/><path d="M11 9h4a2 2 0 0 0 2-2V3"/><circle cx="9" cy="9" r="2"/><path d="M7 21v-4a2 2 0 0 1 2-2h4"/><circle cx="15" cy="15" r="2"/>',
  ];
  const cols = [5, 15, 25, 35, 45, 55, 65, 75, 85, 95, 10, 30, 50, 70, 90];
  const drops = cols.map((left, i) => ({
    left,
    paths: ICONS[i % ICONS.length],
    delay: (i * 0.7) % 5,
    duration: 7 + ((i * 1.3) % 4),
  }));
</script>

<div class="loading">
  <div class="floaters" aria-hidden="true">
    {#each drops as d, i (i)}
      <span
        class="floater"
        style="left:{d.left}%; animation-delay:{d.delay}s; animation-duration:{d.duration}s;"
      >
        <svg
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          stroke-width="1.5"
          stroke-linecap="round"
          stroke-linejoin="round">{@html d.paths}</svg
        >
      </span>
    {/each}
  </div>

  <div class="card loader-card">
    <div class="head-row">
      <svg
        class="robot"
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
        stroke-linecap="round"
        stroke-linejoin="round">{@html BOT}</svg
      >
      <div>
        <h2>Preparing the simulator</h2>
        <p class="hint">Loading MuJoCo &amp; the SO-101 model. Please hold tight.</p>
      </div>
    </div>
    <div class="divider"></div>
    <div class="bar-row">
      <div class="bar-label">
        <span>{message}</span>
        {#if percent != null}<span>{Math.round(percent)}%</span>{/if}
      </div>
      <div class="track">
        <div
          class="fill"
          class:indeterminate={percent == null}
          style={percent != null ? `width:${Math.max(0, Math.min(100, percent))}%` : ''}
        ></div>
      </div>
    </div>
  </div>
</div>

<style>
  .loading {
    position: absolute;
    inset: 0;
    display: flex;
    align-items: center;
    justify-content: center;
    background: var(--surface-2);
    border-radius: 8px;
    overflow: hidden;
    z-index: 20;
  }
  .floaters {
    position: absolute;
    inset: 0;
    pointer-events: none;
    color: var(--ink);
  }
  .floater {
    position: absolute;
    top: -8%;
    width: 48px;
    height: 48px;
    opacity: 0.12;
    animation-name: fall;
    animation-timing-function: linear;
    animation-iteration-count: infinite;
  }
  .floater svg {
    width: 100%;
    height: 100%;
  }
  @keyframes fall {
    0% {
      top: -8%;
      transform: rotate(0deg);
    }
    100% {
      top: 108%;
      transform: rotate(360deg);
    }
  }
  .loader-card {
    position: relative;
    width: min(92%, 26rem);
  }
  .head-row {
    display: flex;
    align-items: center;
    gap: 1rem;
  }
  .robot {
    width: 3rem;
    height: 3rem;
    flex-shrink: 0;
    color: var(--ink);
    transform-origin: 50% 60%;
    animation: robotHead 6s ease-in-out infinite;
  }
  @keyframes robotHead {
    0%, 20%, 40%, 60% { transform: rotate(0deg) translateY(0); }
    10% { transform: rotate(20deg); }
    30% { transform: rotate(-20deg); }
    50% { transform: rotate(360deg); }
    70% { transform: translateY(-8px); }
    80% { transform: translateY(-4px); }
    100% { transform: translateY(0); }
  }
  h2 {
    font-size: 1.25rem;
    font-weight: 600;
    margin: 0;
  }
  .divider {
    border-top: 1px solid var(--line);
    margin: 1rem 0;
  }
  .bar-label {
    display: flex;
    justify-content: space-between;
    font-size: 0.85rem;
    color: var(--muted);
    margin-bottom: 0.4rem;
  }
  .track {
    width: 100%;
    height: 0.5rem;
    border-radius: 999px;
    background: #e5e7eb;
    overflow: hidden;
  }
  .fill {
    height: 100%;
    background: var(--accent);
    border-radius: 999px;
    transition: width 0.3s ease;
  }
  .fill.indeterminate {
    width: 40%;
    animation: slide 1.3s ease-in-out infinite;
  }
  @keyframes slide {
    0% { margin-left: -40%; }
    100% { margin-left: 100%; }
  }
</style>
