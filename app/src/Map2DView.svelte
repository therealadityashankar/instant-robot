<script lang="ts">
  import type { StationDef } from './lib/boardSim';
  import { BOUNDARY_TAG_ID } from './lib/boardSim';

  let {
    stations = [],
    knownTags = new Map(),
    robotX = 0,
    robotY = 0,
    robotYawDeg = 0,
    hasBase = true,
    navigating = false,
    navTagId = null,
    atTag = null,
    onSelectStation,
  }: {
    stations: StationDef[];
    knownTags?: Map<number, any>;
    robotX?: number;
    robotY?: number;
    robotYawDeg?: number;
    hasBase?: boolean;
    navigating?: boolean;
    navTagId?: number | null;
    atTag?: number | null;
    onSelectStation?: (id: number) => void;
  } = $props();

  let scrollContainer = $state<HTMLDivElement>();
  let isDragging = $state(false);
  let dragStartX = $state(0);
  let dragScrollLeft = $state(0);

  // Boundary stop stations that are always visible along the route
  const boundaryStations = $derived(
    stations.filter((s) => s.isBoundary || s.navTag === BOUNDARY_TAG_ID)
  );

  // Discovered non-boundary stations derived from knownTags
  const discoveredStations = $derived(
    [...knownTags.values()]
      .filter((k) => k.id > 200 && k.id !== BOUNDARY_TAG_ID)
      .map((k) => {
        const def = stations.find((s) => s.navTag === k.id);
        return {
          id: k.id,
          navTag: k.id,
          x: k.x,
          y: k.y,
          label: k.label || def?.label || `Tag ${k.id}`,
          description: k.description || def?.description,
          stationType: def?.stationType || 'simple',
        };
      })
      .sort((a, b) => b.y - a.y)
  );

  // Stable track bounds with generous outer end padding buffer (0.35m)
  const rawMinY = $derived(
    stations.length
      ? Math.min(...stations.map((s) => s.y))
      : discoveredStations.length
        ? Math.min(...discoveredStations.map((s) => s.y))
        : -0.75
  );

  const rawMaxY = $derived(
    stations.length
      ? Math.max(...stations.map((s) => s.y))
      : discoveredStations.length
        ? Math.max(...discoveredStations.map((s) => s.y))
        : 0.75
  );

  const paddingM = 0.35;
  const minY = $derived(rawMinY - paddingM);
  const maxY = $derived(rawMaxY + paddingM);
  const spanY = $derived(Math.max(0.4, maxY - minY));

  // Position percentage along the track (2% to 98%)
  // Inverted so simulation left (+Y) maps to screen left, and simulation right (-Y) maps to screen right
  function getYPercent(y: number): number {
    const raw = ((maxY - y) / spanY) * 100;
    return Math.max(2, Math.min(98, raw));
  }

  const robotPercent = $derived(
    getYPercent(robotY)
  );

  function handleMouseDown(e: MouseEvent) {
    if (!scrollContainer) return;
    isDragging = true;
    dragStartX = e.pageX - scrollContainer.offsetLeft;
    dragScrollLeft = scrollContainer.scrollLeft;
  }

  function handleMouseMove(e: MouseEvent) {
    if (!isDragging || !scrollContainer) return;
    e.preventDefault();
    const x = e.pageX - scrollContainer.offsetLeft;
    const walk = (x - dragStartX) * 1.5;
    scrollContainer.scrollLeft = dragScrollLeft - walk;
  }

  function handleMouseUp() {
    isDragging = false;
  }

  function handleClickStation(navTag: number) {
    if (isDragging) return;
    onSelectStation?.(navTag);
  }
</script>

<div class="map-container">
  <!-- svelte-ignore a11y_no_static_element_interactions -->
  <div
    class="map-scroll-area"
    bind:this={scrollContainer}
    onmousedown={handleMouseDown}
    onmousemove={handleMouseMove}
    onmouseup={handleMouseUp}
    onmouseleave={handleMouseUp}
  >
    <div
      class="map-track-content"
      style="min-width: {Math.max(340, Math.max(stations.length, discoveredStations.length) * 80)}px;"
    >
      <!-- Central Guide Rail / Track Route -->
      <div class="track-rail">
        <div class="track-line"></div>
      </div>

      <!-- Always-Visible Straight-Line Boundary Stop Markers (Overlapping the route line, no tags on top) -->
      {#each boundaryStations as b (b.navTag + '-' + b.y)}
        {@const pct = getYPercent(b.y)}
        <div
          class="boundary-stop-line"
          style="left: {pct}%;"
          title="Stop Barrier ({b.label || 'Boundary'})"
        >
          <div class="stop-bar"></div>
        </div>
      {/each}

      <!-- Discovered Station Place Indicators (Pure black centered dots, no vertical stem lines, flat transparent labels) -->
      <div class="stations-layer">
        {#if discoveredStations.length === 0}
          <div class="undiscovered-hint">
            <span>No stations discovered yet</span>
          </div>
        {:else}
          {#each discoveredStations as s (s.navTag)}
            {@const pct = getYPercent(s.y)}
            {@const isTarget = navigating && navTagId === s.navTag}
            {@const isArrived = atTag === s.navTag}

            <div
              class="station-anchor"
              style="left: {pct}%;"
            >
              <!-- Pure black dot perfectly centered on the route line -->
              <div class="track-dot" class:is-target={isTarget} class:is-arrived={isArrived}></div>

              <!-- Clickable Station Label directly above the dot (Transparent background, no border, flat) -->
              <button
                type="button"
                class="station-label-btn"
                class:is-target={isTarget}
                class:is-arrived={isArrived}
                onclick={() => handleClickStation(s.navTag)}
                title="Click to navigate to {s.label} (#{s.navTag}, Y={s.y.toFixed(2)}m)"
              >
                <div class="station-tag-row">
                  <span class="station-tag">#{s.navTag}</span>
                  {#if s.stationType && s.stationType !== 'simple'}
                    <span class="station-type">{s.stationType.toUpperCase()}</span>
                  {/if}
                  {#if isArrived}
                    <span class="arrived-text">HERE</span>
                  {/if}
                </div>
                <div class="station-name">
                  {s.label}
                </div>
              </button>
            </div>
          {/each}
        {/if}
      </div>

      <!-- Simplified Robot Indicator (Pure flat neutral dot centered on the route line) -->
      {#if hasBase}
        <div
          class="robot-anchor"
          style="left: {robotPercent}%;"
          title="Robot Pose: X={robotX.toFixed(2)}m, Y={robotY.toFixed(2)}m"
        >
          <div class="robot-avatar">
            <svg viewBox="0 0 16 16" class="robot-svg">
              <circle cx="8" cy="8" r="5" class="chassis-dot" />
            </svg>
          </div>
        </div>
      {/if}
    </div>
  </div>
</div>

<style>
  .map-container {
    position: relative;
    width: 100%;
    height: 72px;
    background: transparent;
    border: none;
    overflow: hidden;
    user-select: none;
  }

  .map-scroll-area {
    position: absolute;
    inset: 0;
    overflow-x: auto;
    overflow-y: hidden;
    scrollbar-width: thin;
    scrollbar-color: #cbd5e1 transparent;
    cursor: grab;
    display: flex;
    align-items: center;
  }

  .map-scroll-area::-webkit-scrollbar {
    height: 5px;
    display: block;
  }

  .map-scroll-area::-webkit-scrollbar-track {
    background: transparent;
  }

  .map-scroll-area::-webkit-scrollbar-thumb {
    background: #cbd5e1;
    border-radius: 4px;
  }

  .map-scroll-area::-webkit-scrollbar-thumb:hover {
    background: #94a3b8;
  }

  .map-scroll-area:active {
    cursor: grabbing;
  }

  .map-track-content {
    position: relative;
    height: 100%;
    width: 100%;
    padding: 0 1.2rem;
    box-sizing: border-box;
    display: flex;
    align-items: center;
  }

  /* Central Guide Rail / Track Route */
  .track-rail {
    position: absolute;
    left: 0.8rem;
    right: 0.8rem;
    bottom: 14px;
    height: 2px;
    pointer-events: none;
  }

  .track-line {
    width: 100%;
    height: 100%;
    background: #94a3b8;
    border-radius: 1px;
  }

  /* Always-Visible Boundary Stop Straight Lines (Centered on route line, no tags on top) */
  .boundary-stop-line {
    position: absolute;
    bottom: 15px;
    transform: translate(-50%, 50%);
    display: flex;
    align-items: center;
    justify-content: center;
    pointer-events: none;
    z-index: 5;
  }

  .stop-bar {
    width: 2.5px;
    height: 20px;
    background: #ef4444;
    border-radius: 1px;
  }

  .stations-layer {
    position: absolute;
    inset: 0;
    pointer-events: none;
  }

  .undiscovered-hint {
    position: absolute;
    inset: 0;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 0.68rem;
    font-weight: 500;
    color: var(--muted, #94a3b8);
    pointer-events: none;
  }

  /* Station Node Anchor: Anchored exactly at the vertical centerline of the rail (bottom: 15px) */
  .station-anchor {
    position: absolute;
    bottom: 15px;
    transform: translateX(-50%);
    display: flex;
    flex-direction: column;
    align-items: center;
    pointer-events: none;
    z-index: 10;
  }

  /* Pure black place dot centered precisely on the 2px route rail */
  .track-dot {
    width: 6px;
    height: 6px;
    border-radius: 50%;
    background: #000000;
    position: absolute;
    bottom: -3px;
    left: 50%;
    transform: translateX(-50%);
    transition: all 0.12s ease;
  }

  .track-dot.is-target {
    background: #2563eb;
    transform: translateX(-50%) scale(1.4);
  }

  .track-dot.is-arrived {
    background: #10b981;
    transform: translateX(-50%) scale(1.4);
  }

  /* Station Label Button directly above the centered dot: Pure transparent background, no borders, flat */
  .station-label-btn {
    all: unset;
    position: absolute;
    bottom: 7px;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 0;
    padding: 0;
    margin: 0;
    min-height: 0;
    background: transparent !important;
    border: none !important;
    border-radius: 0 !important;
    outline: none !important;
    cursor: pointer;
    pointer-events: auto;
    white-space: nowrap;
    box-shadow: none !important;
    -webkit-box-shadow: none !important;
    transition: transform 0.1s ease, color 0.1s ease;
  }

  .station-label-btn:hover {
    transform: translateY(-1px);
    background: transparent !important;
    border: none !important;
    box-shadow: none !important;
    -webkit-box-shadow: none !important;
  }

  .station-label-btn:focus,
  .station-label-btn:focus-visible,
  .station-label-btn:active {
    background: transparent !important;
    border: none !important;
    box-shadow: none !important;
    -webkit-box-shadow: none !important;
    outline: none !important;
  }

  .station-label-btn:hover .station-name {
    color: #2563eb;
  }

  .station-tag-row {
    display: flex;
    align-items: center;
    gap: 2px;
    line-height: 1;
  }

  .station-tag {
    font-size: 0.52rem;
    font-weight: 800;
    color: #2563eb;
    letter-spacing: -0.02em;
    line-height: 1;
  }

  .station-label-btn.is-target .station-tag {
    color: #1d4ed8;
  }

  .station-label-btn.is-arrived .station-tag {
    color: #059669;
  }

  .station-type {
    font-size: 0.42rem;
    font-weight: 700;
    color: #64748b;
    line-height: 1;
    letter-spacing: 0.02em;
  }

  .station-name {
    font-size: 0.54rem;
    font-weight: 700;
    color: #0f172a;
    text-transform: uppercase;
    letter-spacing: 0.02em;
    line-height: 1.1;
    max-width: 58px;
    overflow: hidden;
    text-overflow: ellipsis;
    transition: color 0.1s ease;
  }

  .station-label-btn.is-target .station-name {
    color: #2563eb;
    font-weight: 800;
  }

  .station-label-btn.is-arrived .station-name {
    color: #059669;
    font-weight: 800;
  }

  .arrived-text {
    font-size: 0.44rem;
    font-weight: 800;
    color: #059669;
    letter-spacing: 0.04em;
    line-height: 1;
  }

  /* Simplified Live Robot Marker: Centered precisely on the route line */
  .robot-anchor {
    position: absolute;
    bottom: 15px;
    transform: translate(-50%, 50%);
    display: flex;
    align-items: center;
    justify-content: center;
    pointer-events: none;
    z-index: 20;
    transition: left 0.08s linear;
  }

  .robot-avatar {
    width: 10px;
    height: 10px;
    display: flex;
    align-items: center;
    justify-content: center;
  }

  .robot-svg {
    width: 100%;
    height: 100%;
  }

  .chassis-dot {
    fill: #0f172a;
  }
</style>
