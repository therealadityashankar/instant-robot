<script lang="ts">
  import { onMount } from 'svelte';
  import arucoTable from './aruco6x6_250.json';

  let mode = $state<'card' | 'single'>('card');

  // Card parameters (200 + 201 pair with label in between)
  let labelTitle = $state('DOCKING STATION');
  let labelDesc = $state('Return and recharge zone for LeKiwi');
  let cardMarkerMm = $state(30);

  // Single tag parameters
  let singleTagId = $state(42);
  let singleLabel = $state('APPLE');
  let singleMarkerMm = $state(40);
  let showArrow = $state(true);

  let canvasEl = $state<HTMLCanvasElement>();

  function drawArucoMarker(
    ctx: CanvasRenderingContext2D,
    tagId: number,
    x: number,
    y: number,
    sizePx: number,
    borderBits = 1
  ) {
    const bits = arucoTable[tagId % arucoTable.length];
    if (!bits) return;

    const totalCells = 6 + borderBits * 2; // 8x8 for borderBits=1
    const cellSize = sizePx / totalCells;

    // Outer black border
    ctx.fillStyle = '#000000';
    ctx.fillRect(x, y, sizePx, sizePx);

    // Inner 6x6 bit cells
    for (let r = 0; r < 6; r++) {
      for (let c = 0; c < 6; c++) {
        const bit = bits[r * 6 + c];
        if (bit === '1') {
          ctx.fillStyle = '#ffffff';
          ctx.fillRect(
            x + (c + borderBits) * cellSize,
            y + (r + borderBits) * cellSize,
            cellSize + 0.5,
            cellSize + 0.5
          );
        }
      }
    }
  }

  function renderPreview() {
    if (!canvasEl) return;
    const ctx = canvasEl.getContext('2d');
    if (!ctx) return;

    const scale = 2; // Retina / high-res preview

    if (mode === 'card') {
      // 200 (Left) + Text + 201 (Right)
      const markerPx = 160;
      const quietPx = 20;
      const textWidthPx = 360;
      const heightPx = markerPx + quietPx * 2;
      const widthPx = markerPx * 2 + quietPx * 4 + textWidthPx;

      canvasEl.width = widthPx * scale;
      canvasEl.height = heightPx * scale;
      ctx.setTransform(scale, 0, 0, scale, 0, 0);

      // Card background & rounded border
      ctx.fillStyle = '#ffffff';
      ctx.fillRect(0, 0, widthPx, heightPx);
      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 3;
      ctx.strokeRect(4, 4, widthPx - 8, heightPx - 8);

      // Left Marker: ID 200
      drawArucoMarker(ctx, 200, quietPx, quietPx, markerPx, 1);

      // Right Marker: ID 201
      drawArucoMarker(ctx, 201, widthPx - quietPx - markerPx, quietPx, markerPx, 1);

      // Text region between markers
      const textX = quietPx + markerPx + 20;
      const midY = heightPx / 2;

      // Divider line
      ctx.strokeStyle = '#cbd5e1';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(textX, midY);
      ctx.lineTo(textX + textWidthPx - 40, midY);
      ctx.stroke();

      // Top text: Main Label Title
      ctx.fillStyle = '#0f172a';
      ctx.font = 'bold 26px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(labelTitle.toUpperCase(), textX + (textWidthPx - 40) / 2, midY - 35);

      // Bottom text: Description
      ctx.fillStyle = '#475569';
      ctx.font = '15px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      ctx.fillText(labelDesc, textX + (textWidthPx - 40) / 2, midY + 35);

      // Tag ID labels in small text
      ctx.fillStyle = '#94a3b8';
      ctx.font = '10px monospace';
      ctx.fillText('ID 200 (NAV-L)', quietPx + markerPx / 2, heightPx - 6);
      ctx.fillText('ID 201 (NAV-R)', widthPx - quietPx - markerPx / 2, heightPx - 6);

    } else {
      // Single Item Marker
      const markerPx = 220;
      const padPx = 30;
      const widthPx = markerPx + padPx * 2;
      const heightPx = markerPx + padPx * 2 + 50;

      canvasEl.width = widthPx * scale;
      canvasEl.height = heightPx * scale;
      ctx.setTransform(scale, 0, 0, scale, 0, 0);

      // Background
      ctx.fillStyle = '#ffffff';
      ctx.fillRect(0, 0, widthPx, heightPx);
      ctx.strokeStyle = '#0284c7';
      ctx.lineWidth = 2;
      ctx.strokeRect(3, 3, widthPx - 6, heightPx - 6);

      // Marker
      drawArucoMarker(ctx, singleTagId, padPx, padPx, markerPx, 1);

      // Orientation arrow on top if enabled
      if (showArrow) {
        ctx.fillStyle = '#0284c7';
        ctx.beginPath();
        const ax = widthPx / 2;
        ctx.moveTo(ax, 8);
        ctx.lineTo(ax - 7, 20);
        ctx.lineTo(ax + 7, 20);
        ctx.closePath();
        ctx.fill();
      }

      // Bottom text: Label and ID
      ctx.fillStyle = '#0f172a';
      ctx.font = 'bold 20px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(singleLabel.toUpperCase(), widthPx / 2, heightPx - 30);

      ctx.fillStyle = '#64748b';
      ctx.font = '12px monospace';
      ctx.fillText(`DICT_6X6_250 · ID #${singleTagId} · ${singleMarkerMm}mm`, widthPx / 2, heightPx - 12);
    }
  }

  $effect(() => {
    // Re-render whenever parameters change
    void mode;
    void labelTitle;
    void labelDesc;
    void cardMarkerMm;
    void singleTagId;
    void singleLabel;
    void singleMarkerMm;
    void showArrow;
    renderPreview();
  });

  onMount(() => {
    renderPreview();
  });

  function downloadPng() {
    if (!canvasEl) return;
    const link = document.createElement('a');
    link.download = mode === 'card'
      ? `station_tag_${labelTitle.toLowerCase().replace(/\\s+/g, '_')}.png`
      : `tag_id_${singleTagId}_${singleLabel.toLowerCase().replace(/\\s+/g, '_')}.png`;
    link.href = canvasEl.toDataURL('image/png');
    link.click();
  }

  function printTag() {
    window.print();
  }
</script>

<div class="tag-generator-card">
  <div class="tab-header">
    <button class:active={mode === 'card'} onclick={() => (mode = 'card')}>
      🏷️ Station Nav Card (200 & 201)
    </button>
    <button class:active={mode === 'single'} onclick={() => (mode = 'single')}>
      🎯 Single Item / Prop Tag
    </button>
  </div>

  <div class="controls-grid">
    {#if mode === 'card'}
      <div class="field">
        <label for="labelTitle">Station Title (OCR Label)</label>
        <input id="labelTitle" type="text" bind:value={labelTitle} placeholder="e.g. APPLE STATION" />
      </div>
      <div class="field">
        <label for="labelDesc">Description (Gemini AI Context)</label>
        <input id="labelDesc" type="text" bind:value={labelDesc} placeholder="e.g. Red fruit pick zone" />
      </div>
      <div class="field">
        <label for="cardMarkerMm">Marker Size (mm)</label>
        <select id="cardMarkerMm" bind:value={cardMarkerMm}>
          <option value={25}>25 mm (Compact)</option>
          <option value={30}>30 mm (Standard)</option>
          <option value={40}>40 mm (Large)</option>
          <option value={50}>50 mm (Long Range)</option>
        </select>
      </div>
    {:else}
      <div class="field">
        <label for="singleTagId">Tag ID (0 – 249)</label>
        <input id="singleTagId" type="number" min="0" max="249" bind:value={singleTagId} />
      </div>
      <div class="field">
        <label for="singleLabel">Object Label Text</label>
        <input id="singleLabel" type="text" bind:value={singleLabel} placeholder="e.g. BANANA" />
      </div>
      <div class="field">
        <label for="singleMarkerMm">Physical Size (mm)</label>
        <select id="singleMarkerMm" bind:value={singleMarkerMm}>
          <option value={20}>20 mm (Tiny Prop)</option>
          <option value={30}>30 mm (Standard Block)</option>
          <option value={40}>40 mm (Standard Target)</option>
          <option value={60}>60 mm (Station Anchor)</option>
        </select>
      </div>
      <div class="field checkbox-field">
        <label>
          <input type="checkbox" bind:checked={showArrow} />
          Show Top Direction Arrow
        </label>
      </div>
    {/if}
  </div>

  <div class="preview-container">
    <canvas bind:this={canvasEl} class="tag-canvas"></canvas>
  </div>

  <div class="action-buttons">
    <button class="primary-btn" onclick={downloadPng}>
      ⬇️ Download High-Res PNG
    </button>
    <button class="secondary-btn" onclick={printTag}>
      🖨️ Print Tag (100% Scale)
    </button>
  </div>
</div>

<style>
  .tag-generator-card {
    background: var(--sl-color-bg-inline-code, #1e293b1a);
    border: 1px solid var(--sl-color-hairline, #334155);
    border-radius: 12px;
    padding: 1.5rem;
    margin: 1.5rem 0;
  }
  .tab-header {
    display: flex;
    gap: 0.5rem;
    margin-bottom: 1.25rem;
    border-bottom: 1px solid var(--sl-color-hairline, #334155);
    padding-bottom: 0.75rem;
  }
  .tab-header button {
    background: transparent;
    border: none;
    color: var(--sl-color-text-soft, #94a3b8);
    font-size: 0.95rem;
    font-weight: 500;
    padding: 0.5rem 1rem;
    cursor: pointer;
    border-radius: 8px;
    transition: all 0.2s ease;
  }
  .tab-header button.active {
    background: var(--sl-color-accent-low, #064e3b);
    color: var(--sl-color-accent-high, #a7f3d0);
  }
  .controls-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
    gap: 1rem;
    margin-bottom: 1.5rem;
  }
  .field {
    display: flex;
    flex-direction: column;
    gap: 0.35rem;
  }
  .field label {
    font-size: 0.82rem;
    font-weight: 600;
    color: var(--sl-color-text-accent, #38bdf8);
  }
  .field input,
  .field select {
    background: var(--sl-color-bg-nav, #0f172a);
    border: 1px solid var(--sl-color-hairline, #334155);
    border-radius: 6px;
    color: var(--sl-color-text, #f8fafc);
    padding: 0.5rem 0.75rem;
    font-size: 0.9rem;
  }
  .checkbox-field {
    justify-content: center;
  }
  .checkbox-field label {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    cursor: pointer;
    color: var(--sl-color-text, #f8fafc);
    font-size: 0.88rem;
  }
  .preview-container {
    background: #ffffff;
    padding: 1.5rem;
    border-radius: 8px;
    display: flex;
    justify-content: center;
    align-items: center;
    margin-bottom: 1.25rem;
    overflow-x: auto;
    box-shadow: inset 0 2px 4px rgba(0, 0, 0, 0.06);
  }
  .tag-canvas {
    max-width: 100%;
    height: auto;
    display: block;
    box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
  }
  .action-buttons {
    display: flex;
    gap: 0.75rem;
    justify-content: flex-end;
    flex-wrap: wrap;
  }
  .primary-btn {
    background: var(--sl-color-accent, #10b981);
    color: #ffffff;
    border: none;
    padding: 0.6rem 1.2rem;
    border-radius: 6px;
    font-weight: 600;
    font-size: 0.9rem;
    cursor: pointer;
    transition: opacity 0.2s;
  }
  .secondary-btn {
    background: transparent;
    border: 1px solid var(--sl-color-hairline, #334155);
    color: var(--sl-color-text, #f8fafc);
    padding: 0.6rem 1.2rem;
    border-radius: 6px;
    font-weight: 500;
    font-size: 0.9rem;
    cursor: pointer;
  }
  .primary-btn:hover {
    opacity: 0.9;
  }
</style>
