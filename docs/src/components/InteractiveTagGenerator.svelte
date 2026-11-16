<script lang="ts">
  import { onMount } from 'svelte';
  import arucoTable from './aruco6x6_250.json';

  interface StationItem {
    id: number;
    title: string;
    description: string;
  }

  let mode = $state<'card' | 'boundary' | 'single' | 'calibration_line'>('card');

  // Fixed standard marker size
  const MARKER_MM = 30;

  // Multi-card station list
  let stations = $state<StationItem[]>([
    { id: 201, title: 'APPLE', description: 'Red fruit, pick it up' },
    { id: 202, title: 'BANANA', description: 'Yellow curved fruit' },
    { id: 203, title: 'ORANGE', description: 'Round citrus fruit' },
    { id: 204, title: 'BASKET', description: 'Drop zone for items' },
    { id: 205, title: 'BLOCK', description: 'Wooden jenga block' },
    { id: 206, title: 'BOTTLE', description: 'Water bottle, side tag' },
    { id: 207, title: 'PLANT', description: 'Potted plant, water me' },
  ]);

  let selectedIndex = $state(0);

  // Active card inputs (bound to selected station or single editor)
  let activeId = $state(201);
  let activeTitle = $state('APPLE');
  let activeDesc = $state('Red fruit, pick it up');

  // Boundary tag parameters
  let boundaryTagId = $state(240);
  let boundaryLabel = $state('RACK BOUNDARY / STOP');

  // Single tag parameters
  let singleTagId = $state(42);
  let singleLabel = $state('APPLE');
  let showArrow = $state(true);

  // Alignment strip parameters
  let calLineTagIds = $state<number[]>([201, 202, 203, 204]);
  let calLineDotOffsetMm = $state(25);
  let calLineTagSizeMm = $state(30);

  let canvasEl = $state<HTMLCanvasElement>();
  let isGeneratingPdf = $state(false);

  // Sync active station inputs when selectedIndex changes
  $effect(() => {
    if (stations[selectedIndex]) {
      activeId = stations[selectedIndex].id;
      activeTitle = stations[selectedIndex].title;
      activeDesc = stations[selectedIndex].description;
    }
  });

  function updateActiveStation() {
    if (stations[selectedIndex]) {
      stations[selectedIndex] = {
        id: Math.max(1, Math.min(499, Number(activeId) || 201)),
        title: (activeTitle || '').slice(0, 20),
        description: activeDesc || '',
      };
    }
  }

  function addStation() {
    const nextId = stations.length > 0 ? Math.max(...stations.map((s) => s.id)) + 1 : 201;
    stations = [
      ...stations,
      {
        id: nextId,
        title: `STATION ${nextId}`,
        description: 'Target location',
      },
    ];
    selectedIndex = stations.length - 1;
  }

  function removeStation(index: number) {
    if (stations.length <= 1) return;
    stations = stations.filter((_, i) => i !== index);
    if (selectedIndex >= stations.length) {
      selectedIndex = stations.length - 1;
    }
  }

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

  /**
   * Render a single station card onto an arbitrary canvas context.
   */
  function renderCardOnContext(
    ctx: CanvasRenderingContext2D,
    item: StationItem,
    widthPx: number,
    heightPx: number
  ) {
    // Background
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, widthPx, heightPx);

    // Outer card border
    ctx.strokeStyle = '#0f172a';
    ctx.lineWidth = 3;
    ctx.strokeRect(3, 3, widthPx - 6, heightPx - 6);

    const markerPx = 150;
    const quietPx = 18;
    const padTop = 18;

    // ── Left Marker: Unique Station ID ──
    const lx = quietPx + 10;
    const my = padTop;
    drawArucoMarker(ctx, item.id, lx, my, markerPx, 1);

    // ── Right Marker: Universal Anchor ID 200 ──
    const rx = widthPx - quietPx - 10 - markerPx;
    drawArucoMarker(ctx, 200, rx, my, markerPx, 1);

    // ── Prominent Left ID Display (Large, Bold, with generous padding) ──
    const idY = my + markerPx + 38;

    // Highlight pill under left unique marker
    ctx.fillStyle = '#f1f5f9';
    ctx.beginPath();
    ctx.roundRect(lx - 4, idY - 26, markerPx + 8, 38, 6);
    ctx.fill();
    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    ctx.fillStyle = '#0f172a';
    ctx.font = 'bold 24px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(`ID #${item.id}`, lx + markerPx / 2, idY - 7);

    // Right Anchor Tag ID
    ctx.fillStyle = '#64748b';
    ctx.font = 'bold 16px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace';
    ctx.fillText('ID #200 (END)', rx + markerPx / 2, idY - 7);

    // ── Text region between markers ──
    const textX = lx + markerPx + 22;
    const textWidthPx = rx - textX - 22;
    const textMidY = my + markerPx / 2;

    // Divider line
    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(textX, textMidY);
    ctx.lineTo(textX + textWidthPx, textMidY);
    ctx.stroke();

    // Top text: Main Label Title (Max 20 chars, bold, prominent)
    const cleanTitle = (item.title || '').slice(0, 20).toUpperCase();
    ctx.fillStyle = '#0f172a';
    ctx.font = 'bold 28px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(cleanTitle, textX + textWidthPx / 2, textMidY - 34);

    // Bottom text: Description
    ctx.fillStyle = '#475569';
    ctx.font = '15px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.fillText(item.description || '', textX + textWidthPx / 2, textMidY + 34);

    // Subtitle note
    ctx.fillStyle = '#94a3b8';
    ctx.font = '10px monospace';
    ctx.fillText(`DICT_6X6_250 · MARKER ${MARKER_MM}mm · SCALE 100%`, textX + textWidthPx / 2, heightPx - 14);
  }

  /**
   * Render a standardized boundary stop tag.
   * White cutout paper with red rounded border, red marker border, centered square ArUco tag, and red text:
   * "Okay, gotta stop here, explore the other side!"
   */
  function renderBoundaryCardOnContext(
    ctx: CanvasRenderingContext2D,
    tagId: number,
    label: string,
    widthPx: number,
    heightPx: number
  ) {
    void label;
    ctx.clearRect(0, 0, widthPx, heightPx);

    // White Card Background with Rounded Corners and Bold Red Border
    const cornerR = 14;
    ctx.beginPath();
    ctx.roundRect(2, 2, widthPx - 4, heightPx - 4, cornerR);
    ctx.fillStyle = '#ffffff';
    ctx.fill();
    ctx.strokeStyle = '#dc2626';
    ctx.lineWidth = 4;
    ctx.stroke();

    const markerPx = 116;
    const lx = (widthPx - markerPx) / 2;
    const my = 14;

    // Single standalone marker centered horizontally near the top (remains square!)
    drawArucoMarker(ctx, tagId, lx, my, markerPx, 1);

    // Bold red border around the ArUco marker
    ctx.strokeStyle = '#dc2626';
    ctx.lineWidth = 2.5;
    ctx.strokeRect(lx - 2, my - 2, markerPx + 4, markerPx + 4);

    // Crisp black text directly underneath the marker
    ctx.fillStyle = '#0f172a';
    ctx.font = 'bold 12px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('Okay, gotta stop here,', widthPx / 2, my + markerPx + 22);
    ctx.fillText('explore the other side!', widthPx / 2, my + markerPx + 38);
  }

  function renderCalibrationLineOnContext(
    ctx: CanvasRenderingContext2D,
    tagIds: number[],
    dotOffsetMm: number,
    tagSizeMm: number,
    widthPx: number,
    heightPx: number
  ) {
    // Crisp white background
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, widthPx, heightPx);

    // Border
    ctx.strokeStyle = '#0284c7';
    ctx.lineWidth = 2;
    ctx.strokeRect(3, 3, widthPx - 6, heightPx - 6);

    const centerY = heightPx / 2 + 10;

    // Header Title
    ctx.fillStyle = '#0f172a';
    ctx.font = 'bold 14px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText('STRAIGHT-LINE PICK & VISION CALIBRATION STRIP', 16, 12);

    ctx.fillStyle = '#475569';
    ctx.font = '10px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.fillText(
      `ArUco DICT_6X6_250 · Tag Size: ${tagSizeMm} mm · Target Dot: ${dotOffsetMm} mm to Left of Each Tag · Standard 1:1 Scale`,
      16,
      30
    );

    // 50mm Verification Ruler in top-right
    const rulerX = widthPx - 170;
    const rulerY = 22;
    ctx.fillStyle = '#0f172a';
    ctx.font = 'bold 9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('50 mm Scale Check', rulerX + 50, rulerY - 6);
    ctx.strokeStyle = '#0f172a';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(rulerX, rulerY);
    ctx.lineTo(rulerX + 100, rulerY);
    // Ticks
    ctx.moveTo(rulerX, rulerY - 4);
    ctx.lineTo(rulerX, rulerY + 4);
    ctx.moveTo(rulerX + 50, rulerY - 3);
    ctx.lineTo(rulerX + 50, rulerY + 3);
    ctx.moveTo(rulerX + 100, rulerY - 4);
    ctx.lineTo(rulerX + 100, rulerY + 4);
    ctx.stroke();

    // Continuous Horizontal Baseline through all tag centers and dots
    ctx.save();
    ctx.strokeStyle = 'rgba(2, 132, 199, 0.45)';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(12, centerY);
    ctx.lineTo(widthPx - 12, centerY);
    ctx.stroke();
    ctx.restore();

    // Render each (Dot + Tag) pair in a straight line
    const numTags = tagIds.length;
    const itemWidthPx = (widthPx - 40) / numTags;

    for (let i = 0; i < numTags; i++) {
      const tagId = tagIds[i];
      const slotCenterX = 20 + (i + 0.5) * itemWidthPx;

      const tagSizePx = tagSizeMm * 2.5;
      const dotOffsetPx = dotOffsetMm * 2.5;

      const tagCenterX = slotCenterX + 16;
      const dotCenterX = tagCenterX - dotOffsetPx;

      // 1. Target Dot (Horizontal Left of Tag, exact vertical center)
      ctx.save();
      // Outer red precision ring
      ctx.strokeStyle = '#dc2626';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(dotCenterX, centerY, 9, 0, Math.PI * 2);
      ctx.stroke();

      // Middle black ring
      ctx.strokeStyle = '#0f172a';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(dotCenterX, centerY, 4.5, 0, Math.PI * 2);
      ctx.stroke();

      // Center solid bullseye dot
      ctx.fillStyle = '#dc2626';
      ctx.beginPath();
      ctx.arc(dotCenterX, centerY, 2.2, 0, Math.PI * 2);
      ctx.fill();

      // Precision crosshairs
      ctx.strokeStyle = '#dc2626';
      ctx.lineWidth = 0.8;
      ctx.beginPath();
      ctx.moveTo(dotCenterX - 13, centerY);
      ctx.lineTo(dotCenterX + 13, centerY);
      ctx.moveTo(dotCenterX, centerY - 13);
      ctx.lineTo(dotCenterX, centerY + 13);
      ctx.stroke();

      // Dot label
      ctx.fillStyle = '#dc2626';
      ctx.font = 'bold 8px monospace';
      ctx.textAlign = 'center';
      ctx.fillText('TARGET DOT', dotCenterX, centerY + 18);
      ctx.fillText('(Left Finger)', dotCenterX, centerY + 27);
      ctx.restore();

      // 2. Dimension leader between dot and tag
      ctx.save();
      ctx.strokeStyle = '#94a3b8';
      ctx.lineWidth = 1;
      ctx.setLineDash([2, 2]);
      ctx.beginPath();
      ctx.moveTo(dotCenterX, centerY - 20);
      ctx.lineTo(tagCenterX, centerY - 20);
      ctx.stroke();

      ctx.fillStyle = '#64748b';
      ctx.font = '8px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`${dotOffsetMm}mm`, (dotCenterX + tagCenterX) / 2, centerY - 23);
      ctx.restore();

      // 3. ArUco Marker
      const tagLeft = tagCenterX - tagSizePx / 2;
      const tagTop = centerY - tagSizePx / 2;
      drawArucoMarker(ctx, tagId, tagLeft, tagTop, tagSizePx, 1);

      // Tag ID Label
      ctx.fillStyle = '#0f172a';
      ctx.font = 'bold 11px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`ID #${tagId}`, tagCenterX, tagTop + tagSizePx + 15);

      // Orientation arrow on top
      ctx.fillStyle = '#0284c7';
      ctx.beginPath();
      ctx.moveTo(tagCenterX, tagTop - 6);
      ctx.lineTo(tagCenterX - 5, tagTop - 1);
      ctx.lineTo(tagCenterX + 5, tagTop - 1);
      ctx.closePath();
      ctx.fill();
    }

    // Bottom user instructions
    ctx.fillStyle = '#64748b';
    ctx.font = '9px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(
      'Place robot in front of strip → Camera detects tag → Manually jog left gripper finger over target dot → Save pick offset',
      widthPx / 2,
      heightPx - 10
    );
  }

  function renderPreview() {
    if (!canvasEl) return;
    const ctx = canvasEl.getContext('2d');
    if (!ctx) return;

    const scale = 2; // Retina preview

    if (mode === 'card') {
      const widthPx = 700;
      const heightPx = 250;

      canvasEl.width = widthPx * scale;
      canvasEl.height = heightPx * scale;
      ctx.setTransform(scale, 0, 0, scale, 0, 0);

      const activeItem: StationItem = {
        id: Math.max(1, Math.min(499, Number(activeId) || 201)),
        title: (activeTitle || '').slice(0, 20),
        description: activeDesc || '',
      };

      renderCardOnContext(ctx, activeItem, widthPx, heightPx);
    } else if (mode === 'boundary') {
      const widthPx = 184;
      const heightPx = 224;

      canvasEl.width = widthPx * scale;
      canvasEl.height = heightPx * scale;
      ctx.setTransform(scale, 0, 0, scale, 0, 0);

      renderBoundaryCardOnContext(ctx, boundaryTagId, boundaryLabel, widthPx, heightPx);
    } else if (mode === 'calibration_line') {
      const widthPx = 760;
      const heightPx = 230;

      canvasEl.width = widthPx * scale;
      canvasEl.height = heightPx * scale;
      ctx.setTransform(scale, 0, 0, scale, 0, 0);

      renderCalibrationLineOnContext(
        ctx,
        calLineTagIds,
        calLineDotOffsetMm,
        calLineTagSizeMm,
        widthPx,
        heightPx
      );
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

      // ID text
      ctx.fillStyle = '#0f172a';
      ctx.font = 'bold 20px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(`ID #${singleTagId}  ·  ${singleLabel}`, widthPx / 2, heightPx - 26);
    }
  }

  $effect(() => {
    void mode;
    void activeId;
    void activeTitle;
    void activeDesc;
    void boundaryTagId;
    void boundaryLabel;
    void selectedIndex;
    void singleTagId;
    void singleLabel;
    void showArrow;
    renderPreview();
  });

  onMount(() => {
    renderPreview();
  });

  /**
   * Export the current view as a high-resolution PNG.
   */
  function downloadPng() {
    if (!canvasEl) return;
    const link = document.createElement('a');
    link.download =
      mode === 'card'
        ? `station_${activeId || 201}_card.png`
        : mode === 'boundary'
        ? `stop_stand_tag_${boundaryTagId}.png`
        : mode === 'calibration_line'
        ? `pick_calibration_strip.png`
        : `tag_${singleTagId}_${singleLabel.toLowerCase().replace(/[^a-z0-9]/g, '_')}.png`;
    link.href = canvasEl.toDataURL('image/png');
    link.click();
  }

  /**
   * Generate an A4 Multi-Card PDF containing all configured stations or boundary markers.
   */
  async function downloadMultiCardPdf() {
    isGeneratingPdf = true;
    try {
      const { jsPDF } = await import('jspdf');

      const a4WidthMm = 210;
      const a4HeightMm = 297;
      const marginMm = 15;

      // Offscreen canvas for rendering high-res cards (300 DPI)
      const offCanvas = document.createElement('canvas');
      const offCtx = offCanvas.getContext('2d')!;

      if (mode === 'calibration_line') {
        const stripPdf = new jsPDF({
          orientation: 'landscape',
          unit: 'mm',
          format: 'a4',
        });

        const landscapeWidthMm = 297;
        const landscapeHeightMm = 210;
        const stripWidthMm = 260;
        const stripHeightMm = 75;

        const widthPx = 760;
        const heightPx = 230;
        offCanvas.width = widthPx * 2;
        offCanvas.height = heightPx * 2;
        offCtx.setTransform(2, 0, 0, 2, 0, 0);

        stripPdf.setFont('helvetica', 'bold');
        stripPdf.setFontSize(12);
        stripPdf.setTextColor(30, 41, 59);
        stripPdf.text(
          'Instant Robot Pick & Vision Calibration Strip — Print at 100% (Actual Size)',
          18,
          16
        );

        offCtx.clearRect(0, 0, widthPx, heightPx);
        renderCalibrationLineOnContext(
          offCtx,
          calLineTagIds,
          calLineDotOffsetMm,
          calLineTagSizeMm,
          widthPx,
          heightPx
        );

        const imgData = offCanvas.toDataURL('image/png');
        const posX = (landscapeWidthMm - stripWidthMm) / 2;
        const posY = (landscapeHeightMm - stripHeightMm) / 2;

        stripPdf.addImage(imgData, 'PNG', posX, posY, stripWidthMm, stripHeightMm);

        stripPdf.setDrawColor(2, 132, 199);
        stripPdf.setLineDashPattern([2, 2], 0);
        stripPdf.rect(posX, posY, stripWidthMm, stripHeightMm);

        stripPdf.save('pick_calibration_strip.pdf');
        return;
      }

      if (mode === 'boundary') {
        const pdf = new jsPDF({
          orientation: 'portrait',
          unit: 'mm',
          format: 'a4',
        });
        // Stop Stand sheet: Print Start Stop Tag + End Stop Tag pair side-by-side with rounded margins
        const boundaryWidthMm = 46;
        const boundaryHeightMm = 56;
        const boundaryWidthPx = 184;
        const boundaryHeightPx = 224;

        offCanvas.width = boundaryWidthPx * 2;
        offCanvas.height = boundaryHeightPx * 2;
        offCtx.setTransform(2, 0, 0, 2, 0, 0);

        pdf.setFont('helvetica', 'bold');
        pdf.setFontSize(11);
        pdf.setTextColor(30, 41, 59);
        pdf.text(
          'Instant Robot Stop Stand Tags — DICT_6X6_250 — PRINT AT 100% SCALE',
          marginMm,
          marginMm - 4
        );

        const boundaryItems = [
          { tagId: boundaryTagId, label: 'START' },
          { tagId: boundaryTagId, label: 'END' },
        ];

        const gapXMm = 16;
        const startXMm = (a4WidthMm - (boundaryWidthMm * 2 + gapXMm)) / 2;
        const posYMm = marginMm + 10;

        for (let i = 0; i < boundaryItems.length; i++) {
          const item = boundaryItems[i];
          offCtx.clearRect(0, 0, boundaryWidthPx, boundaryHeightPx);
          renderBoundaryCardOnContext(offCtx, item.tagId, item.label, boundaryWidthPx, boundaryHeightPx);

          const imgData = offCanvas.toDataURL('image/png');
          const posX = startXMm + i * (boundaryWidthMm + gapXMm);

          pdf.addImage(imgData, 'PNG', posX, posYMm, boundaryWidthMm, boundaryHeightMm);

          pdf.setDrawColor(220, 38, 38);
          pdf.setLineDashPattern([2, 2], 0);
          pdf.roundedRect(posX, posYMm, boundaryWidthMm, boundaryHeightMm, 3.5, 3.5);
        }

        pdf.save(`stop_stand_tags_${boundaryTagId}.pdf`);
        return;
      }

      const pdf = new jsPDF({
        orientation: 'portrait',
        unit: 'mm',
        format: 'a4',
      });

      const cardWidthMm = 140;
      const cardHeightMm = 50;
      const gapYMm = 8;
      const cardsPerPage = Math.floor((a4HeightMm - 2 * marginMm - 10) / (cardHeightMm + gapYMm)); // 4 cards per page

      const widthPx = 700;
      const heightPx = 250;
      offCanvas.width = widthPx * 2;
      offCanvas.height = heightPx * 2;
      offCtx.scale(2, 2);

      const itemsToPrint = mode === 'card'
        ? stations
        : [{ id: singleTagId, title: singleLabel, description: 'Single prop tag' }];

      for (let i = 0; i < itemsToPrint.length; i++) {
        const pageIdx = Math.floor(i / cardsPerPage);
        const cardIdxOnPage = i % cardsPerPage;

        if (i > 0 && cardIdxOnPage === 0) {
          pdf.addPage('a4', 'portrait');
        }

        // Header title on each page
        if (cardIdxOnPage === 0) {
          pdf.setFont('helvetica', 'bold');
          pdf.setFontSize(11);
          pdf.setTextColor(30, 41, 59);
          pdf.text(
            'Instant Robot Navigation Cards — DICT_6X6_250 (30 mm markers) — PRINT AT 100% SCALE',
            marginMm,
            marginMm - 4
          );
        }

        const item = itemsToPrint[i];
        offCtx.clearRect(0, 0, widthPx, heightPx);
        renderCardOnContext(offCtx, item, widthPx, heightPx);

        const imgData = offCanvas.toDataURL('image/png');
        const posX = (a4WidthMm - cardWidthMm) / 2;
        const posY = marginMm + cardIdxOnPage * (cardHeightMm + gapYMm);

        pdf.addImage(imgData, 'PNG', posX, posY, cardWidthMm, cardHeightMm);

        // Dashed cutting guide
        pdf.setDrawColor(203, 213, 225);
        pdf.setLineDashPattern([2, 2], 0);
        pdf.rect(posX, posY, cardWidthMm, cardHeightMm);
      }

      pdf.save(mode === 'card' ? `instant_robot_station_cards.pdf` : `tag_${singleTagId}.pdf`);
    } catch (err) {
      console.error('PDF export failed:', err);
      alert('Failed to generate PDF. You can use the Print button to print the cards directly.');
    } finally {
      isGeneratingPdf = false;
    }
  }

  function printTag() {
    window.print();
  }
</script>

<div class="tag-generator-card">
  <div class="tab-header">
    <button class:active={mode === 'card'} onclick={() => (mode = 'card')}>
      🏷️ Station Cards (Dual Tag + OCR)
    </button>
    <button class:active={mode === 'boundary'} onclick={() => (mode = 'boundary')}>
      🛑 Rack Boundary Tag (Tag 240)
    </button>
    <button class:active={mode === 'calibration_line'} onclick={() => (mode = 'calibration_line')}>
      📏 Alignment Strip (Straight Line + Dots)
    </button>
    <button class:active={mode === 'single'} onclick={() => (mode = 'single')}>
      🎯 Single Item / Prop Tag
    </button>
  </div>

  {#if mode === 'card'}
    <!-- Multi-Station Management Queue -->
    <div class="station-queue-section">
      <div class="queue-header">
        <span class="queue-title">📋 Station Cards in PDF Sheet ({stations.length})</span>
        <button class="add-station-btn" onclick={addStation}>+ Add Another Station</button>
      </div>

      <div class="station-chips-row">
        {#each stations as st, i}
          <div
            class="station-chip"
            class:active={selectedIndex === i}
            onclick={() => (selectedIndex = i)}
            role="button"
            tabindex="0"
            onkeydown={(e) => e.key === 'Enter' && (selectedIndex = i)}
          >
            <span class="chip-badge">ID #{st.id}</span>
            <span class="chip-label">{st.title || '(No Title)'}</span>
            {#if stations.length > 1}
              <button
                class="chip-remove-btn"
                title="Remove station"
                onclick={(e) => {
                  e.stopPropagation();
                  removeStation(i);
                }}
              >
                ✕
              </button>
            {/if}
          </div>
        {/each}
      </div>
    </div>

    <!-- Active Card Editor -->
    <div class="controls-grid">
      <div class="field">
        <label for="activeId">Unique Station Tag ID (Left Marker)</label>
        <input
          id="activeId"
          type="number"
          min="1"
          max="499"
          bind:value={activeId}
          oninput={updateActiveStation}
        />
        <span class="field-hint">Unique left marker ID (e.g. 201..499). Right marker is anchor #200.</span>
      </div>

      <div class="field">
        <div class="label-row">
          <label for="activeTitle">Station Title (OCR Label)</label>
          <span class="char-count" class:limit={activeTitle.length >= 20}>
            {activeTitle.length}/20 chars
          </span>
        </div>
        <input
          id="activeTitle"
          type="text"
          maxlength="20"
          bind:value={activeTitle}
          placeholder="e.g. APPLE STATION"
          oninput={updateActiveStation}
        />
        <span class="field-hint">Max 20 uppercase characters for high-accuracy OCR.</span>
      </div>

      <div class="field">
        <label for="activeDesc">Description (Gemini AI Context)</label>
        <input
          id="activeDesc"
          type="text"
          bind:value={activeDesc}
          placeholder="e.g. Red fruit pick zone"
          oninput={updateActiveStation}
        />
        <span class="field-hint">Context description used for semantic reasoning.</span>
      </div>

      <div class="field">
        <label for="markerSizeDisplay">Marker Size (Standard)</label>
        <div id="markerSizeDisplay" class="static-badge">
          <span>📏 {MARKER_MM} mm (Fixed Standard)</span>
        </div>
        <span class="field-hint">Locked to 30 mm standard for calibrated visual servoing.</span>
      </div>
    </div>
  {:else if mode === 'boundary'}
    <div class="boundary-info-callout">
      <div class="callout-icon">🛑</div>
      <div class="callout-text">
        <strong>Stop Stand Tag with Exploration Reminder</strong>
        <p>
          Stick one rounded vertical stop tag at the <strong>start</strong> and one at the <strong>end</strong> of your linear station rack.
          Features ArUco Tag #240 with <em>"Okay, gotta stop here, explore the other side!"</em> printed underneath.
        </p>
      </div>
    </div>

    <div class="controls-grid">
      <div class="field">
        <label for="boundaryTagId">Boundary Tag ID (Standard #240)</label>
        <input
          id="boundaryTagId"
          type="number"
          min="200"
          max="249"
          bind:value={boundaryTagId}
        />
        <span class="field-hint">Default 240. Used by robot to mark exploration start/stop limits.</span>
      </div>

      <div class="field">
        <label for="boundarySizeDisplay">Marker Size (Standard)</label>
        <div id="boundarySizeDisplay" class="static-badge">
          <span>📏 {MARKER_MM} mm (Fixed Standard)</span>
        </div>
        <span class="field-hint">Calibrated ArUco 6×6 fiducial size.</span>
      </div>
    </div>
  {:else if mode === 'calibration_line'}
    <div class="boundary-info-callout" style="border-left-color: #0284c7; background: rgba(2, 132, 199, 0.08);">
      <div class="callout-icon">📏</div>
      <div class="callout-text">
        <strong>Straight-Line Pick Offset Calibration Strip</strong>
        <p>
          Prints 4 ArUco tags in a continuous straight line, with high-contrast target dots located horizontally to the left of each tag.
          Use this sheet with the <strong>Pick Offset</strong> calibration system to calibrate the robot's physical gripper alignment.
        </p>
      </div>
    </div>

    <div class="controls-grid">
      <div class="field">
        <label for="dotOffsetMm">Target Dot Offset (mm Left)</label>
        <input
          id="dotOffsetMm"
          type="number"
          min="15"
          max="40"
          step="1"
          bind:value={calLineDotOffsetMm}
        />
        <span class="field-hint">Horizontal distance between marker center and alignment dot (Default: 25mm).</span>
      </div>

      <div class="field">
        <label for="calTagSize">Marker Size (mm)</label>
        <input
          id="calTagSize"
          type="number"
          min="20"
          max="45"
          step="5"
          bind:value={calLineTagSizeMm}
        />
        <span class="field-hint">Physical size of each ArUco marker (Default: 30mm).</span>
      </div>

      <div class="field">
        <label for="calTagsDisplay">Strip Markers</label>
        <div id="calTagsDisplay" class="static-badge">
          <span>🏷️ Tags #{calLineTagIds.join(', #')}</span>
        </div>
        <span class="field-hint">4 sequential fiducials on a straight reference baseline.</span>
      </div>
    </div>
  {:else}
    <div class="controls-grid">
      <div class="field">
        <label for="singleTagId">Tag ID (0 – 249)</label>
        <input id="singleTagId" type="number" min="0" max="249" bind:value={singleTagId} />
      </div>
      <div class="field">
        <label for="singleLabel">Object Label Text</label>
        <input id="singleLabel" type="text" maxlength="20" bind:value={singleLabel} placeholder="e.g. BANANA" />
      </div>
      <div class="field">
        <label for="singleSizeDisplay">Marker Size</label>
        <div id="singleSizeDisplay" class="static-badge">
          <span>📏 {MARKER_MM} mm (Standard)</span>
        </div>
      </div>
      <div class="field checkbox-field">
        <label>
          <input type="checkbox" bind:checked={showArrow} />
          Show Top Direction Arrow
        </label>
      </div>
    </div>
  {/if}

  <!-- Live Retina Preview -->
  <div class="preview-section-header">
    <span>Live Card Preview (100% Scale Layout)</span>
  </div>
  <div class="preview-container">
    <canvas bind:this={canvasEl} class="tag-canvas"></canvas>
  </div>

  <!-- Action Buttons -->
  <div class="action-buttons">
    <button class="secondary-btn" onclick={downloadPng}>
      ⬇️ Download PNG
    </button>
    <button class="primary-btn" onclick={downloadMultiCardPdf} disabled={isGeneratingPdf}>
      {#if isGeneratingPdf}
        ⏳ Generating PDF...
      {:else}
        📄 Download Multi-Card PDF (A4 Sheet)
      {/if}
    </button>
    <button class="secondary-btn" onclick={printTag}>
      🖨️ Print (100% Scale)
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

  .boundary-info-callout {
    display: flex;
    align-items: flex-start;
    gap: 0.75rem;
    background: #450a0a33;
    border: 1px solid #ef444466;
    border-radius: 8px;
    padding: 0.85rem 1rem;
    margin-bottom: 1.25rem;
  }
  .boundary-info-callout .callout-icon {
    font-size: 1.5rem;
    line-height: 1;
  }
  .boundary-info-callout .callout-text strong {
    display: block;
    color: #fca5a5;
    font-size: 0.9rem;
    margin-bottom: 0.25rem;
  }
  .boundary-info-callout .callout-text p {
    margin: 0;
    font-size: 0.82rem;
    color: #fecaca;
    line-height: 1.4;
  }

  /* Multi-Station Queue */
  .station-queue-section {
    background: var(--sl-color-bg-nav, #0f172a80);
    border: 1px solid var(--sl-color-hairline, #334155);
    border-radius: 8px;
    padding: 0.85rem 1rem;
    margin-bottom: 1.25rem;
  }
  .queue-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 0.6rem;
  }
  .queue-title {
    font-size: 0.85rem;
    font-weight: 600;
    color: var(--sl-color-text, #f8fafc);
  }
  .add-station-btn {
    background: var(--sl-color-accent-low, #064e3b);
    color: var(--sl-color-accent-high, #a7f3d0);
    border: 1px solid var(--sl-color-accent, #10b981);
    font-size: 0.8rem;
    font-weight: 600;
    padding: 0.3rem 0.75rem;
    border-radius: 6px;
    cursor: pointer;
    transition: opacity 0.2s;
  }
  .add-station-btn:hover {
    opacity: 0.9;
  }
  .station-chips-row {
    display: flex;
    gap: 0.5rem;
    flex-wrap: wrap;
  }
  .station-chip {
    display: flex;
    align-items: center;
    gap: 0.4rem;
    background: var(--sl-color-bg, #0f172a);
    border: 1px solid var(--sl-color-hairline, #334155);
    border-radius: 6px;
    padding: 0.35rem 0.6rem;
    cursor: pointer;
    font-size: 0.82rem;
    color: var(--sl-color-text-soft, #94a3b8);
    transition: all 0.15s ease;
  }
  .station-chip:hover {
    border-color: var(--sl-color-accent, #10b981);
    color: var(--sl-color-text, #f8fafc);
  }
  .station-chip.active {
    background: var(--sl-color-accent-low, #064e3b);
    border-color: var(--sl-color-accent, #10b981);
    color: #ffffff;
    font-weight: 600;
  }
  .chip-badge {
    background: var(--sl-color-accent, #10b981);
    color: #ffffff;
    padding: 0.1rem 0.35rem;
    border-radius: 4px;
    font-size: 0.75rem;
    font-family: monospace;
    font-weight: bold;
  }
  .chip-label {
    max-width: 120px;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .chip-remove-btn {
    background: transparent;
    border: none;
    color: #ef4444;
    cursor: pointer;
    padding: 0 0.2rem;
    font-size: 0.75rem;
  }
  .chip-remove-btn:hover {
    color: #f87171;
  }

  /* Controls Grid */
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
  .label-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
  }
  .field label {
    font-size: 0.82rem;
    font-weight: 600;
    color: var(--sl-color-text-accent, #38bdf8);
  }
  .char-count {
    font-size: 0.75rem;
    color: var(--sl-color-text-soft, #94a3b8);
    font-family: monospace;
  }
  .char-count.limit {
    color: #fbbf24;
    font-weight: bold;
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
  .field-hint {
    font-size: 0.72rem;
    color: var(--sl-color-text-soft, #64748b);
  }
  .static-badge {
    background: var(--sl-color-bg-nav, #0f172a);
    border: 1px solid var(--sl-color-hairline, #334155);
    border-radius: 6px;
    padding: 0.5rem 0.75rem;
    font-size: 0.88rem;
    color: var(--sl-color-text, #f8fafc);
    font-weight: 500;
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

  .preview-section-header {
    font-size: 0.8rem;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    color: var(--sl-color-text-soft, #94a3b8);
    margin-bottom: 0.4rem;
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
  .primary-btn:disabled {
    opacity: 0.6;
    cursor: not-allowed;
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
  .primary-btn:hover:not(:disabled) {
    opacity: 0.9;
  }
</style>
