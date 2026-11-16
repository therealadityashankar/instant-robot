<script lang="ts">
  import { onMount } from 'svelte';
  import {
    type GoalQuadConfig,
    type CornerPoint,
    DEFAULT_GOAL_QUAD,
    DEFAULT_GOAL_QUAD_SIM,
    DEFAULT_GOAL_QUAD_REAL,
    loadGoalQuad,
    saveGoalQuad,
    computeQuadCenter,
    computeQuadSideLengths,
    computeQuadAvgSize,
    computeQuadOrientationDeg,
    translateQuad,
    scaleQuad,
    rotateQuad,
    evaluateTolerance,
  } from './lib/pickCalibration';
  import { pickCalState } from './lib/pickCalState.svelte';

  let { defaultMode = 'sim' }: { defaultMode?: 'sim' | 'real' } = $props();
  let calMode = $state<'sim' | 'real'>(defaultMode);
  let config = $state<GoalQuadConfig>(loadGoalQuad(calMode));

  // Reactive corners: [P0 (TL), P1 (TR), P2 (BR), P3 (BL)]
  let p0 = $state<CornerPoint>([config.corners[0][0], config.corners[0][1]]);
  let p1 = $state<CornerPoint>([config.corners[1][0], config.corners[1][1]]);
  let p2 = $state<CornerPoint>([config.corners[2][0], config.corners[2][1]]);
  let p3 = $state<CornerPoint>([config.corners[3][0], config.corners[3][1]]);
  let targetTagId = $state<number>(config.targetTagId ?? 201);
  let downDistanceMm = $state<number>(15);
  let pickDownDistanceMm = $state<number>(20);
  let stepDelayMs = $state<number>(450);
  let stepSpeedGain = $state<number>(0.5);
  let tolerancePx = $state<number>(config.tolerancePx ?? 14);
  let downStepMm = $state<number>(config.downStepMm ?? 6);

  function setCalMode(mode: 'sim' | 'real') {
    calMode = mode;
    const newCfg = loadGoalQuad(mode);
    config = newCfg;
    p0 = [newCfg.corners[0][0], newCfg.corners[0][1]];
    p1 = [newCfg.corners[1][0], newCfg.corners[1][1]];
    p2 = [newCfg.corners[2][0], newCfg.corners[2][1]];
    p3 = [newCfg.corners[3][0], newCfg.corners[3][1]];
    targetTagId = newCfg.targetTagId ?? 201;
    tolerancePx = newCfg.tolerancePx ?? 14;
    downStepMm = newCfg.downStepMm ?? 6;
    saveMsg = `Switched to ${mode === 'real' ? 'Physical Robot (Real)' : 'Simulation (Sim)'} goal quad config`;
    setTimeout(() => (saveMsg = null), 2500);
  }

  // Derived quad metrics
  let corners = $derived<[CornerPoint, CornerPoint, CornerPoint, CornerPoint]>([
    [p0[0], p0[1]],
    [p1[0], p1[1]],
    [p2[0], p2[1]],
    [p3[0], p3[1]],
  ]);

  let center = $derived(computeQuadCenter(corners));
  let sideLengths = $derived(computeQuadSideLengths(corners)); // [Top, Right, Bottom, Left]
  let avgSize = $derived(computeQuadAvgSize(corners));
  let orientationDeg = $derived(computeQuadOrientationDeg(corners));

  let canvasEl: HTMLCanvasElement;
  let animId: number = 0;
  let saveMsg = $state<string | null>(null);

  // Canvas interaction & dragging state
  let isDragging = $state(false);
  let draggedCornerIdx: number | null = null; // 0, 1, 2, 3 or null
  let isTranslating = false;
  let dragStartMouse = { x: 0, y: 0 };
  let dragStartCorners: [CornerPoint, CornerPoint, CornerPoint, CornerPoint] = [
    [0, 0], [0, 0], [0, 0], [0, 0],
  ];
  let hoveredCornerIdx = $state<number | null>(null);

  // Live detection & matching telemetry
  let tagFound = $state<boolean>(false);
  let tagCenter = $state<{ x: number; y: number } | null>(null);
  let tagSideSize = $state<number>(0);
  let tagYaw = $state<number>(0);

  let centerErrPx = $state<number>(0);
  let sizeErrPx = $state<number>(0);
  let yawErrDeg = $state<number>(0);
  let meanCornerErrPx = $state<number>(0);
  let isAligned = $state<boolean>(false);
  let isNear = $state<boolean>(false);

  function syncConfig() {
    config = {
      corners: [
        [Math.round(p0[0]), Math.round(p0[1])],
        [Math.round(p1[0]), Math.round(p1[1])],
        [Math.round(p2[0]), Math.round(p2[1])],
        [Math.round(p3[0]), Math.round(p3[1])],
      ],
      targetTagId,
      tolerancePx,
      downStepMm,
      savedAt: new Date().toISOString(),
    };
    saveGoalQuad(config, calMode);
  }

  function setCorners(newCorners: [CornerPoint, CornerPoint, CornerPoint, CornerPoint]) {
    p0 = [newCorners[0][0], newCorners[0][1]];
    p1 = [newCorners[1][0], newCorners[1][1]];
    p2 = [newCorners[2][0], newCorners[2][1]];
    p3 = [newCorners[3][0], newCorners[3][1]];
    syncConfig();
  }

  function matchCurrentDetectedTag() {
    const cornersMap = pickCalState.detectedCorners;
    const detected = cornersMap.get(targetTagId) ?? (cornersMap.size > 0 ? cornersMap.values().next().value : null);
    if (!detected || detected.length < 4) {
      saveMsg = `Tag #${targetTagId} not detected in camera view yet.`;
      setTimeout(() => (saveMsg = null), 3500);
      return;
    }

    setCorners([
      [Math.round(detected[0][0]), Math.round(detected[0][1])],
      [Math.round(detected[1][0]), Math.round(detected[1][1])],
      [Math.round(detected[2][0]), Math.round(detected[2][1])],
      [Math.round(detected[3][0]), Math.round(detected[3][1])],
    ]);
    saveMsg = `Goal quadrilateral snapped to live Tag #${targetTagId} corners!`;
    setTimeout(() => (saveMsg = null), 3500);
  }

  function resetToNominalRight() {
    const defaultQuad = calMode === 'real' ? DEFAULT_GOAL_QUAD_REAL : DEFAULT_GOAL_QUAD_SIM;
    setCorners([
      [defaultQuad.corners[0][0], defaultQuad.corners[0][1]],
      [defaultQuad.corners[1][0], defaultQuad.corners[1][1]],
      [defaultQuad.corners[2][0], defaultQuad.corners[2][1]],
      [defaultQuad.corners[3][0], defaultQuad.corners[3][1]],
    ]);
    targetTagId = defaultQuad.targetTagId;
    saveMsg = `Reset goal quadrilateral to ${calMode === 'real' ? 'Real' : 'Sim'} default position`;
    setTimeout(() => (saveMsg = null), 3500);
  }

  function resetToRectangle() {
    const [cx, cy] = center;
    const half = 45;
    setCorners([
      [cx - half, cy - half],
      [cx + half, cy - half],
      [cx + half, cy + half],
      [cx - half, cy + half],
    ]);
    saveMsg = 'Reset goal shape to a symmetric rectangle';
    setTimeout(() => (saveMsg = null), 3500);
  }

  function handlePan(dx: number, dy: number) {
    setCorners(translateQuad(corners, dx, dy));
  }

  function handleScale(factor: number) {
    setCorners(scaleQuad(corners, factor));
  }

  function handleRotate(angleDeg: number) {
    setCorners(rotateQuad(corners, angleDeg));
  }

  function handleSave() {
    syncConfig();
    saveMsg = `Saved ${calMode === 'real' ? 'Physical Robot (Real)' : 'Simulation (Sim)'} goal quad configuration!`;
    setTimeout(() => (saveMsg = null), 3500);
  }

  // Point in polygon helper to detect clicks inside the quadrilateral
  function isPointInsideQuad(x: number, y: number, pts: [CornerPoint, CornerPoint, CornerPoint, CornerPoint]): boolean {
    let inside = false;
    for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
      const xi = pts[i][0], yi = pts[i][1];
      const xj = pts[j][0], yj = pts[j][1];
      const intersect = yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi;
      if (intersect) inside = !inside;
    }
    return inside;
  }

  function getCanvasCoords(e: MouseEvent): { x: number; y: number } {
    const rect = canvasEl.getBoundingClientRect();
    const scaleX = canvasEl.width / rect.width;
    const scaleY = canvasEl.height / rect.height;
    return {
      x: (e.clientX - rect.left) * scaleX,
      y: (e.clientY - rect.top) * scaleY,
    };
  }

  function onMouseDown(e: MouseEvent) {
    const { x, y } = getCanvasCoords(e);
    dragStartMouse = { x, y };
    dragStartCorners = [
      [p0[0], p0[1]],
      [p1[0], p1[1]],
      [p2[0], p2[1]],
      [p3[0], p3[1]],
    ];

    // 1. Check if clicking near a corner vertex (threshold 16px)
    const HIT_RADIUS = 16;
    for (let i = 0; i < 4; i++) {
      const [cx, cy] = dragStartCorners[i];
      if (Math.hypot(x - cx, y - cy) <= HIT_RADIUS) {
        isDragging = true;
        draggedCornerIdx = i;
        isTranslating = false;
        return;
      }
    }

    // 2. Check if clicking inside the quadrilateral to pan
    if (isPointInsideQuad(x, y, dragStartCorners)) {
      isDragging = true;
      draggedCornerIdx = null;
      isTranslating = true;
      return;
    }
  }

  function onMouseMove(e: MouseEvent) {
    const { x, y } = getCanvasCoords(e);

    if (!isDragging) {
      // Check hover for cursor changes
      let hovered: number | null = null;
      for (let i = 0; i < 4; i++) {
        const [cx, cy] = corners[i];
        if (Math.hypot(x - cx, y - cy) <= 16) {
          hovered = i;
          break;
        }
      }
      hoveredCornerIdx = hovered;
      return;
    }

    const dx = x - dragStartMouse.x;
    const dy = y - dragStartMouse.y;

    if (draggedCornerIdx !== null) {
      // Move individual corner
      const base = dragStartCorners[draggedCornerIdx];
      const newX = Math.max(10, Math.min(630, base[0] + dx));
      const newY = Math.max(10, Math.min(470, base[1] + dy));

      if (draggedCornerIdx === 0) p0 = [newX, newY];
      else if (draggedCornerIdx === 1) p1 = [newX, newY];
      else if (draggedCornerIdx === 2) p2 = [newX, newY];
      else if (draggedCornerIdx === 3) p3 = [newX, newY];
      syncConfig();
    } else if (isTranslating) {
      // Pan entire quadrilateral
      const newCorners = translateQuad(dragStartCorners, dx, dy);
      // Ensure all corners remain within canvas bounds
      const minX = Math.min(...newCorners.map((c) => c[0]));
      const maxX = Math.max(...newCorners.map((c) => c[0]));
      const minY = Math.min(...newCorners.map((c) => c[1]));
      const maxY = Math.max(...newCorners.map((c) => c[1]));

      if (minX >= 5 && maxX <= 635 && minY >= 5 && maxY <= 475) {
        setCorners(newCorners);
      }
    }
  }

  function onMouseUp() {
    isDragging = false;
    draggedCornerIdx = null;
    isTranslating = false;
  }

  onMount(() => {
    if (!canvasEl) return;
    const ctx = canvasEl.getContext('2d');
    if (!ctx) return;

    function renderLoop() {
      animId = requestAnimationFrame(renderLoop);

      const W = canvasEl.width;
      const H = canvasEl.height;

      // 1. Draw Live Camera Stream
      const camSrc = pickCalState.cameraSource;
      if (camSrc) {
        try {
          ctx.drawImage(camSrc, 0, 0, W, H);
        } catch {
          ctx.fillStyle = '#090d16';
          ctx.fillRect(0, 0, W, H);
        }
      } else {
        ctx.fillStyle = '#090d16';
        ctx.fillRect(0, 0, W, H);

        ctx.strokeStyle = 'rgba(56, 189, 248, 0.08)';
        ctx.lineWidth = 1;
        for (let x = 0; x < W; x += 40) {
          ctx.beginPath();
          ctx.moveTo(x, 0);
          ctx.lineTo(x, H);
          ctx.stroke();
        }
        for (let y = 0; y < H; y += 40) {
          ctx.beginPath();
          ctx.moveTo(0, y);
          ctx.lineTo(W, y);
          ctx.stroke();
        }

        ctx.fillStyle = '#64748b';
        ctx.font = '13px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('Arm camera stream waiting or simulator active...', W / 2, H / 2 - 10);
      }

      // 2. Draw Generic Goal Quadrilateral & Tolerance Error Region
      const currentCorners = corners;

      // Draw Tolerance Error Region (Thick translucent halo / error corridor around goal quad)
      ctx.save();
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      ctx.strokeStyle = isAligned ? 'rgba(34, 197, 94, 0.32)' : 'rgba(245, 158, 11, 0.22)';
      ctx.lineWidth = tolerancePx * 2;
      ctx.beginPath();
      ctx.moveTo(currentCorners[0][0], currentCorners[0][1]);
      ctx.lineTo(currentCorners[1][0], currentCorners[1][1]);
      ctx.lineTo(currentCorners[2][0], currentCorners[2][1]);
      ctx.lineTo(currentCorners[3][0], currentCorners[3][1]);
      ctx.closePath();
      ctx.stroke();

      // Corner tolerance bubbles
      for (let i = 0; i < 4; i++) {
        const [px, py] = currentCorners[i];
        ctx.fillStyle = isAligned ? 'rgba(34, 197, 94, 0.15)' : 'rgba(245, 158, 11, 0.12)';
        ctx.strokeStyle = isAligned ? 'rgba(34, 197, 94, 0.6)' : 'rgba(245, 158, 11, 0.45)';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(px, py, tolerancePx, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      }
      ctx.restore();

      // Goal Quad Inner Fill & Border
      ctx.save();
      ctx.beginPath();
      ctx.moveTo(currentCorners[0][0], currentCorners[0][1]);
      ctx.lineTo(currentCorners[1][0], currentCorners[1][1]);
      ctx.lineTo(currentCorners[2][0], currentCorners[2][1]);
      ctx.lineTo(currentCorners[3][0], currentCorners[3][1]);
      ctx.closePath();

      // Translucent fill
      ctx.fillStyle = isAligned ? 'rgba(34, 197, 94, 0.22)' : isTranslating ? 'rgba(245, 158, 11, 0.28)' : 'rgba(245, 158, 11, 0.16)';
      ctx.fill();

      // Glowing border
      ctx.strokeStyle = isAligned ? '#22c55e' : '#f59e0b';
      ctx.lineWidth = 2.5;
      ctx.shadowColor = isAligned ? '#22c55e' : '#f59e0b';
      ctx.shadowBlur = isAligned ? 12 : 8;
      ctx.stroke();
      ctx.restore();

      // Draw Corner Vertices & Labels
      const cornerNames = ['P0 (TL)', 'P1 (TR)', 'P2 (BR)', 'P3 (BL)'];
      for (let i = 0; i < 4; i++) {
        const [px, py] = currentCorners[i];
        const isHovered = hoveredCornerIdx === i || draggedCornerIdx === i;

        ctx.save();
        if (isHovered) {
          ctx.strokeStyle = '#ffffff';
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.arc(px, py, 9, 0, Math.PI * 2);
          ctx.stroke();
        }

        ctx.fillStyle = isHovered ? '#ffffff' : '#f59e0b';
        ctx.shadowColor = '#f59e0b';
        ctx.shadowBlur = isHovered ? 12 : 6;
        ctx.beginPath();
        ctx.arc(px, py, isHovered ? 6 : 4.5, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 10px monospace';
        ctx.shadowBlur = 0;
        ctx.textAlign = i === 0 || i === 3 ? 'right' : 'left';
        const offX = i === 0 || i === 3 ? -10 : 10;
        const offY = i < 2 ? -6 : 14;
        ctx.fillText(cornerNames[i], px + offX, py + offY);
        ctx.restore();
      }

      // Draw Side Length Labels on the edges
      ctx.save();
      ctx.fillStyle = 'rgba(251, 191, 36, 0.9)';
      ctx.font = '10px monospace';
      ctx.textAlign = 'center';
      const midPoints = [
        [(currentCorners[0][0] + currentCorners[1][0]) / 2, (currentCorners[0][1] + currentCorners[1][1]) / 2 - 6], // Top
        [(currentCorners[1][0] + currentCorners[2][0]) / 2 + 16, (currentCorners[1][1] + currentCorners[2][1]) / 2], // Right
        [(currentCorners[2][0] + currentCorners[3][0]) / 2, (currentCorners[2][1] + currentCorners[3][1]) / 2 + 14], // Bottom
        [(currentCorners[3][0] + currentCorners[0][0]) / 2 - 16, (currentCorners[3][1] + currentCorners[0][1]) / 2], // Left
      ];
      ctx.fillText(`Top: ${sideLengths[0].toFixed(0)}px`, midPoints[0][0], midPoints[0][1]);
      ctx.fillText(`${sideLengths[1].toFixed(0)}px`, midPoints[1][0], midPoints[1][1]);
      ctx.fillText(`Bottom: ${sideLengths[2].toFixed(0)}px`, midPoints[2][0], midPoints[2][1]);
      ctx.fillText(`${sideLengths[3].toFixed(0)}px`, midPoints[3][0], midPoints[3][1]);
      ctx.restore();

      // Draw Centroid Target Crosshair & Goal Orientation Vector
      const [cx, cy] = center;
      ctx.save();
      ctx.strokeStyle = '#fbbf24';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(cx - 10, cy);
      ctx.lineTo(cx + 10, cy);
      ctx.moveTo(cx, cy - 10);
      ctx.lineTo(cx, cy + 10);
      ctx.stroke();

      // Goal Direction Arrow (Top edge heading vector)
      const goalRad = (orientationDeg * Math.PI) / 180;
      const goalArrLen = 32;
      const gEndX = cx + Math.cos(goalRad) * goalArrLen;
      const gEndY = cy + Math.sin(goalRad) * goalArrLen;
      ctx.strokeStyle = '#fbbf24';
      ctx.fillStyle = '#fbbf24';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(gEndX, gEndY);
      ctx.stroke();

      ctx.beginPath();
      ctx.moveTo(gEndX, gEndY);
      ctx.lineTo(gEndX - 7 * Math.cos(goalRad - Math.PI / 6), gEndY - 7 * Math.sin(goalRad - Math.PI / 6));
      ctx.lineTo(gEndX - 7 * Math.cos(goalRad + Math.PI / 6), gEndY - 7 * Math.sin(goalRad + Math.PI / 6));
      ctx.closePath();
      ctx.fill();

      ctx.fillStyle = '#fbbf24';
      ctx.font = 'bold 10px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`GOAL ${orientationDeg.toFixed(0)}°`, cx, cy - 16);
      ctx.restore();

      // 3. Observe & Draw Detected ArUco Tag (No solvePnP)
      const cornersMap = pickCalState.detectedCorners;
      const detectedTagCorners = cornersMap.get(targetTagId) ?? (cornersMap.size > 0 ? cornersMap.values().next().value : null);

      if (detectedTagCorners && detectedTagCorners.length >= 4) {
        tagFound = true;
        const [q0, q1, q2, q3] = detectedTagCorners;

        const tX = (q0[0] + q1[0] + q2[0] + q3[0]) / 4;
        const tY = (q0[1] + q1[1] + q2[1] + q3[1]) / 4;
        tagCenter = { x: tX, y: tY };

        const s0 = Math.hypot(q1[0] - q0[0], q1[1] - q0[1]);
        const s1 = Math.hypot(q2[0] - q1[0], q2[1] - q1[1]);
        const s2 = Math.hypot(q3[0] - q2[0], q3[1] - q2[1]);
        const s3 = Math.hypot(q0[0] - q3[0], q0[1] - q3[1]);
        const avgSide = (s0 + s1 + s2 + s3) / 4;
        tagSideSize = avgSide;

        const tagYawVal = (Math.atan2(q1[1] - q0[1], q1[0] - q0[0]) * 180) / Math.PI;
        tagYaw = tagYawVal;

        // Draw Detected Tag Polygon (Cyan)
        ctx.save();
        ctx.beginPath();
        ctx.moveTo(q0[0], q0[1]);
        ctx.lineTo(q1[0], q1[1]);
        ctx.lineTo(q2[0], q2[1]);
        ctx.lineTo(q3[0], q3[1]);
        ctx.closePath();

        ctx.fillStyle = isAligned ? 'rgba(34, 197, 94, 0.22)' : 'rgba(56, 189, 248, 0.18)';
        ctx.fill();

        ctx.strokeStyle = isAligned ? '#22c55e' : '#38bdf8';
        ctx.lineWidth = 2.5;
        ctx.shadowColor = isAligned ? '#22c55e' : '#38bdf8';
        ctx.shadowBlur = 8;
        ctx.stroke();

        // Highlight Top Edge (q0 -> q1) with thick luminous line
        ctx.beginPath();
        ctx.moveTo(q0[0], q0[1]);
        ctx.lineTo(q1[0], q1[1]);
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 3.5;
        ctx.shadowColor = '#38bdf8';
        ctx.shadowBlur = 10;
        ctx.stroke();
        ctx.restore();

        // Draw Direction Arrow from tag centroid
        const tagRad = (tagYawVal * Math.PI) / 180;
        const tagArrLen = 34;
        const tEndX = tX + Math.cos(tagRad) * tagArrLen;
        const tEndY = tY + Math.sin(tagRad) * tagArrLen;
        ctx.save();
        ctx.strokeStyle = '#38bdf8';
        ctx.fillStyle = '#38bdf8';
        ctx.lineWidth = 2.5;
        ctx.shadowColor = '#38bdf8';
        ctx.shadowBlur = 6;
        ctx.beginPath();
        ctx.moveTo(tX, tY);
        ctx.lineTo(tEndX, tEndY);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(tEndX, tEndY);
        ctx.lineTo(tEndX - 8 * Math.cos(tagRad - Math.PI / 6), tEndY - 8 * Math.sin(tagRad - Math.PI / 6));
        ctx.lineTo(tEndX - 8 * Math.cos(tagRad + Math.PI / 6), tEndY - 8 * Math.sin(tagRad + Math.PI / 6));
        ctx.closePath();
        ctx.fill();
        ctx.restore();

        // Draw Tag Corner Vertices (q0..q3) & Labels
        const tagCornerNames = ['q0', 'q1', 'q2', 'q3'];
        for (let i = 0; i < 4; i++) {
          const [qx, qy] = detectedTagCorners[i];
          ctx.save();
          ctx.fillStyle = i === 0 ? '#ffffff' : '#38bdf8';
          ctx.beginPath();
          ctx.arc(qx, qy, 4, 0, Math.PI * 2);
          ctx.fill();

          ctx.fillStyle = '#38bdf8';
          ctx.font = 'bold 9px monospace';
          ctx.textAlign = i === 0 || i === 3 ? 'right' : 'left';
          const offX = i === 0 || i === 3 ? -6 : 6;
          const offY = i < 2 ? -6 : 10;
          ctx.fillText(tagCornerNames[i], qx + offX, qy + offY);
          ctx.restore();
        }

        // Connecting dashed leader lines from detected corners to goal corners
        ctx.save();
        ctx.strokeStyle = isAligned ? 'rgba(34, 197, 94, 0.65)' : 'rgba(148, 163, 184, 0.65)';
        ctx.lineWidth = 1.2;
        ctx.setLineDash([4, 4]);
        let cornerSumDist = 0;
        for (let i = 0; i < 4; i++) {
          const [gx, gy] = currentCorners[i];
          const [tx, ty] = detectedTagCorners[i];
          ctx.beginPath();
          ctx.moveTo(tx, ty);
          ctx.lineTo(gx, gy);
          ctx.stroke();
          cornerSumDist += Math.hypot(gx - tx, gy - ty);
        }
        ctx.restore();

        // Error metrics
        centerErrPx = Math.hypot(tX - cx, tY - cy);
        sizeErrPx = avgSide - avgSize;
        let dYaw = tagYawVal - orientationDeg;
        while (dYaw > 180) dYaw -= 360;
        while (dYaw < -180) dYaw += 360;
        yawErrDeg = dYaw;
        meanCornerErrPx = cornerSumDist / 4;

        const evalResult = evaluateTolerance(currentCorners, detectedTagCorners as any, tolerancePx);
        isAligned = evalResult.isInside;
        isNear = !isAligned && evalResult.maxDist <= tolerancePx * 2.2;

        // Draw Detected Tag Center Marker & Title
        ctx.save();
        ctx.fillStyle = '#38bdf8';
        ctx.beginPath();
        ctx.arc(tX, tY, 5, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = '#f1f5f9';
        ctx.font = 'bold 11px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(`TAG #${targetTagId} (${tagYawVal.toFixed(0)}°)`, tX, tY + 18);
        ctx.restore();
      } else {
        tagFound = false;
        tagCenter = null;
        isAligned = false;
        isNear = false;
      }
    }

    renderLoop();

    return () => {
      cancelAnimationFrame(animId);
    };
  });
</script>

<div class="pick-setup-container">
  <!-- Header & Instructions -->
  <div class="pick-setup-header">
    <div class="header-main">
      <h3>5 · Pick Alignment & Goal Quadrilateral Setup</h3>
      <p class="subtitle">
        Calibrates the pick position using a <strong>generic quadrilateral with movable sides</strong>. Drag any of the 4 corners ($P_0 \dots P_3$) or drag inside to pan the goal shape to match your camera's perspective. When the tag overlaps the goal quadrilateral, the gripper is confirmed in pick position.
      </p>
    </div>

    <div class="header-status">
      {#if isAligned}
        <span class="status-badge aligned">ON TARGET (Inside ±{tolerancePx}px error region)</span>
      {:else if isNear}
        <span class="status-badge near">APPROACHING (Near Error Region)</span>
      {:else if tagFound}
        <span class="status-badge tracking">TAG #{targetTagId} IN VIEW</span>
      {:else}
        <span class="status-badge searching">SEARCHING FOR TAG #{targetTagId}</span>
      {/if}
    </div>
  </div>

  <!-- Main Grid Layout -->
  <div class="main-grid">
    <!-- Camera View Panel -->
    <div class="camera-panel">
      <div class="panel-header">
        <span class="panel-title">Live Camera View · Interactive Goal Quadrilateral</span>
        <span class="panel-sub">Drag corners or inside the quad to adjust</span>
      </div>

      <div class="canvas-wrap">
        <canvas
          bind:this={canvasEl}
          width="640"
          height="480"
          style="cursor: {hoveredCornerIdx !== null ? 'crosshair' : isDragging ? 'grabbing' : 'grab'};"
          onmousedown={onMouseDown}
          onmousemove={onMouseMove}
          onmouseup={onMouseUp}
          onmouseleave={onMouseUp}
        ></canvas>

        <!-- Top-left overlay legend -->
        <div class="cam-legend">
          <span class="legend-chip goal"><span class="swatch goal"></span> Goal Quad & Error Region</span>
          <span class="legend-chip tag"><span class="swatch tag"></span> Detected Tag</span>
        </div>
      </div>

      <!-- Live Error Telemetry Bar -->
      <div class="telemetry-bar">
        <div class="telemetry-item">
          <span class="lbl">Center Error:</span>
          <span class="val {centerErrPx < 8 ? 'good' : 'neutral'}">{centerErrPx.toFixed(1)} px</span>
        </div>
        <div class="telemetry-item">
          <span class="lbl">Avg Size Diff:</span>
          <span class="val {Math.abs(sizeErrPx) < 6 ? 'good' : 'neutral'}">{sizeErrPx >= 0 ? '+' : ''}{sizeErrPx.toFixed(1)} px</span>
        </div>
        <div class="telemetry-item">
          <span class="lbl">Rotation Error:</span>
          <span class="val {Math.abs(yawErrDeg) < 5 ? 'good' : 'neutral'}">{yawErrDeg >= 0 ? '+' : ''}{yawErrDeg.toFixed(1)}°</span>
        </div>
        <div class="telemetry-item">
          <span class="lbl">Error Region:</span>
          <span class="val {isAligned ? 'good' : 'neutral'}">±{tolerancePx} px</span>
        </div>
      </div>
    </div>

    <!-- Levers & Controls Panel -->
    <div class="controls-panel">
      <!-- Target Environment Switch: Sim vs Real -->
      <div class="control-section mode-section">
        <div class="section-title">
          <span>Target Environment</span>
          <span class="val-badge">{calMode.toUpperCase()}</span>
        </div>
        <div class="mode-toggle-group">
          <button
            class="mode-toggle-btn"
            class:active={calMode === 'sim'}
            onclick={() => setCalMode('sim')}
            title="Configure goal quad for simulation camera view"
          >
            Simulation (Sim)
          </button>
          <button
            class="mode-toggle-btn"
            class:active={calMode === 'real'}
            onclick={() => setCalMode('real')}
            title="Configure goal quad for physical robot camera view"
          >
            Physical Robot (Real)
          </button>
        </div>
      </div>

      <!-- Target Tag & Quick Match -->
      <div class="control-section tag-sel-section">
        <label for="tagSelInp">Target Tag & Quick Snap</label>
        <div class="tag-input-row">
          <input
            id="tagSelInp"
            type="number"
            min="1"
            max="499"
            bind:value={targetTagId}
            onchange={syncConfig}
          />
          <button
            class="match-tag-btn"
            onclick={matchCurrentDetectedTag}
            title="Snap goal quadrilateral corners to current detected tag corners"
          >
            Match Current Tag
          </button>
        </div>
      </div>

      <!-- Corner Position Controls -->
      <div class="control-section corners-section">
        <div class="section-title">
          <span>Corner Vertices (Movable)</span>
          <div class="mini-btn-group">
            <button class="mini-btn" onclick={resetToNominalRight} title="Reset to standard right side quad">Right Quad</button>
            <button class="mini-btn" onclick={resetToRectangle} title="Reset to a flat symmetric rectangle">Rectangle</button>
          </div>
        </div>

        <div class="corner-grid">
          <!-- P0: Top-Left -->
          <div class="corner-box">
            <span class="c-title">P0 (Top-Left)</span>
            <div class="coord-inputs">
              <label>X: <input type="number" min="0" max="640" bind:value={p0[0]} onchange={syncConfig} /></label>
              <label>Y: <input type="number" min="0" max="480" bind:value={p0[1]} onchange={syncConfig} /></label>
            </div>
          </div>

          <!-- P1: Top-Right -->
          <div class="corner-box">
            <span class="c-title">P1 (Top-Right)</span>
            <div class="coord-inputs">
              <label>X: <input type="number" min="0" max="640" bind:value={p1[0]} onchange={syncConfig} /></label>
              <label>Y: <input type="number" min="0" max="480" bind:value={p1[1]} onchange={syncConfig} /></label>
            </div>
          </div>

          <!-- P3: Bottom-Left -->
          <div class="corner-box">
            <span class="c-title">P3 (Bottom-Left)</span>
            <div class="coord-inputs">
              <label>X: <input type="number" min="0" max="640" bind:value={p3[0]} onchange={syncConfig} /></label>
              <label>Y: <input type="number" min="0" max="480" bind:value={p3[1]} onchange={syncConfig} /></label>
            </div>
          </div>

          <!-- P2: Bottom-Right -->
          <div class="corner-box">
            <span class="c-title">P2 (Bottom-Right)</span>
            <div class="coord-inputs">
              <label>X: <input type="number" min="0" max="640" bind:value={p2[0]} onchange={syncConfig} /></label>
              <label>Y: <input type="number" min="0" max="480" bind:value={p2[1]} onchange={syncConfig} /></label>
            </div>
          </div>
        </div>

        <!-- Side Lengths Readout -->
        <div class="side-lengths-chip">
          <span>Edges:</span>
          <span>Top: <strong>{sideLengths[0].toFixed(0)}px</strong></span>
          <span>Right: <strong>{sideLengths[1].toFixed(0)}px</strong></span>
          <span>Bot: <strong>{sideLengths[2].toFixed(0)}px</strong></span>
          <span>Left: <strong>{sideLengths[3].toFixed(0)}px</strong></span>
        </div>
      </div>

      <!-- Quick Transform Nudge Levers & Tolerance -->
      <div class="control-section levers-section">
        <span class="section-title">Whole Shape Levers & Error Margin</span>

        <div class="lever-row" style="margin-bottom: 0.5rem;">
          <div class="lever-info">
            <span class="lever-name">Error Border Region (Tolerance)</span>
            <span class="lever-val">±{tolerancePx} px</span>
          </div>
          <input
            type="range"
            min="4"
            max="40"
            step="1"
            bind:value={tolerancePx}
            onchange={syncConfig}
          />
        </div>

        <div class="nudge-btn-grid">
          <button class="nudge-btn" onclick={() => handlePan(-10, 0)}>Left</button>
          <button class="nudge-btn" onclick={() => handlePan(10, 0)}>Right</button>
          <button class="nudge-btn" onclick={() => handlePan(0, -10)}>Up</button>
          <button class="nudge-btn" onclick={() => handlePan(0, 10)}>Down</button>
          <button class="nudge-btn" onclick={() => handleScale(1.08)}>Scale +</button>
          <button class="nudge-btn" onclick={() => handleScale(0.92)}>Scale −</button>
          <button class="nudge-btn" onclick={() => handleRotate(-5)}>Rot −5°</button>
          <button class="nudge-btn" onclick={() => handleRotate(5)}>Rot +5°</button>
        </div>
      </div>

      <!-- Auto-Navigation Section -->
      <div class="control-section auto-nav-section">
        <div class="section-title">
          <span>Visual Servoing Navigation</span>
          <div class="mini-btn-group">
            {#if pickCalState.isNavigating}
              <button
                class="stop-mini-btn"
                onclick={() => pickCalState.stopNavigation()}
                title="Immediately stop navigation"
              >
                STOP
              </button>
            {/if}
            <button
              class="scan-pos-btn"
              disabled={pickCalState.isNavigating || pickCalState.isMovingToScan || pickCalState.isPicking}
              onclick={() => pickCalState.moveToScanPosition()}
              title="Move arm up to scan position pointing straight down at table"
            >
              {#if pickCalState.isMovingToScan}
                <span class="btn-spinner"></span> Moving…
              {:else}
                Move to Scan Position
              {/if}
            </button>
          </div>
        </div>

        <!-- Camera FPS, Descent Step & Timing Levers -->
        <div class="nav-timing-box">
          <div class="lever-row">
            <div class="lever-info">
              <span class="lever-name">Step Settle Delay</span>
              <span class="lever-val">{stepDelayMs} ms</span>
            </div>
            <input
              type="range"
              min="150"
              max="1500"
              step="50"
              bind:value={stepDelayMs}
            />
          </div>

          <div class="lever-row">
            <div class="lever-info">
              <span class="lever-name">Stepwise Descent Step</span>
              <span class="lever-val">{downStepMm} mm</span>
            </div>
            <input
              type="range"
              min="1"
              max="20"
              step="1"
              bind:value={downStepMm}
              onchange={syncConfig}
            />
          </div>

          <div class="lever-row">
            <div class="lever-info">
              <span class="lever-name">Motion Speed / Step Size</span>
              <span class="lever-val">{(stepSpeedGain * 100).toFixed(0)}%</span>
            </div>
            <input
              type="range"
              min="0.1"
              max="1.0"
              step="0.05"
              bind:value={stepSpeedGain}
            />
          </div>
        </div>

        <!-- Primary Stepwise Navigation & Stop Controls -->
        {#if pickCalState.isNavigating || pickCalState.isPicking}
          <div class="nav-active-row">
            <button
              class="primary-btn auto-nav-stepwise-btn busy"
              disabled={true}
            >
              <span class="btn-spinner"></span>
              {#if pickCalState.isPicking}
                Picking & Lifting…
              {:else if pickCalState.activeMode === 'xy'}
                Navigating X-Y…
              {:else if pickCalState.activeMode === 'z'}
                Navigating Z…
              {:else}
                Approaching Stepwise…
              {/if}
            </button>
            <button
              class="stop-nav-btn"
              onclick={() => pickCalState.stopNavigation()}
              title="Immediately stop arm navigation and hold position"
            >
              Stop Navigation
            </button>
          </div>
        {:else}
          <div class="nav-primary-action-group">
            <button
              class="primary-btn auto-nav-stepwise-btn pick-combo-btn"
              disabled={pickCalState.isMovingToScan || pickCalState.isNudging || pickCalState.isPicking}
              onclick={() => pickCalState.autoApproachAndPick(config, targetTagId, stepDelayMs, stepSpeedGain, downStepMm, tolerancePx, pickDownDistanceMm)}
              title="Performs stepwise visual auto-approach to the goal quad, moves {pickDownDistanceMm}mm down onto object, closes gripper, and lifts"
            >
              Auto-Approach & Pick Up
            </button>
            <button
              class="secondary-btn auto-nav-stepwise-btn"
              disabled={pickCalState.isMovingToScan || pickCalState.isNudging || pickCalState.isPicking}
              onclick={() => pickCalState.autoNavigate(config, targetTagId, 'all', stepDelayMs, stepSpeedGain, downStepMm, tolerancePx)}
              title="Aligns X-Y and yaw, descends by {downStepMm}mm, and stops at the goal quad without gripping"
            >
              Approach Only (X-Y → Descend {downStepMm}mm)
            </button>
          </div>
        {/if}

        <div class="auto-nav-btn-grid">
          <button
            class="secondary-btn auto-nav-btn xy {pickCalState.isNavigating && pickCalState.activeMode === 'xy' ? 'busy' : ''}"
            disabled={pickCalState.isNavigating || pickCalState.isMovingToScan || pickCalState.isNudging}
            onclick={() => pickCalState.autoNavigate(config, targetTagId, 'xy', stepDelayMs, stepSpeedGain, downStepMm, tolerancePx)}
            title="Aligns X and Y lateral position and rotation without changing arm height Z"
          >
            {#if pickCalState.isNavigating && pickCalState.activeMode === 'xy'}
              <span class="btn-spinner"></span> Navigating X-Y…
            {:else}
              Auto-Navigate X-Y
            {/if}
          </button>

          <button
            class="secondary-btn auto-nav-btn z {pickCalState.isNavigating && pickCalState.activeMode === 'z' ? 'busy' : ''}"
            disabled={pickCalState.isNavigating || pickCalState.isMovingToScan || pickCalState.isNudging}
            onclick={() => pickCalState.autoNavigate(config, targetTagId, 'z', stepDelayMs, stepSpeedGain, downStepMm, tolerancePx)}
            title="Adjusts height Z until tag size matches goal quadrilateral without moving X or Y"
          >
            {#if pickCalState.isNavigating && pickCalState.activeMode === 'z'}
              <span class="btn-spinner"></span> Navigating Z…
            {:else}
              Auto-Navigate Z
            {/if}
          </button>
        </div>

        <p class="auto-nav-desc">
          <strong>Stepwise Approach</strong>: First visual-servos $(X, Y)$ & yaw until centered, descends by <strong>{downStepMm} mm</strong>, and repeats until the tag is verified inside the <strong>±{tolerancePx} px</strong> error region.
        </p>

        {#if pickCalState.navStatus}
          <div
            class="nav-feedback-box"
            class:ok={pickCalState.navSuccess === true}
            class:err={pickCalState.navSuccess === false}
            class:running={pickCalState.isNavigating || pickCalState.isMovingToScan || pickCalState.isNudging}
          >
            {pickCalState.navStatus}
          </div>
        {/if}
      </div>

      <!-- Height Jog & Final Pick Action -->
      <div class="control-section height-jog-section">
        <div class="section-title">
          <span>Height Jog & Pick Action</span>
          <span class="val-badge">{downDistanceMm} mm</span>
        </div>

        <div class="lever-row">
          <div class="lever-info">
            <span class="lever-name">Jog Step Distance</span>
            <span class="lever-val">{downDistanceMm} mm</span>
          </div>
          <input
            type="range"
            min="1"
            max="60"
            step="1"
            bind:value={downDistanceMm}
          />
        </div>

        <div class="height-btn-row">
          <button
            class="secondary-btn down-btn {pickCalState.isNudging ? 'busy' : ''}"
            disabled={pickCalState.isNavigating || pickCalState.isMovingToScan || pickCalState.isNudging || pickCalState.isPicking}
            onclick={() => pickCalState.nudgeHeight(-downDistanceMm)}
            title="Move arm down by {downDistanceMm} mm"
          >
            {#if pickCalState.isNudging}
              <span class="btn-spinner"></span> Moving…
            {:else}
              Go down (−{downDistanceMm} mm)
            {/if}
          </button>

          <button
            class="secondary-btn up-btn"
            disabled={pickCalState.isNavigating || pickCalState.isMovingToScan || pickCalState.isNudging || pickCalState.isPicking}
            onclick={() => pickCalState.nudgeHeight(downDistanceMm)}
            title="Move arm up by {downDistanceMm} mm"
          >
            + {downDistanceMm}mm
          </button>
        </div>

        <!-- Dedicated Move a bit below and pick up button -->
        <button
          class="primary-btn pick-up-btn {pickCalState.isPicking ? 'busy' : ''}"
          disabled={pickCalState.isNavigating || pickCalState.isMovingToScan || pickCalState.isNudging || pickCalState.isPicking}
          onclick={() => pickCalState.pickAndLift(pickDownDistanceMm)}
          title="Moves the arm {pickDownDistanceMm}mm more down, closes the gripper, and lifts the arm"
        >
          {#if pickCalState.isPicking}
            <span class="btn-spinner"></span> Picking & Lifting…
          {:else}
            Move a bit below and pick up (−{pickDownDistanceMm}mm, grip & lift)
          {/if}
        </button>
      </div>

      <!-- Action Footer -->
      <div class="footer-btn-row">
        <button class="secondary-btn" onclick={handleSave}>
          Save Goal Config
        </button>
      </div>

      {#if saveMsg}
        <div class="toast-msg">✓ {saveMsg}</div>
      {/if}
    </div>
  </div>
</div>

<style>
  .pick-setup-container {
    display: flex;
    flex-direction: column;
    gap: 1rem;
    padding: 0.25rem 0;
    color: var(--text, #f1f5f9);
  }

  .pick-setup-header {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    gap: 1rem;
    flex-wrap: wrap;
  }
  .pick-setup-header h3 {
    margin: 0 0 0.3rem;
    font-size: 1.15rem;
    color: #f1f5f9;
  }
  .subtitle {
    margin: 0;
    font-size: 0.82rem;
    color: var(--muted, #94a3b8);
    line-height: 1.45;
    max-width: 680px;
  }

  .status-badge {
    font-size: 0.75rem;
    font-weight: 700;
    padding: 0.35rem 0.75rem;
    border-radius: 6px;
    letter-spacing: 0.02em;
    text-transform: uppercase;
    display: inline-flex;
    align-items: center;
  }
  .status-badge.aligned {
    background: rgba(34, 197, 94, 0.2);
    border: 1px solid rgba(34, 197, 94, 0.5);
    color: #4ade80;
    box-shadow: 0 0 12px rgba(34, 197, 94, 0.3);
  }
  .status-badge.near {
    background: rgba(234, 179, 8, 0.2);
    border: 1px solid rgba(234, 179, 8, 0.4);
    color: #facc15;
  }
  .status-badge.tracking {
    background: rgba(56, 189, 248, 0.15);
    border: 1px solid rgba(56, 189, 248, 0.35);
    color: #38bdf8;
  }
  .status-badge.searching {
    background: rgba(148, 163, 184, 0.12);
    border: 1px solid rgba(148, 163, 184, 0.25);
    color: #94a3b8;
  }

  .main-grid {
    display: grid;
    grid-template-columns: 1fr 340px;
    gap: 1.25rem;
    align-items: start;
  }

  @media (max-width: 960px) {
    .main-grid {
      grid-template-columns: 1fr;
    }
  }

  .camera-panel {
    display: flex;
    flex-direction: column;
    background: #090d16;
    border: 1px solid rgba(56, 189, 248, 0.25);
    border-radius: 8px;
    overflow: hidden;
    box-shadow: 0 4px 14px rgba(0, 0, 0, 0.3);
  }

  .panel-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 0.5rem 0.85rem;
    background: rgba(15, 23, 42, 0.95);
    border-bottom: 1px solid rgba(148, 163, 184, 0.15);
    font-size: 0.78rem;
  }
  .panel-title {
    font-weight: 600;
    color: #f1f5f9;
  }
  .panel-sub {
    color: #38bdf8;
    font-size: 0.72rem;
  }

  .canvas-wrap {
    position: relative;
    width: 100%;
    aspect-ratio: 4 / 3;
    background: #000000;
    display: flex;
    align-items: center;
    justify-content: center;
    user-select: none;
  }

  canvas {
    width: 100%;
    height: 100%;
    object-fit: contain;
    display: block;
  }

  .cam-legend {
    position: absolute;
    top: 8px;
    left: 8px;
    display: flex;
    gap: 0.5rem;
    pointer-events: none;
  }
  .legend-chip {
    font-size: 0.68rem;
    font-weight: 600;
    padding: 0.2rem 0.45rem;
    border-radius: 4px;
    background: rgba(15, 23, 42, 0.85);
    backdrop-filter: blur(4px);
    display: flex;
    align-items: center;
    gap: 0.3rem;
  }
  .legend-chip.goal { color: #fbbf24; border: 1px solid rgba(245, 158, 11, 0.4); }
  .legend-chip.tag { color: #38bdf8; border: 1px solid rgba(56, 189, 248, 0.4); }
  .swatch {
    width: 8px;
    height: 8px;
    border-radius: 2px;
  }
  .swatch.goal { background: #f59e0b; }
  .swatch.tag { background: #38bdf8; }

  .telemetry-bar {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 0.5rem;
    padding: 0.5rem 0.75rem;
    background: rgba(15, 23, 42, 0.95);
    border-top: 1px solid rgba(148, 163, 184, 0.15);
    font-family: monospace;
    font-size: 0.72rem;
  }
  .telemetry-item {
    display: flex;
    flex-direction: column;
    gap: 0.1rem;
  }
  .telemetry-item .lbl {
    color: #64748b;
    font-size: 0.65rem;
  }
  .telemetry-item .val {
    color: #f1f5f9;
    font-weight: 700;
  }
  .telemetry-item .val.good { color: #4ade80; }
  .telemetry-item .val.near { color: #facc15; }

  .controls-panel {
    display: flex;
    flex-direction: column;
    gap: 0.85rem;
  }

  .control-section {
    background: rgba(15, 23, 42, 0.7);
    border: 1px solid rgba(148, 163, 184, 0.18);
    border-radius: 8px;
    padding: 0.75rem 0.85rem;
    display: flex;
    flex-direction: column;
    gap: 0.6rem;
  }

  .tag-sel-section label {
    font-size: 0.78rem;
    font-weight: 600;
    color: #f1f5f9;
  }
  .tag-input-row {
    display: flex;
    gap: 0.5rem;
  }
  .tag-input-row input {
    width: 70px;
    background: rgba(15, 23, 42, 0.9);
    border: 1px solid rgba(148, 163, 184, 0.3);
    color: #38bdf8;
    font-weight: 700;
    font-size: 0.88rem;
    border-radius: 4px;
    padding: 0.35rem 0.5rem;
    text-align: center;
  }

  .match-tag-btn {
    flex: 1;
    background: rgba(56, 189, 248, 0.15);
    border: 1px solid rgba(56, 189, 248, 0.4);
    color: #38bdf8;
    font-weight: 600;
    font-size: 0.8rem;
    border-radius: 4px;
    padding: 0.35rem 0.6rem;
    cursor: pointer;
    transition: all 0.15s ease;
  }
  .match-tag-btn:hover {
    background: rgba(56, 189, 248, 0.25);
    color: #ffffff;
    border-color: #38bdf8;
  }

  .section-title {
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 0.82rem;
    font-weight: 600;
    color: #fbbf24;
  }
  .mini-btn-group {
    display: flex;
    gap: 0.3rem;
  }
  .mini-btn {
    background: rgba(30, 41, 59, 0.8);
    border: 1px solid rgba(148, 163, 184, 0.25);
    color: #e2e8f0;
    border-radius: 4px;
    padding: 0.15rem 0.4rem;
    font-size: 0.68rem;
    cursor: pointer;
    transition: all 0.15s;
  }
  .mini-btn:hover {
    background: rgba(56, 189, 248, 0.2);
    border-color: #38bdf8;
  }

  .scan-pos-btn {
    background: rgba(30, 41, 59, 0.9);
    border: 1px solid rgba(56, 189, 248, 0.35);
    color: #38bdf8;
    border-radius: 4px;
    padding: 0.2rem 0.5rem;
    font-size: 0.72rem;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.15s ease;
    display: inline-flex;
    align-items: center;
    gap: 0.3rem;
  }
  .scan-pos-btn:hover:not(:disabled) {
    background: rgba(56, 189, 248, 0.2);
    border-color: #38bdf8;
    color: #ffffff;
  }
  .scan-pos-btn:disabled {
    opacity: 0.6;
    cursor: not-allowed;
  }

  .corner-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 0.5rem;
  }
  .corner-box {
    background: rgba(15, 23, 42, 0.8);
    border: 1px solid rgba(148, 163, 184, 0.15);
    border-radius: 6px;
    padding: 0.4rem 0.5rem;
    display: flex;
    flex-direction: column;
    gap: 0.3rem;
  }
  .c-title {
    font-size: 0.68rem;
    font-weight: 700;
    color: #fbbf24;
  }
  .coord-inputs {
    display: flex;
    gap: 0.4rem;
  }
  .coord-inputs label {
    display: flex;
    align-items: center;
    gap: 0.2rem;
    font-size: 0.68rem;
    color: #94a3b8;
  }
  .coord-inputs input {
    width: 48px;
    background: rgba(15, 23, 42, 0.9);
    border: 1px solid rgba(148, 163, 184, 0.25);
    color: #f1f5f9;
    font-size: 0.75rem;
    font-family: monospace;
    padding: 0.15rem 0.25rem;
    border-radius: 3px;
    text-align: center;
  }

  .side-lengths-chip {
    display: flex;
    justify-content: space-between;
    font-size: 0.68rem;
    color: #94a3b8;
    background: rgba(15, 23, 42, 0.8);
    padding: 0.3rem 0.5rem;
    border-radius: 4px;
    border: 1px solid rgba(148, 163, 184, 0.12);
  }
  .side-lengths-chip strong {
    color: #fbbf24;
  }

  .nudge-btn-grid {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 0.35rem;
  }
  .nudge-btn {
    background: rgba(30, 41, 59, 0.7);
    border: 1px solid rgba(148, 163, 184, 0.2);
    color: #f1f5f9;
    border-radius: 4px;
    padding: 0.3rem 0.2rem;
    font-size: 0.7rem;
    font-weight: 500;
    cursor: pointer;
    transition: all 0.15s ease;
    text-align: center;
  }
  .nudge-btn:hover {
    background: rgba(56, 189, 248, 0.2);
    border-color: #38bdf8;
    color: #38bdf8;
  }

  .nav-timing-box {
    display: flex;
    flex-direction: column;
    gap: 0.45rem;
    background: rgba(15, 23, 42, 0.55);
    border: 1px solid rgba(148, 163, 184, 0.15);
    border-radius: 6px;
    padding: 0.45rem 0.6rem;
  }

  .auto-nav-btn-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 0.5rem;
  }

  .auto-nav-btn {
    width: 100%;
    font-weight: 700;
    font-size: 0.84rem;
    padding: 0.6rem 0.5rem;
    border-radius: 6px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 0.4rem;
    transition: all 0.15s ease;
    text-align: center;
  }
  .auto-nav-btn.xy {
    background: linear-gradient(135deg, #0284c7 0%, #0369a1 100%);
    border: 1px solid #38bdf8;
    color: #ffffff;
    box-shadow: 0 2px 8px rgba(2, 132, 199, 0.4);
  }
  .auto-nav-btn.xy:hover:not(:disabled) {
    background: linear-gradient(135deg, #0ea5e9 0%, #0284c7 100%);
    box-shadow: 0 4px 12px rgba(14, 165, 233, 0.5);
    transform: translateY(-1px);
  }
  .auto-nav-btn.z {
    background: linear-gradient(135deg, #0d9488 0%, #0f766e 100%);
    border: 1px solid #2dd4bf;
    color: #ffffff;
    box-shadow: 0 2px 8px rgba(13, 148, 136, 0.4);
  }
  .auto-nav-btn.z:hover:not(:disabled) {
    background: linear-gradient(135deg, #14b8a6 0%, #0d9488 100%);
    box-shadow: 0 4px 12px rgba(20, 184, 166, 0.5);
    transform: translateY(-1px);
  }
  .auto-nav-btn:disabled {
    opacity: 0.65;
    cursor: not-allowed;
  }

  .mode-toggle-group {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 0.4rem;
  }
  .mode-toggle-btn {
    background: rgba(15, 23, 42, 0.8);
    border: 1px solid rgba(148, 163, 184, 0.25);
    color: #94a3b8;
    border-radius: 6px;
    padding: 0.45rem 0.5rem;
    font-size: 0.76rem;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.15s ease;
    text-align: center;
  }
  .mode-toggle-btn:hover {
    border-color: rgba(56, 189, 248, 0.5);
    color: #f1f5f9;
  }
  .mode-toggle-btn.active {
    background: linear-gradient(135deg, rgba(2, 132, 199, 0.35) 0%, rgba(14, 165, 233, 0.2) 100%);
    border-color: #38bdf8;
    color: #38bdf8;
    box-shadow: 0 0 10px rgba(56, 189, 248, 0.25);
  }

  .nav-primary-action-group {
    display: flex;
    flex-direction: column;
    gap: 0.45rem;
  }

  .auto-nav-stepwise-btn {
    background: linear-gradient(135deg, #6366f1 0%, #4f46e5 100%);
    border: 1px solid #818cf8;
    color: #ffffff;
    font-weight: 700;
    font-size: 0.88rem;
    padding: 0.65rem 0.75rem;
    border-radius: 6px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 0.4rem;
    box-shadow: 0 3px 10px rgba(99, 102, 241, 0.4);
    transition: all 0.15s ease;
  }
  .auto-nav-stepwise-btn:hover:not(:disabled) {
    background: linear-gradient(135deg, #818cf8 0%, #6366f1 100%);
    box-shadow: 0 4px 14px rgba(129, 140, 248, 0.55);
    transform: translateY(-1px);
  }
  .auto-nav-stepwise-btn:disabled {
    opacity: 0.65;
    cursor: not-allowed;
  }

  .pick-combo-btn {
    background: linear-gradient(135deg, #2563eb 0%, #1d4ed8 100%);
    border: 1px solid #60a5fa;
    box-shadow: 0 3px 10px rgba(37, 99, 235, 0.45);
    font-size: 0.92rem;
  }
  .pick-combo-btn:hover:not(:disabled) {
    background: linear-gradient(135deg, #3b82f6 0%, #2563eb 100%);
    box-shadow: 0 4px 14px rgba(59, 130, 246, 0.6);
  }

  .nav-active-row {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 0.5rem;
  }

  .stop-nav-btn {
    background: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
    border: 1px solid #f87171;
    color: #ffffff;
    font-weight: 700;
    font-size: 0.85rem;
    padding: 0.65rem 0.75rem;
    border-radius: 6px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 0.4rem;
    box-shadow: 0 3px 10px rgba(239, 68, 68, 0.45);
    transition: all 0.15s ease;
  }
  .stop-nav-btn:hover {
    background: linear-gradient(135deg, #f87171 0%, #ef4444 100%);
    box-shadow: 0 4px 14px rgba(248, 113, 113, 0.6);
    transform: translateY(-1px);
  }

  .stop-mini-btn {
    background: rgba(239, 68, 68, 0.2);
    border: 1px solid #ef4444;
    color: #fca5a5;
    border-radius: 4px;
    padding: 0.2rem 0.5rem;
    font-size: 0.72rem;
    font-weight: 700;
    cursor: pointer;
    transition: all 0.15s ease;
    display: inline-flex;
    align-items: center;
    gap: 0.25rem;
  }
  .stop-mini-btn:hover {
    background: rgba(239, 68, 68, 0.4);
    color: #ffffff;
  }

  .auto-nav-all-btn {
    font-size: 0.78rem;
    padding: 0.45rem;
    border-color: rgba(148, 163, 184, 0.35);
  }
  .auto-nav-all-btn:hover:not(:disabled) {
    border-color: #fbbf24;
    color: #fbbf24;
  }

  .val-badge {
    font-family: monospace;
    font-size: 0.76rem;
    font-weight: 700;
    color: #38bdf8;
    background: rgba(56, 189, 248, 0.12);
    padding: 0.15rem 0.45rem;
    border-radius: 4px;
    border: 1px solid rgba(56, 189, 248, 0.25);
  }

  .height-btn-row {
    display: grid;
    grid-template-columns: 1fr auto;
    gap: 0.5rem;
  }

  .down-btn {
    background: linear-gradient(135deg, #059669 0%, #047857 100%);
    border: 1px solid #34d399;
    color: #ffffff;
    font-weight: 700;
    font-size: 0.82rem;
    padding: 0.55rem 0.75rem;
    border-radius: 6px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 0.4rem;
    box-shadow: 0 2px 8px rgba(5, 150, 105, 0.35);
    transition: all 0.15s ease;
  }
  .down-btn:hover:not(:disabled) {
    background: linear-gradient(135deg, #10b981 0%, #059669 100%);
    box-shadow: 0 4px 12px rgba(16, 185, 129, 0.5);
    transform: translateY(-1px);
  }
  .down-btn:disabled {
    opacity: 0.65;
    cursor: not-allowed;
  }

  .up-btn {
    padding: 0.55rem 0.75rem;
    font-size: 0.8rem;
    font-weight: 600;
    white-space: nowrap;
  }

  .pick-up-btn {
    margin-top: 0.4rem;
    background: linear-gradient(135deg, #d97706 0%, #b45309 100%);
    border: 1px solid #f59e0b;
    color: #ffffff;
    font-weight: 700;
    font-size: 0.86rem;
    padding: 0.65rem 0.75rem;
    border-radius: 6px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 0.4rem;
    box-shadow: 0 3px 10px rgba(217, 119, 6, 0.4);
    transition: all 0.15s ease;
  }
  .pick-up-btn:hover:not(:disabled) {
    background: linear-gradient(135deg, #f59e0b 0%, #d97706 100%);
    box-shadow: 0 4px 14px rgba(245, 158, 11, 0.55);
    transform: translateY(-1px);
  }
  .pick-up-btn:disabled {
    opacity: 0.65;
    cursor: not-allowed;
  }

  .auto-nav-desc {
    margin: 0;
    font-size: 0.72rem;
    color: #94a3b8;
    line-height: 1.35;
  }

  .nav-feedback-box {
    padding: 0.55rem 0.75rem;
    border-radius: 6px;
    font-size: 0.76rem;
    line-height: 1.35;
    background: rgba(15, 23, 42, 0.8);
    border: 1px solid rgba(148, 163, 184, 0.2);
    color: #f1f5f9;
  }
  .nav-feedback-box.ok {
    background: rgba(34, 197, 94, 0.15);
    border-color: rgba(34, 197, 94, 0.4);
    color: #4ade80;
  }
  .nav-feedback-box.err {
    background: rgba(239, 68, 68, 0.15);
    border-color: rgba(239, 68, 68, 0.4);
    color: #f87171;
  }
  .nav-feedback-box.running {
    background: rgba(2, 132, 199, 0.15);
    border-color: rgba(56, 189, 248, 0.4);
    color: #38bdf8;
  }

  .btn-spinner {
    width: 14px;
    height: 14px;
    border: 2px solid rgba(255, 255, 255, 0.3);
    border-top-color: #ffffff;
    border-radius: 50%;
    animation: spin 0.7s linear infinite;
    display: inline-block;
  }
  @keyframes spin {
    to { transform: rotate(360deg); }
  }

  .footer-btn-row {
    display: flex;
    gap: 0.5rem;
  }
  .secondary-btn {
    background: rgba(15, 23, 42, 0.8);
    border: 1px solid rgba(148, 163, 184, 0.3);
    color: var(--text, #f1f5f9);
    border-radius: 6px;
    padding: 0.45rem 0.8rem;
    font-size: 0.8rem;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.15s;
    width: 100%;
  }
  .secondary-btn:hover:not(:disabled) {
    background: rgba(30, 41, 59, 0.9);
    border-color: #38bdf8;
    color: #38bdf8;
  }

  .toast-msg {
    padding: 0.4rem 0.75rem;
    border-radius: 6px;
    font-size: 0.76rem;
    background: rgba(34, 197, 94, 0.18);
    border: 1px solid rgba(34, 197, 94, 0.4);
    color: #4ade80;
    text-align: center;
  }
</style>
