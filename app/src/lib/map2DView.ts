// Interactive 2D top-down map view showing the discovered arena, robot pose,
// heading, camera FOV cone, trajectory trail, and discovered stations/tags.

import { STATIONS, type PropKind } from './boardSim';

export interface MapPlace {
  id: number;
  p: [number, number, number]; // [x, y, z] in world meters
  faceYaw?: number;
  label?: string;
  prop?: PropKind;
}

export interface MapLiveTag {
  id: number;
  p: [number, number, number];
  sizeMm?: number;
}

export interface Map2DOptions {
  onSelectPlace?: (id: number) => void;
}

// Station icon / prop metadata
const PROP_ICONS: Record<string, string> = {
  block: '🪵',
  apple: '🍎',
  banana: '🍌',
  orange: '🍊',
  bottle: '🧴',
  plant: '🪴',
  basket: '🧺',
};

const PROP_NAMES: Record<string, string> = {
  block: 'Jenga Block',
  apple: 'Apple',
  banana: 'Banana',
  orange: 'Orange',
  bottle: 'Water Bottle',
  plant: 'Potted Plant',
  basket: 'Drop Basket',
};

export class Map2DView {
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private options: Map2DOptions;

  // View transform
  private scale = 140; // pixels per meter
  private panX = 0; // offset in canvas pixels
  private panY = 0;
  private isDragging = false;
  private dragStartX = 0;
  private dragStartY = 0;
  private hoveredPlaceId: number | null = null;

  // Robot state
  private robotX = 0;
  private robotY = 0;
  private robotYawRad = 0;
  private hasBase = true;
  private trail: Array<{ x: number; y: number }> = [];
  private lastTrailTime = 0;

  // Arena & Places state
  private places: MapPlace[] = [];
  private liveTags: MapLiveTag[] = [];
  private activeTargetId: number | null = null;
  private arrivedPlaceId: number | null = null;

  // Animation frame
  private animationId: number | null = null;
  private boundListeners: Array<{ target: HTMLElement | Window; event: string; fn: any }> = [];

  constructor(canvas: HTMLCanvasElement, options: Map2DOptions = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d')!;
    this.options = options;

    this.initEventListeners();
    this.resize(canvas.clientWidth || 300, canvas.clientHeight || 200);
  }

  private initEventListeners() {
    const onMouseDown = (e: MouseEvent) => {
      this.isDragging = true;
      this.dragStartX = e.clientX - this.panX;
      this.dragStartY = e.clientY - this.panY;
    };

    const onMouseMove = (e: MouseEvent) => {
      const rect = this.canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      if (this.isDragging) {
        this.panX = e.clientX - this.dragStartX;
        this.panY = e.clientY - this.dragStartY;
        this.render();
        return;
      }

      // Check hover over places
      const world = this.screenToWorld(mouseX, mouseY);
      let found: number | null = null;
      for (const place of this.places) {
        const dx = place.p[0] - world.x;
        const dy = place.p[1] - world.y;
        if (Math.hypot(dx, dy) < 0.18) {
          found = place.id;
          break;
        }
      }
      if (found !== this.hoveredPlaceId) {
        this.hoveredPlaceId = found;
        this.canvas.style.cursor = found !== null ? 'pointer' : 'grab';
        this.render();
      }
    };

    const onMouseUp = (e: MouseEvent) => {
      if (this.isDragging) {
        this.isDragging = false;
      }
    };

    const onClick = (e: MouseEvent) => {
      const rect = this.canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;
      const world = this.screenToWorld(mouseX, mouseY);

      for (const place of this.places) {
        const dx = place.p[0] - world.x;
        const dy = place.p[1] - world.y;
        if (Math.hypot(dx, dy) < 0.18) {
          if (this.options.onSelectPlace) {
            this.options.onSelectPlace(place.id);
          }
          break;
        }
      }
    };

    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const zoomFactor = e.deltaY < 0 ? 1.12 : 0.89;
      const rect = this.canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      // Zoom toward cursor
      const worldBefore = this.screenToWorld(mouseX, mouseY);
      this.scale = Math.max(50, Math.min(450, this.scale * zoomFactor));
      const screenAfter = this.worldToScreen(worldBefore.x, worldBefore.y);
      this.panX += mouseX - screenAfter.x;
      this.panY += mouseY - screenAfter.y;

      this.render();
    };

    this.canvas.addEventListener('mousedown', onMouseDown);
    window.addEventListener('mousemove', onMouseMove);
    window.addEventListener('mouseup', onMouseUp);
    this.canvas.addEventListener('click', onClick);
    this.canvas.addEventListener('wheel', onWheel, { passive: false });

    this.boundListeners.push(
      { target: this.canvas, event: 'mousedown', fn: onMouseDown },
      { target: window, event: 'mousemove', fn: onMouseMove },
      { target: window, event: 'mouseup', fn: onMouseUp },
      { target: this.canvas, event: 'click', fn: onClick },
      { target: this.canvas, event: 'wheel', fn: onWheel },
    );
  }

  private screenToWorld(sx: number, sy: number): { x: number; y: number } {
    const cx = this.canvas.width / (2 * (window.devicePixelRatio || 1)) + this.panX;
    const cy = this.canvas.height / (2 * (window.devicePixelRatio || 1)) + this.panY;
    // In sim: +X is Forward (Up on screen), +Y is Left (Left on screen)
    // Canvas: +sx is Right, +sy is Down
    // Screen coords: x_screen = cx - y_world * scale; y_screen = cy - x_world * scale
    const worldY = -(sx - cx) / this.scale;
    const worldX = -(sy - cy) / this.scale;
    return { x: worldX, y: worldY };
  }

  private worldToScreen(wx: number, wy: number): { x: number; y: number } {
    const cx = this.canvas.width / (2 * (window.devicePixelRatio || 1)) + this.panX;
    const cy = this.canvas.height / (2 * (window.devicePixelRatio || 1)) + this.panY;
    const sx = cx - wy * this.scale;
    const sy = cy - wx * this.scale;
    return { x: sx, y: sy };
  }

  setRobotPose(x: number, y: number, yawDeg: number, hasBase = true) {
    this.robotX = x;
    this.robotY = y;
    this.robotYawRad = (yawDeg * Math.PI) / 180;
    this.hasBase = hasBase;

    const now = performance.now();
    if (now - this.lastTrailTime > 150) {
      this.lastTrailTime = now;
      if (
        this.trail.length === 0 ||
        Math.hypot(this.trail[this.trail.length - 1].x - x, this.trail[this.trail.length - 1].y - y) > 0.02
      ) {
        this.trail.push({ x, y });
        if (this.trail.length > 80) this.trail.shift();
      }
    }
  }

  setPlaces(places: Array<{ id: number; p: [number, number, number]; faceYaw?: number }>) {
    this.places = places.map((p) => {
      const def = STATIONS.find((s) => s.navTag === p.id);
      return {
        ...p,
        prop: def?.prop,
        label: def ? PROP_NAMES[def.prop] || `Tag ${p.id}` : `Tag ${p.id}`,
      };
    });
  }

  setTags(tags: Array<{ id: number; p: [number, number, number]; sizeMm?: number }>) {
    this.liveTags = tags;
  }

  setNavigatingTarget(targetId: number | null) {
    this.activeTargetId = targetId;
  }

  setArrivedPlace(placeId: number | null) {
    this.arrivedPlaceId = placeId;
  }

  // Backwards compatibility shim with TagView if needed
  setRobot(_model: any, _data: any, _geomIds: number[]) {}

  resize(width: number, height: number) {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.canvas.width = width * dpr;
    this.canvas.height = height * dpr;
    this.canvas.style.width = `${width}px`;
    this.canvas.style.height = `${height}px`;
    this.render();
  }

  render() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = this.canvas.width / dpr;
    const h = this.canvas.height / dpr;

    this.ctx.save();
    this.ctx.scale(dpr, dpr);

    // Clear background
    this.ctx.fillStyle = '#0a0d14';
    this.ctx.fillRect(0, 0, w, h);

    // Draw Grid & Axes
    this.drawGrid(w, h);

    // Draw Trajectory Trail
    this.drawTrail();

    // Draw Discovered Places & Pedestals
    this.drawPlaces();

    // Draw Live Tag Detections
    this.drawLiveTags();

    // Draw Robot Chassis & Heading FOV
    this.drawRobot();

    // Draw HUD & Mini Overlay
    this.drawHud(w, h);

    this.ctx.restore();
  }

  private drawGrid(w: number, h: number) {
    const ctx = this.ctx;
    const stepMeters = 0.5;
    const stepPx = stepMeters * this.scale;

    ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
    ctx.lineWidth = 1;

    const center = this.worldToScreen(0, 0);

    // Vertical grid lines (constant world Y)
    const minScreenX = 0;
    const maxScreenX = w;
    const firstLineX = center.x + Math.floor((minScreenX - center.x) / stepPx) * stepPx;
    for (let x = firstLineX; x <= maxScreenX; x += stepPx) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }

    // Horizontal grid lines (constant world X)
    const minScreenY = 0;
    const maxScreenY = h;
    const firstLineY = center.y + Math.floor((minScreenY - center.y) / stepPx) * stepPx;
    for (let y = firstLineY; y <= maxScreenY; y += stepPx) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    // World Origin axes
    ctx.strokeStyle = 'rgba(239, 68, 68, 0.4)'; // X axis (Red) - forward
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(center.x, center.y);
    ctx.lineTo(center.x, center.y - 40);
    ctx.stroke();

    ctx.strokeStyle = 'rgba(34, 197, 94, 0.4)'; // Y axis (Green) - left
    ctx.beginPath();
    ctx.moveTo(center.x, center.y);
    ctx.lineTo(center.x - 40, center.y);
    ctx.stroke();

    // Origin dot
    ctx.fillStyle = '#64748b';
    ctx.beginPath();
    ctx.arc(center.x, center.y, 3, 0, Math.PI * 2);
    ctx.fill();

    // Subtle distance rings around origin (0.5m, 1.0m, 1.5m)
    ctx.strokeStyle = 'rgba(59, 130, 246, 0.12)';
    ctx.setLineDash([4, 4]);
    for (const r of [0.5, 1.0, 1.5, 2.0]) {
      ctx.beginPath();
      ctx.arc(center.x, center.y, r * this.scale, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }

  private drawTrail() {
    if (this.trail.length < 2) return;
    const ctx = this.ctx;
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.35)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < this.trail.length; i++) {
      const s = this.worldToScreen(this.trail[i].x, this.trail[i].y);
      if (i === 0) ctx.moveTo(s.x, s.y);
      else ctx.lineTo(s.x, s.y);
    }
    ctx.stroke();
  }

  private drawPlaces() {
    const ctx = this.ctx;

    for (const place of this.places) {
      const s = this.worldToScreen(place.p[0], place.p[1]);
      const isHovered = this.hoveredPlaceId === place.id;
      const isTarget = this.activeTargetId === place.id;
      const isArrived = this.arrivedPlaceId === place.id;

      // Pedestal radius: ~0.12m
      const rPx = Math.max(14, 0.12 * this.scale);

      // Station pulse if target
      if (isTarget) {
        ctx.fillStyle = 'rgba(59, 130, 246, 0.2)';
        ctx.beginPath();
        ctx.arc(s.x, s.y, rPx + 8, 0, Math.PI * 2);
        ctx.fill();
      }

      // Base circle
      ctx.fillStyle = isArrived
        ? 'rgba(34, 197, 94, 0.25)'
        : isTarget
          ? 'rgba(59, 130, 246, 0.3)'
          : isHovered
            ? 'rgba(245, 158, 11, 0.3)'
            : 'rgba(30, 41, 59, 0.85)';
      ctx.strokeStyle = isArrived
        ? '#22c55e'
        : isTarget
          ? '#3b82f6'
          : isHovered
            ? '#f59e0b'
            : '#475569';
      ctx.lineWidth = isHovered || isTarget ? 2.5 : 1.5;

      ctx.beginPath();
      ctx.arc(s.x, s.y, rPx, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Tag ID inside marker
      ctx.fillStyle = isTarget ? '#93c5fd' : isHovered ? '#fde68a' : '#e2e8f0';
      ctx.font = 'bold 11px system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(String(place.id), s.x, s.y);

      // Station Label badge
      const labelText = `Tag ${place.id}`;
      ctx.font = '10px system-ui, sans-serif';
      const textWidth = ctx.measureText(labelText).width;
      const badgeY = s.y + rPx + 11;

      ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
      ctx.strokeStyle = isTarget ? '#3b82f6' : 'rgba(255, 255, 255, 0.15)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.roundRect(s.x - textWidth / 2 - 5, badgeY - 7, textWidth + 10, 14, 4);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = isTarget ? '#60a5fa' : '#cbd5e1';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(labelText, s.x, badgeY);

      // Draw navigation approach standoff arrow if target
      if (isTarget) {
        const rob = this.worldToScreen(this.robotX, this.robotY);
        ctx.strokeStyle = '#3b82f6';
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        ctx.moveTo(rob.x, rob.y);
        ctx.lineTo(s.x, s.y);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    }
  }

  private drawLiveTags() {
    const ctx = this.ctx;
    for (const tag of this.liveTags) {
      const s = this.worldToScreen(tag.p[0], tag.p[1]);
      ctx.fillStyle = '#ef4444';
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(s.x, s.y, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
  }

  private drawRobot() {
    const ctx = this.ctx;
    const s = this.worldToScreen(this.robotX, this.robotY);

    // Heading direction
    // In our coordinate mapping: screen angle 0 is along -Y (Forward X)
    // robotYawRad = 0 => pointing UP on screen (-Y)
    // robotYawRad > 0 (CCW in world) => turning LEFT on screen (-X)
    const headingScreenAngle = -Math.PI / 2 - this.robotYawRad;

    // FOV cone (approx ~65° FOV, 1.2m range)
    const fovAngle = (65 * Math.PI) / 180;
    const fovRangePx = 1.2 * this.scale;
    const leftAngle = headingScreenAngle - fovAngle / 2;
    const rightAngle = headingScreenAngle + fovAngle / 2;

    const grad = ctx.createRadialGradient(s.x, s.y, 5, s.x, s.y, fovRangePx);
    grad.addColorStop(0, 'rgba(56, 189, 248, 0.28)');
    grad.addColorStop(0.7, 'rgba(56, 189, 248, 0.08)');
    grad.addColorStop(1, 'rgba(56, 189, 248, 0)');

    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.moveTo(s.x, s.y);
    ctx.arc(s.x, s.y, fovRangePx, leftAngle, rightAngle);
    ctx.closePath();
    ctx.fill();

    // Robot body (radius ~0.11m)
    const robotR = Math.max(12, 0.11 * this.scale);

    ctx.save();
    ctx.translate(s.x, s.y);
    ctx.rotate(headingScreenAngle + Math.PI / 2);

    // Chassis body
    ctx.fillStyle = '#0284c7';
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(-robotR, -robotR * 1.1, robotR * 2, robotR * 2.2, 6);
    ctx.fill();
    ctx.stroke();

    // Wheels
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(-robotR - 3, -robotR * 0.7, 3, robotR * 1.4);
    ctx.fillRect(robotR, -robotR * 0.7, 3, robotR * 1.4);

    // Heading nose pointer
    ctx.fillStyle = '#f8fafc';
    ctx.beginPath();
    ctx.moveTo(0, -robotR * 1.1 - 4);
    ctx.lineTo(-4, -robotR * 1.1 + 3);
    ctx.lineTo(4, -robotR * 1.1 + 3);
    ctx.closePath();
    ctx.fill();

    ctx.restore();
  }

  private drawHud(w: number, h: number) {
    const ctx = this.ctx;

    // Coordinate & status overlay top-right
    ctx.font = '10px monospace';
    ctx.fillStyle = 'rgba(255, 255, 255, 0.45)';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'top';
    const yawDeg = Math.round((this.robotYawRad * 180) / Math.PI);
    ctx.fillText(`X:${this.robotX.toFixed(2)} Y:${this.robotY.toFixed(2)} θ:${yawDeg}°`, w - 8, 6);

    // Scale reference bar bottom-right (0.5m)
    const barWidthPx = 0.5 * this.scale;
    const barX = w - barWidthPx - 10;
    const barY = h - 14;

    ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(barX, barY);
    ctx.lineTo(barX + barWidthPx, barY);
    ctx.moveTo(barX, barY - 3);
    ctx.lineTo(barX, barY + 3);
    ctx.moveTo(barX + barWidthPx, barY - 3);
    ctx.lineTo(barX + barWidthPx, barY + 3);
    ctx.stroke();

    ctx.font = '9px system-ui, sans-serif';
    ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    ctx.fillText('0.5 m', barX + barWidthPx / 2, barY - 2);
  }

  dispose() {
    if (this.animationId !== null) {
      cancelAnimationFrame(this.animationId);
      this.animationId = null;
    }
    for (const { target, event, fn } of this.boundListeners) {
      target.removeEventListener(event, fn);
    }
    this.boundListeners = [];
  }
}
