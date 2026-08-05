<script lang="ts">
  // Translation joystick — ported from roboter/src/components/MovementControl.tsx.
  // Drag the knob (or WASD / arrows) to strafe. Emits a velocity each animation
  // frame via `onmove(forward, strafe)`, both in [-1, 1]. (Rotation is a separate
  // control — see RotationControl.svelte.)
  import { onMount } from 'svelte';

  let { onmove }: { onmove: (forward: number, strafe: number) => void } = $props();

  let pad = $state<HTMLDivElement>();
  let knob = $state<HTMLDivElement>();
  let stick = $state({ x: 0, z: 0 });
  let keys = $state({ x: 0, z: 0 });
  let pointerId: number | null = null;

  function clamp(x: number, z: number) {
    const l = Math.hypot(x, z);
    return l <= 1 || l === 0 ? { x, z } : { x: x / l, z: z / l };
  }
  const control = $derived(clamp(stick.x + keys.x, stick.z + keys.z));

  $effect(() => {
    if (!pad || !knob) return;
    const r = pad.clientWidth / 2;
    const kr = knob.clientWidth / 2;
    const a = Math.max(r - kr, 0);
    knob.style.transform = `translate(calc(-50% + ${control.x * a}px), calc(-50% + ${-control.z * a}px))`;
  });

  function fromEvent(e: PointerEvent) {
    if (!pad) return;
    const rect = pad.getBoundingClientRect();
    let ox = e.clientX - rect.left - rect.width / 2;
    let oy = e.clientY - rect.top - rect.height / 2;
    const r = rect.width / 2;
    const d = Math.hypot(ox, oy);
    if (d > r) { ox *= r / d; oy *= r / d; }
    stick = clamp(ox / r, -oy / r);
  }
  function down(e: PointerEvent) {
    pointerId = e.pointerId;
    (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
    if (knob) knob.style.transition = 'none';
    fromEvent(e);
  }
  function move(e: PointerEvent) {
    if (pointerId !== e.pointerId) return;
    fromEvent(e);
  }
  function up(e: PointerEvent) {
    if (pointerId !== e.pointerId) return;
    pointerId = null;
    if (knob) knob.style.transition = '';
    stick = { x: 0, z: 0 };
  }

  function editable(t: EventTarget | null) {
    const el = t as HTMLElement | null;
    return !!el && (el.isContentEditable || ['INPUT', 'TEXTAREA', 'SELECT'].includes(el.tagName));
  }
  function key(e: KeyboardEvent, pressed: boolean) {
    if (editable(e.target)) return;
    const k = e.key.toLowerCase();
    const dir: Record<string, [number, number]> = {
      w: [0, 1], arrowup: [0, 1], s: [0, -1], arrowdown: [0, -1],
      a: [-1, 0], arrowleft: [-1, 0], d: [1, 0], arrowright: [1, 0],
    };
    const v = dir[k];
    if (!v) return;
    e.preventDefault();
    if (v[0]) keys = { ...keys, x: pressed ? v[0] : 0 };
    if (v[1]) keys = { ...keys, z: pressed ? v[1] : 0 };
  }

  onMount(() => {
    const kd = (e: KeyboardEvent) => key(e, true);
    const ku = (e: KeyboardEvent) => key(e, false);
    const reset = () => { keys = { x: 0, z: 0 }; };
    window.addEventListener('keydown', kd);
    window.addEventListener('keyup', ku);
    window.addEventListener('blur', reset);
    let raf = 0;
    const tick = () => {
      onmove(control.z, control.x);
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

<div class="joywrap">
  <div
    class="pad"
    bind:this={pad}
    role="application"
    aria-label="Drive the robot — drag or WASD"
    title="Drag or WASD to move"
    onpointerdown={down}
    onpointermove={move}
    onpointerup={up}
    onpointercancel={up}
    onpointerleave={up}
  >
    <span class="arr t">↑</span>
    <span class="arr b">↓</span>
    <span class="arr l">←</span>
    <span class="arr r">→</span>
    <div class="knob" bind:this={knob}></div>
  </div>
  <span class="cap">drag / WASD</span>
</div>

<style>
  .joywrap { display: flex; flex-direction: column; align-items: center; gap: 0.15rem; }
  .pad {
    position: relative;
    width: 4rem;
    height: 4rem;
    border-radius: 50%;
    border: 1px solid var(--line, #888);
    background: color-mix(in srgb, var(--bg, #222) 85%, #888);
    cursor: pointer;
    touch-action: none;
    user-select: none;
  }
  .knob {
    position: absolute;
    top: 50%;
    left: 50%;
    width: 2.3rem;
    height: 2.3rem;
    border-radius: 50%;
    background: var(--knob, #e8e8e8);
    border: 1px solid #000;
    transform: translate(-50%, -50%);
    transition: transform 75ms ease-out;
  }
  .arr { position: absolute; font-size: 0.7rem; color: var(--text-soft, #999); }
  .arr.t { top: 1px; left: 50%; transform: translateX(-50%); }
  .arr.b { bottom: 1px; left: 50%; transform: translateX(-50%); }
  .arr.l { left: 2px; top: 50%; transform: translateY(-50%); }
  .arr.r { right: 2px; top: 50%; transform: translateY(-50%); }
  .cap { font-size: 0.68rem; color: var(--text-soft, #999); }
</style>
