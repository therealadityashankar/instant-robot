<script lang="ts">
  // Left/right rotation panel — ported from roboter/src/components/RotationControl.tsx.
  // A split circle: hold ⟲ / ⟳ (or Q / E) to turn. Emits `onrotate(direction)`
  // (−1, 0, or 1) each animation frame while held.
  import { onMount } from 'svelte';

  let { onrotate }: { onrotate: (direction: number) => void } = $props();

  let dir = $state(0);
  const pressed = { q: false, e: false };

  function recompute() {
    dir = pressed.q && !pressed.e ? 1 : pressed.e && !pressed.q ? -1 : 0;
  }
  function editable(t: EventTarget | null) {
    const el = t as HTMLElement | null;
    return !!el && (el.isContentEditable || ['INPUT', 'TEXTAREA', 'SELECT'].includes(el.tagName));
  }

  onMount(() => {
    const kd = (e: KeyboardEvent) => {
      if (editable(e.target)) return;
      const k = e.key.toLowerCase();
      if (k === 'q' && !pressed.q) { pressed.q = true; recompute(); e.preventDefault(); }
      else if (k === 'e' && !pressed.e) { pressed.e = true; recompute(); e.preventDefault(); }
    };
    const ku = (e: KeyboardEvent) => {
      if (editable(e.target)) return;
      const k = e.key.toLowerCase();
      if (k === 'q' && pressed.q) { pressed.q = false; recompute(); e.preventDefault(); }
      else if (k === 'e' && pressed.e) { pressed.e = false; recompute(); e.preventDefault(); }
    };
    const reset = () => { pressed.q = false; pressed.e = false; dir = 0; };
    window.addEventListener('keydown', kd);
    window.addEventListener('keyup', ku);
    window.addEventListener('blur', reset);
    let raf = 0;
    const tick = () => { onrotate(dir); raf = requestAnimationFrame(tick); };
    tick();
    return () => {
      window.removeEventListener('keydown', kd);
      window.removeEventListener('keyup', ku);
      window.removeEventListener('blur', reset);
      cancelAnimationFrame(raf);
    };
  });

  function press(d: number, e: PointerEvent) {
    e.preventDefault();
    (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
    dir = d;
  }
  function release(e: PointerEvent) {
    if ((e.currentTarget as HTMLElement).hasPointerCapture?.(e.pointerId))
      (e.currentTarget as HTMLElement).releasePointerCapture(e.pointerId);
    dir = 0;
  }
</script>

<div class="rotwrap">
  <div class="ring">
    <button
      class="half"
      class:on={dir > 0}
      aria-label="Rotate robot left"
      title="Q"
      onpointerdown={(e) => press(1, e)}
      onpointerup={release}
      onpointerleave={release}
      onpointercancel={release}
    >↺</button>
    <button
      class="half"
      class:on={dir < 0}
      aria-label="Rotate robot right"
      title="E"
      onpointerdown={(e) => press(-1, e)}
      onpointerup={release}
      onpointerleave={release}
      onpointercancel={release}
    >↻</button>
  </div>
  <span class="cap">rotate · Q,E</span>
</div>

<style>
  .rotwrap { display: flex; flex-direction: column; align-items: center; gap: 0.15rem; }
  .ring {
    display: grid;
    grid-template-columns: 1fr 1fr;
    width: 4rem;
    height: 4rem;
    border-radius: 50%;
    border: 1px solid var(--line, #888);
    overflow: hidden;
  }
  .half {
    width: 100%;
    height: 100%;
    min-width: 0;
    margin: 0;
    padding: 0;
    border: none;
    border-radius: 0;
    box-sizing: border-box;
    background: color-mix(in srgb, var(--bg, #222) 82%, #888);
    color: var(--text, #eee);
    font-size: 1.4rem;
    line-height: 1;
    cursor: pointer;
    touch-action: none;
    user-select: none;
    transition: background 80ms;
  }
  .half:first-child { border-right: 1px solid var(--line, #888); }
  .half.on { background: var(--knob, #e8e8e8); color: #111; }
  .cap { font-size: 0.68rem; color: var(--text-soft, #999); }
</style>
