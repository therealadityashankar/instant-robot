<script lang="ts">
  // Left/right rotation panel.
  // A split circle: hold ↺ / Q to turn left, ↻ / E to turn right.
  // Solid black icons and letters at the bottom inside each half circle.
  import { onMount } from 'svelte';

  let {
    onrotate,
    turnForce,
  }: {
    onrotate: (direction: number) => void;
    turnForce?: number;
  } = $props();

  let dir = $state(0);
  const pressed = { q: false, e: false };

  const activeDir = $derived(dir !== 0 ? dir : (turnForce ?? 0));

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
      class:on={activeDir > 0.05}
      aria-label="Rotate robot left (Q)"
      title="Q — rotate left"
      onpointerdown={(e) => press(1, e)}
      onpointerup={release}
      onpointerleave={release}
      onpointercancel={release}
    >
      <span class="icon">↺</span>
      <span class="key">Q</span>
    </button>
    <button
      class="half"
      class:on={activeDir < -0.05}
      aria-label="Rotate robot right (E)"
      title="E — rotate right"
      onpointerdown={(e) => press(-1, e)}
      onpointerup={release}
      onpointerleave={release}
      onpointercancel={release}
    >
      <span class="icon">↻</span>
      <span class="key">E</span>
    </button>
  </div>
</div>

<style>
  .rotwrap {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    user-select: none;
    touch-action: none;
  }
  .ring {
    display: grid;
    grid-template-columns: 1fr 1fr;
    width: 4.4rem;
    height: 4.4rem;
    border-radius: 50%;
    border: 1.8px solid #0f172a;
    overflow: hidden;
    filter: drop-shadow(0 2px 6px rgba(0, 0, 0, 0.35));
  }
  .half {
    width: 100%;
    height: 100%;
    min-width: 0;
    margin: 0;
    padding: 0.3rem 0;
    border: none;
    border-radius: 0;
    box-sizing: border-box;
    background: #f1f5f9;
    color: #0f172a;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 0.15rem;
    cursor: pointer;
    touch-action: none;
    user-select: none;
    transition: background 80ms;
  }
  .half:first-child { border-right: 1.5px solid #0f172a; }
  .half:hover { background: #e2e8f0; }
  .half.on { background: #f59e0b; color: #0f172a; }
  .icon {
    font-size: 1.25rem;
    font-weight: 700;
    line-height: 1;
    color: #0f172a;
  }
  .key {
    font-size: 0.72rem;
    font-weight: 600;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    line-height: 1;
    color: #0f172a;
  }
</style>
