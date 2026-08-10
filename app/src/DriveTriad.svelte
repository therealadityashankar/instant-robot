<script lang="ts">
  // Drive for the LeKiwi omniwheel base. Three primitive directions 120° apart —
  // forward, back-left (+120°), back-right (−120°) — each drives one pair of wheels.
  // Holding Shift (or the Reverse toggle) negates them, giving all 6 directions
  // (backward, front-right, front-left). Emits `onmove(fwd, bl, br)` each frame,
  // each −1/0/+1. Rotation is separate (Q/E).
  import { onMount } from 'svelte';

  let { onmove }: { onmove: (fwd: number, bl: number, br: number) => void } = $props();

  type Dir = 'fwd' | 'bl' | 'br';
  const KEY: Record<string, Dir> = { w: 'fwd', a: 'bl', d: 'br' };
  let held = $state<Record<Dir, boolean>>({ fwd: false, bl: false, br: false });
  let shiftKey = $state(false); // Shift held on the keyboard
  let toggleRev = $state(false); // on-screen reverse toggle (for touch/mouse)
  const reverse = $derived(shiftKey || toggleRev);

  function editable(t: EventTarget | null) {
    const el = t as HTMLElement | null;
    return !!el && (el.isContentEditable || ['INPUT', 'TEXTAREA', 'SELECT'].includes(el.tagName));
  }
  function press(d: Dir, e: PointerEvent) {
    e.preventDefault();
    (e.currentTarget as HTMLElement).setPointerCapture?.(e.pointerId);
    held[d] = true;
  }
  const release = (d: Dir) => { held[d] = false; };

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

<div class="triad" class:rev={reverse}>
  <button
    class="dir fwd"
    class:on={held.fwd}
    aria-label="Drive forward"
    title="W — forward (Shift: backward)"
    onpointerdown={(e) => press('fwd', e)}
    onpointerup={() => release('fwd')}
    onpointerleave={() => release('fwd')}
    onpointercancel={() => release('fwd')}
  >{reverse ? '▼' : '▲'}<small>W {reverse ? 'back' : 'fwd'}</small></button>
  <div class="backs">
    <button
      class="dir"
      class:on={held.bl}
      aria-label="Drive back-left"
      title="A — back-left +120° (Shift: front-right)"
      onpointerdown={(e) => press('bl', e)}
      onpointerup={() => release('bl')}
      onpointerleave={() => release('bl')}
      onpointercancel={() => release('bl')}
    >{reverse ? '↗' : '↙'}<small>A {reverse ? 'fr-R' : 'bk-L'}</small></button>
    <button
      class="dir"
      class:on={held.br}
      aria-label="Drive back-right"
      title="D — back-right −120° (Shift: front-left)"
      onpointerdown={(e) => press('br', e)}
      onpointerup={() => release('br')}
      onpointerleave={() => release('br')}
      onpointercancel={() => release('br')}
    >{reverse ? '↖' : '↘'}<small>D {reverse ? 'fr-L' : 'bk-R'}</small></button>
  </div>
  <button class="revtoggle" class:on={toggleRev} onclick={() => (toggleRev = !toggleRev)}>
    reverse (Shift){toggleRev ? ' ✓' : ''}
  </button>
</div>

<style>
  .triad { display: flex; flex-direction: column; align-items: center; gap: 0.2rem; }
  .backs { display: flex; gap: 1.4rem; }
  .dir {
    width: 2.7rem;
    height: 2.7rem;
    border-radius: 8px;
    border: 1px solid var(--line, #888);
    background: color-mix(in srgb, var(--bg, #222) 82%, #888);
    color: var(--text, #eee);
    font-size: 1.1rem;
    line-height: 1;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    cursor: pointer;
    touch-action: none;
    user-select: none;
    transition: background 80ms;
  }
  .dir small { font-size: 0.5rem; opacity: 0.7; margin-top: 2px; }
  .dir.on { background: var(--knob, #e8e8e8); color: #111; }
  .triad.rev .dir { border-color: #d08770; }
  .revtoggle {
    font-size: 0.6rem;
    padding: 1px 6px;
    border-radius: 6px;
    border: 1px solid var(--line, #888);
    background: transparent;
    color: var(--text-soft, #999);
    cursor: pointer;
  }
  .revtoggle.on { background: #d08770; color: #111; border-color: #d08770; }
</style>
