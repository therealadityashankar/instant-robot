<script lang="ts">
  // 3-direction drive for the LeKiwi omniwheel base. The base has three natural
  // drive directions 120° apart, each spinning one pair of wheels: forward,
  // back-left (+120°) and back-right (−120°). Emits `onmove(fwd, bl, br)` each
  // frame — the amount of each primitive held (0 or 1). Rotation is separate (Q/E).
  import { onMount } from 'svelte';

  let { onmove }: { onmove: (fwd: number, bl: number, br: number) => void } = $props();

  type Dir = 'fwd' | 'bl' | 'br';
  const KEY: Record<string, Dir> = { w: 'fwd', a: 'bl', d: 'br' };
  let held = $state<Record<Dir, boolean>>({ fwd: false, bl: false, br: false });

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
      if (editable(e.target)) return;
      const d = KEY[e.key.toLowerCase()];
      if (d) { held[d] = true; e.preventDefault(); }
    };
    const ku = (e: KeyboardEvent) => {
      const d = KEY[e.key.toLowerCase()];
      if (d) { held[d] = false; e.preventDefault(); }
    };
    const reset = () => { held = { fwd: false, bl: false, br: false }; };
    window.addEventListener('keydown', kd);
    window.addEventListener('keyup', ku);
    window.addEventListener('blur', reset);
    let raf = 0;
    const tick = () => {
      onmove(held.fwd ? 1 : 0, held.bl ? 1 : 0, held.br ? 1 : 0);
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

<div class="triad">
  <button
    class="dir fwd"
    class:on={held.fwd}
    aria-label="Drive forward"
    title="W — forward"
    onpointerdown={(e) => press('fwd', e)}
    onpointerup={() => release('fwd')}
    onpointerleave={() => release('fwd')}
    onpointercancel={() => release('fwd')}
  >▲<small>W fwd</small></button>
  <div class="backs">
    <button
      class="dir"
      class:on={held.bl}
      aria-label="Drive back-left"
      title="A — back-left (+120°)"
      onpointerdown={(e) => press('bl', e)}
      onpointerup={() => release('bl')}
      onpointerleave={() => release('bl')}
      onpointercancel={() => release('bl')}
    >↙<small>A bk-L</small></button>
    <button
      class="dir"
      class:on={held.br}
      aria-label="Drive back-right"
      title="D — back-right (−120°)"
      onpointerdown={(e) => press('br', e)}
      onpointerup={() => release('br')}
      onpointerleave={() => release('br')}
      onpointercancel={() => release('br')}
    >↘<small>D bk-R</small></button>
  </div>
  <span class="cap">W · A · D</span>
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
  .cap { font-size: 0.68rem; color: var(--text-soft, #999); }
</style>
