// End-to-end pick, driven in a real browser.
//
// Loads the LeKiwi scene, scatters the block on the station, clicks "Run
// everything", and reports what each stage did. The point is to exercise the
// hand-offs — explore → drive → look → approach → grasp — which is where runs
// break, and which no unit test can reach: they depend on rendered pixels going
// through the real detector.
//
//   node e2e.mjs                 one run
//   RUNS=5 node e2e.mjs          five runs, each with the block somewhere new
//   HEADED=1 node e2e.mjs        watch it happen
import { chromium } from 'playwright-core';

const URL = process.env.APP_URL ?? 'http://localhost:5174/';
const OUT = process.env.OUT ?? '/private/tmp/claude-501/-Users-river-Fun-instant-robot/80ea3c92-299f-4760-afd1-b3e958afb776/scratchpad';
const RUNS = +(process.env.RUNS ?? 1);
const STEP_TIMEOUT = +(process.env.STEP_TIMEOUT ?? 240000);

const browser = await chromium.launch({
  channel: 'chrome',
  headless: !process.env.HEADED,
  args: ['--use-gl=angle', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
});
const page = await browser.newPage({ viewport: { width: 1500, height: 1100 } });
const errors = [];
page.on('pageerror', (e) => errors.push(`pageerror: ${e.message}`));
page.on('console', (m) => { if (m.type() === 'error' && !m.text().includes('404')) errors.push(m.text()); });

const runStep = () => page.locator('[data-run-step]').textContent().catch(() => null);

await page.goto(URL, { waitUntil: 'load', timeout: 60000 });
await page.waitForTimeout(4000);

// The mobile base is what makes explore/drive meaningful.
await page.selectOption('select[aria-label="robot"]', 'lekiwi');
await page.waitForTimeout(12000); // base meshes + scene rebuild
await page.locator('button', { hasText: 'Manually control arm' }).first().click().catch(() => {});
await page.waitForTimeout(500);
await page.$$eval('details', (ds) => ds.forEach((d) => (d.open = true)));

const results = [];
for (let run = 1; run <= RUNS; run++) {
  console.log(`\n════════ run ${run}/${RUNS} ════════`);
  // Each run starts from the same place: the previous one leaves the base parked
  // wherever it crept to, which quietly makes run 2 a different experiment.
  await page.locator('button', { hasText: 'Centre' }).first().click().catch(() => {});
  await page.waitForTimeout(1500);
  await page.locator('button', { hasText: 'Randomise block' }).first().click().catch(() => {});
  await page.waitForTimeout(2000);

  const seen = new Set();
  const t0 = Date.now();
  await page.locator('button', { hasText: 'Run everything' }).first().click();

  // Follow the stage line until the button frees up again.
  let done = false;
  while (Date.now() - t0 < STEP_TIMEOUT) {
    const s = (await runStep())?.trim();
    if (s && !seen.has(s)) {
      seen.add(s);
      console.log(`  ${((Date.now() - t0) / 1000).toFixed(1).padStart(5)}s  ${s}`);
    }
    const busy = await page
      .locator('button', { hasText: 'Running…' })
      .count()
      .catch(() => 0);
    if (!busy && Date.now() - t0 > 3000) { done = true; break; }
    await page.waitForTimeout(300);
  }
  const final = (await runStep())?.trim() ?? '(no status)';
  if (!done) console.log('  !! timed out');
  console.log(`  final: ${final}`);
  const log = await page.locator('pre.logtext').first().textContent().catch(() => null);
  if (log && log.trim() !== 'nothing yet') {
    console.log('  --- pick log ---');
    for (const line of log.trim().split('\n')) console.log(`   ${line}`);
  }
  results.push({ run, secs: ((Date.now() - t0) / 1000).toFixed(0), final });
  await page.screenshot({ path: `${OUT}/e2e-run${run}.png` });
}

console.log('\n════════ summary ════════');
for (const r of results) console.log(` run ${r.run}  ${r.secs.padStart(4)}s  ${r.final}`);
const ok = results.filter((r) => /lifted/i.test(r.final)).length;
console.log(`\npicked up: ${ok}/${results.length}`);
console.log('errors:', errors.length ? [...new Set(errors)].slice(0, 6) : 'none');

await browser.close();
