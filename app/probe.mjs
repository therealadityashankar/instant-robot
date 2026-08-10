// Drive a real pick in the browser and report what the arm actually did.
import { chromium } from 'playwright-core';

const URL = process.env.APP_URL ?? 'http://localhost:5174/';
const OUT = '/private/tmp/claude-501/-Users-river-Fun-instant-robot/80ea3c92-299f-4760-afd1-b3e958afb776/scratchpad';
const TAG = process.env.TAG ?? 'run';

const browser = await chromium.launch({
  channel: 'chrome',
  headless: true,
  args: ['--use-gl=angle', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
});
const page = await browser.newPage({ viewport: { width: 1500, height: 1100 } });
const bad = [];
page.on('pageerror', (e) => bad.push(`pageerror: ${e.message}`));
page.on('console', (m) => { if (m.type() === 'error') bad.push(m.text()); });

await page.goto(URL, { waitUntil: 'load', timeout: 60000 });
await page.waitForTimeout(7000);
await page.locator('button', { hasText: 'Manually control arm' }).first().click();
await page.waitForTimeout(600);
// The step buttons live in a collapsed <details>; open every one.
await page.$$eval('details', (ds) => ds.forEach((d) => (d.open = true)));
await page.waitForTimeout(400);

const click = async (label) => {
  const b = page.locator('button', { hasText: label }).first();
  if (!(await b.count())) { console.log(`!! no button "${label}"`); return false; }
  await b.click();
  return true;
};
/** Wait until every pick button is enabled again (armPickBusy false). */
const idle = async (ms = 90000) => {
  await page.waitForFunction(
    () => {
      const b = [...document.querySelectorAll('button')].find((x) => x.textContent.includes('Approach'));
      return b && !b.disabled;
    },
    { timeout: ms },
  ).catch(() => console.log('!! timed out waiting for idle'));
};
const status = async () => {
  const s = await page.locator('.status').allTextContents();
  return s.map((t) => t.trim().replace(/\s+/g, ' ')).filter(Boolean);
};

// The station sits 0.9 m away — out of a fixed arm's reach — so put the block on
// the floor in front of it instead. BX/BY/BYAW override for repeatability.
const set = async (sel, v) => {
  await page.locator(sel).fill(String(v));
  await page.locator(sel).dispatchEvent('change');
  await page.locator(sel).dispatchEvent('input');
};
await set('#sbx', process.env.BX ?? 0.26);
await set('#sby', process.env.BY ?? 0.02);
await set('#sbyaw', process.env.BYAW ?? 25);
await page.waitForTimeout(2500); // let it fall and settle
console.log('block at', await page.locator('#sbx').inputValue(), await page.locator('#sby').inputValue(),
            'yaw', await page.locator('#sbyaw').inputValue());

console.log('\n--- 1 · Raise to view ---');
await click('1 · Raise to view');
await idle();
await page.waitForTimeout(800);

console.log('--- 2 · Approach ---');
await click('2 · Approach');
await idle();
await page.waitForTimeout(500);
for (const s of await status()) console.log('  ', s);
await page.screenshot({ path: `${OUT}/${TAG}-hover.png` });

console.log('\n--- 3 · Grasp & lift ---');
await click('3 · Grasp & lift');
await idle();
await page.waitForTimeout(500);
for (const s of await status()) console.log('  ', s);
await page.screenshot({ path: `${OUT}/${TAG}-grasp.png` });

// The collapsed per-pick log holds the blow-by-blow.
const det = page.locator('details').first();
if (await det.count()) {
  await det.evaluate((d) => (d.open = true));
  const log = (await det.textContent()) ?? '';
  console.log('\n--- pick log ---');
  console.log(log.replace(/\s*\n\s*/g, '\n').trim().slice(0, 3000));
}
console.log('\nerrors:', bad.length ? bad.slice(0, 5) : 'none');
await browser.close();
