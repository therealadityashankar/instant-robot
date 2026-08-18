#!/usr/bin/env node
import { createRequire } from 'module';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const require = createRequire(path.join(__dirname, '../app/package.json'));
const puppeteer = require('puppeteer-core');

const CHROME_PATH = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';
const TARGET_URL = 'http://localhost:5173/';
const ROOM_CODE = 'my-lekiwi@vmwux3wgc8';

async function runTest() {
  console.log('🚀 Launching Chrome at:', CHROME_PATH);
  const browser = await puppeteer.launch({
    executablePath: CHROME_PATH,
    headless: 'new',
    args: [
      '--no-sandbox',
      '--disable-setuid-sandbox',
      '--use-fake-ui-for-media-stream',
      '--use-fake-device-for-media-stream',
      '--autoplay-policy=no-user-gesture-required',
    ],
  });

  const page = await browser.newPage();
  await page.setViewport({ width: 1400, height: 900 });

  page.on('console', (msg) => {
    const text = msg.text();
    if (
      text.includes('[WebRTC') ||
      text.includes('[Camera') ||
      text.includes('Rx OK') ||
      text.includes('active camera') ||
      text.includes('switching') ||
      text.includes('Switching') ||
      text.includes('error') ||
      text.includes('Error')
    ) {
      console.log(`[Browser Console ${msg.type()}]:`, text);
    }
  });

  page.on('pageerror', (err) => {
    console.error('❌ [Page Error]:', err.message);
  });

  console.log('🌐 Navigating to:', TARGET_URL);
  await page.goto(TARGET_URL, { waitUntil: 'networkidle0', timeout: 15000 });
  await new Promise((r) => setTimeout(r, 1000));

  console.log('🔘 Clicking "Connect robot" button to open modal...');
  await page.evaluate(() => {
    const btns = Array.from(document.querySelectorAll('button'));
    const btn = btns.find((b) => b.textContent && b.textContent.includes('Connect robot'));
    if (btn) btn.click();
  });

  console.log('⏳ Waiting for #connect-combined...');
  await page.waitForSelector('#connect-combined', { timeout: 5000 });

  console.log(`🔑 Typing room code: ${ROOM_CODE}`);
  const input = await page.$('#connect-combined');
  await input.click({ clickCount: 3 });
  await input.type(ROOM_CODE);
  await new Promise((r) => setTimeout(r, 300));

  console.log('🔘 Clicking Connect in modal...');
  const connectRemoteBtn = await page.$('.connectbtn');
  await connectRemoteBtn.click();

  console.log('⏳ Waiting for WebRTC connection (5s)...');
  await new Promise((r) => setTimeout(r, 5000));

  // Check camera controls
  console.log('📸 Searching for camera switch buttons...');
  const camBtns = await page.evaluate(() => {
    return Array.from(document.querySelectorAll('.camctl button')).map((b) => b.textContent?.trim());
  });
  console.log('Found .camctl buttons:', camBtns);

  console.log('🔘 Switching to Arm camera...');
  const switchedToArm = await page.evaluate(() => {
    const btns = Array.from(document.querySelectorAll('.camctl button'));
    const armBtn = btns.find((b) => b.textContent && (b.textContent.includes('Arm') || b.textContent.includes('Switch to Arm')));
    if (armBtn) {
      armBtn.click();
      return armBtn.textContent?.trim();
    }
    return null;
  });
  console.log('Clicked Arm button:', switchedToArm);

  console.log('⏳ Waiting 4s for Arm camera stream...');
  await new Promise((r) => setTimeout(r, 4000));

  console.log('🔘 Switching back to Base camera...');
  const switchedToBase = await page.evaluate(() => {
    const btns = Array.from(document.querySelectorAll('.camctl button'));
    const baseBtn = btns.find((b) => b.textContent && (b.textContent.includes('Base') || b.textContent.includes('Switch to Base')));
    if (baseBtn) {
      baseBtn.click();
      return baseBtn.textContent?.trim();
    }
    return null;
  });
  console.log('Clicked Base button:', switchedToBase);

  console.log('⏳ Waiting 4s for Base camera stream...');
  await new Promise((r) => setTimeout(r, 4000));

  console.log('✅ Automated test execution finished successfully.');
  await browser.close();
}

runTest().catch((err) => {
  console.error('❌ Test failed:', err);
  process.exit(1);
});
