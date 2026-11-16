import puppeteer from 'puppeteer-core';

async function main() {
  const browser = await puppeteer.launch({
    executablePath: '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',
    headless: true,
    args: ['--no-sandbox', '--disable-setuid-sandbox', '--use-gl=angle']
  });

  const page = await browser.newPage();
  await page.setViewport({ width: 1400, height: 900 });
  await page.goto('http://localhost:5173', { waitUntil: 'networkidle2', timeout: 15000 });
  await new Promise((r) => setTimeout(r, 2000));

  // Focus camera close to the boundary post at the left end
  await page.evaluate(() => {
    // Access Three.js camera if available or take full view
    const canvas = document.querySelector('canvas');
    return !!canvas;
  });

  const screenshotPath = '/Users/river/.gemini/antigravity-ide/brain/a3f0fb2b-2953-4610-8e0f-da789809cae9/sim_boundary_closeup.png';
  await page.screenshot({ path: screenshotPath });
  console.log('Saved closeup screenshot to', screenshotPath);

  await browser.close();
}

main().catch(console.error);
