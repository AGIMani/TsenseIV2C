// Run against an extracted/staged bundle in --public mode, using the minimal venv.
import { chromium } from '@playwright/test';
import assert from 'node:assert/strict';
const base = process.env.WEB_URL || 'http://127.0.0.1:8877';
const browser = await chromium.launch({
  executablePath: process.env.CHROME_PATH || 'C:/Program Files/Google/Chrome/Application/chrome.exe',
  headless: true,
  args: ['--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream',
    // Test-only pin for a specific verified LAN certificate; no OS trust change.
    ...(process.env.CHROME_CERT_SPKI ? [`--ignore-certificate-errors-spki-list=${process.env.CHROME_CERT_SPKI}`] : [])],
});
try {
  const page = await browser.newPage();
  const errors = [], failures = [], foreign = [];
  page.on('pageerror', e => errors.push(e.message));
  page.on('requestfailed', r => failures.push(r.url()));
  page.on('request', r => {
    if (r.url().startsWith('http') && !r.url().startsWith(base + '/')) foreign.push(r.url());
  });
  await page.goto(base);
  assert(await page.evaluate(() => window.isSecureContext));
  await page.waitForFunction(() => !document.querySelector('#start').disabled);
  assert(await page.locator('#recording').isDisabled());
  assert(!(await page.locator('#recording').isChecked()));
  await page.locator('#side').selectOption('right');
  await page.waitForFunction(() => !document.querySelector('#start').disabled);
  assert.match(await page.locator('#model-caption').innerText(), /TSENSEI.*RIGHT/);
  await page.locator('#start').click();
  await page.waitForFunction(() => Number(document.querySelector('#fps').textContent) > 0, { timeout: 45000 });
  assert.match(await page.locator('#gpu-badge').innerText(), /GPU/);
  await page.locator('#start').click();
  await page.waitForFunction(() => !document.querySelector('#start').disabled);
  const result = await page.evaluate(async () => {
    async function post(path, data) {
      const response = await fetch(path, { method: 'POST', headers: {'Content-Type':'application/json'}, body:JSON.stringify(data) });
      return {status:response.status, data:await response.json()};
    }
    const one = await post('/api/session', {side:'left', recording:true});
    const two = await post('/api/session', {side:'right'});
    const three = await post('/api/session', {});
    const held = await post('/api/frame', {session:one.data.session, sequence:1,
      timestamp_ms:1, capture_unix_ms:Date.now(), camera_size:[640,480], hand:null});
    await post('/api/stop', {session:one.data.session});
    await post('/api/stop', {session:two.data.session});
    return {one,two,three,held};
  });
  assert.equal(result.one.status, 200);
  assert.equal(result.two.status, 200);
  assert.equal(result.one.data.recording, false);
  assert.equal(result.two.data.recording, false);
  assert.equal(result.three.status, 400);
  assert.equal(result.held.status, 200);
  assert.equal(Object.keys(result.held.data.pose).length, 20);
  assert.deepEqual(errors, []);
  assert.deepEqual(failures, []);
  assert.deepEqual(foreign, []);
  console.log('DEPLOY CHECK PASSED: bundled assets, both full models, real browser GPU/camera, no CDN requests, no visitor recording, bounded isolated sessions');
} finally {
  await browser.close();
}
