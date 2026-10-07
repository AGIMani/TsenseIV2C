// Both A32 models load real meshes and run the web lifecycle without hardware.
import {chromium} from '@playwright/test';
import assert from 'node:assert/strict';
import {mkdir} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';

const base = process.env.WEB_URL || 'http://127.0.0.1:8872';
const output = fileURLToPath(new URL('../../../logs/a32_integration_qa/', import.meta.url));
await mkdir(output, {recursive: true});
const browser = await chromium.launch({headless: true,
  executablePath: process.env.CHROME_PATH || 'C:/Program Files/Google/Chrome/Application/chrome.exe',
  args: ['--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream']});
const errors = [];
const page = await browser.newPage({viewport: {width: 1440, height: 1100}});
page.on('pageerror', error => errors.push(error.message));
try {
  for (const model of ['TsenseI_v3_2', 'TsenseI_v3_2_withid']) {
    await page.goto(`${base}/?model=${model}`);
    await page.waitForFunction(() => !document.getElementById('start').disabled);
    assert.equal(await page.locator('#hand-model').inputValue(), model);
    assert.equal(await page.locator('#model-stage canvas').count(), 1);
    assert(await page.locator('#a32-sdk-panel').isVisible());
    assert(await page.locator('#hardware-panel').isHidden());
    assert.equal(await page.locator('#collision-mode').inputValue(), 'stop');
    await page.screenshot({path: `${output}/${model}-left.png`, fullPage: true});
    // A local session owns its SDK instance, but creation is preview-only.
    const lifecycle = await page.evaluate(async model => {
      const post = async (path, data) => {
        const r = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(data)});
        const result = await r.json();
        if (!r.ok) throw new Error(result.error);
        return result;
      };
      const created = await post('/api/session', {model_id: model, side: 'left', recording: false, collision_mode: 'stop'});
      const status = await post('/api/a32/control', {session: created.session, action: 'status'});
      const frame = await post('/api/frame', {session: created.session, sequence: 0, timestamp_ms: 1,
        capture_unix_ms: Date.now(), camera_size: [640, 480], hand: null});
      await post('/api/stop', {session: created.session});
      return {created, status, frame};
    }, model);
    assert.equal(lifecycle.status.connected, false);
    assert.equal(lifecycle.status.armed, false);
    assert.equal(lifecycle.frame.tracking, false);
    assert.equal(Object.keys(lifecycle.created.pose).length, 20);
    if (await page.locator('#recording').isChecked()) await page.locator('label:has(#recording)').click();
    await page.locator('#start').click();
    await page.waitForFunction(() => document.getElementById('camera-state').textContent === '实时', null, {timeout: 60000});
    await page.waitForFunction(() => Number(document.getElementById('fps').textContent) > 0, null, {timeout: 30000});
    await page.locator('#start').click();
    await page.waitForFunction(() => document.getElementById('camera-state').textContent === '已停止');
    assert(await page.evaluate(() => document.getElementById('video').srcObject === null));
    await page.locator('#side').selectOption('right');
    await page.waitForFunction(() => !document.getElementById('start').disabled);
    assert(await page.locator('#a32-connect').isDisabled());
    assert.match(await page.locator('#a32-sdk-state').innerText(), /右手/);
    await page.screenshot({path: `${output}/${model}-right.png`, fullPage: true});
    await page.setViewportSize({width: 390, height: 844});
    assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
    await page.setViewportSize({width: 1440, height: 1100});
  }
  assert.deepEqual(errors, []);
  console.log('A32_BROWSER_OK: both models, real meshes, left/right, SDK panel, camera inference/frame/stop, session, missing-hand hold, mobile layout');
} finally { await browser.close(); }
