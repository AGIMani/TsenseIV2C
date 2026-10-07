// Browser integration against a local test service; never connects hardware.
import {chromium} from '../online_motion_track/node_modules/@playwright/test/index.mjs';
import assert from 'node:assert/strict';
import {mkdir, readFile} from 'node:fs/promises';

const base = process.env.MOTION_URL || 'http://127.0.0.1:8873';
const browser = await chromium.launch({
  executablePath: process.env.CHROME_PATH || 'C:/Program Files/Google/Chrome/Application/chrome.exe', headless: true,
});
const page = await browser.newPage({viewport:{width:1500,height:1100}});
const errors = [], failedAssets = [];
page.on('pageerror', error => errors.push(error.message));
page.on('response', response => { if (response.url().includes('/assets/') && !response.ok()) failedAssets.push(response.url()); });
try {
  await page.goto(base);
  await page.waitForFunction(() => document.querySelector('#control').textContent === '释放控制');
  for (const model of ['TsenseI_v3_2', 'TsenseI_v3_2_withid']) {
    await page.locator('#modelSelect').selectOption(model);
    await page.waitForFunction(model => document.querySelector('#fileNote').textContent.includes(model.endsWith('_withid') ? 'motions_a32_withid_left' : 'motions_a32_left'), model);
    await page.waitForFunction(() => document.querySelector('#modelNote').textContent === '');
    assert.equal(await page.locator('#joints input[type=range]').count(), 20);
    const response = await page.request.get(base + '/api/state', {headers:{'X-Motion-Client':'a32-range-readonly-browser'}});
    assert.equal(response.ok(), true);
    const state = await response.json();
    for (const joint of state.joints) {
      const expected = joint.name.includes('_swing_') ? [0,.7] : /_(PIP|DIP)_/.test(joint.name) ? [.005,1.565] : joint.name.includes('_MCP_') ? [.005,1.3] : null;
      if (expected) assert.deepEqual([joint.min,joint.max], expected);
      if (joint.name.includes('_swing_')) assert.equal(joint.value, 0);
    }
    assert.equal(await page.locator('#a32Motors').isVisible(), true);
    assert.equal(await page.locator('#a30Motors').isVisible(), false);
    assert.equal(await page.locator('#ports').isVisible(), false);
    assert.equal(await page.locator('#a32Connection').isVisible(), true);
    assert.equal(await page.locator('#a32Enable button').isDisabled(), true);
    await page.locator('#a32Transport').selectOption('eth');
    assert.equal(await page.locator('#a32Host').isVisible(), true);
    await page.locator('#a32Transport').selectOption('usb');
    const joint = page.locator('#joints input[type=number]').first();
    await joint.fill('0.1'); await joint.press('Enter');
    await page.locator('#savemotion button').click();
    await page.locator('#dialog-items input[type=text]').fill('A32 浏览器验证');
    await page.getByRole('button', {name:'确认',exact:true}).click();
    await page.waitForFunction(() => document.querySelector('#motionCount').textContent === '1 个动作');
    const downloadPromise = page.waitForEvent('download');
    await page.locator('#export').click();
    const download = await downloadPromise;
    assert(download.suggestedFilename().includes(model));
    const payload = JSON.parse(await readFile(await download.path(), 'utf8'));
    assert.equal(payload._model_id, model);
    assert.equal(payload._joint_coordinate_version, 'a32-positive-swing-v1');
    assert.equal(payload._swing_axis_alignment, 'id-direction-v1');
    assert.equal(Object.keys(payload.motions['A32 浏览器验证'].pose).length, 20);
    assert.equal(payload.motions['A32 浏览器验证'].pose.left_hand_pinky_DIP_joint, .1);
    await page.locator('#motionstreamplay button').click();
    await page.waitForFunction(() => document.querySelector('#status').textContent.includes('正在播放'));
    await page.waitForFunction(() => document.querySelector('#status').textContent.includes('就绪'), null, {timeout:6000});
    await page.locator('#handSide select').selectOption('1');
    await page.waitForFunction(() => document.querySelector('#fileNote').textContent.includes('_right.json'));
    await page.waitForFunction(() => document.querySelector('#modelNote').textContent === '');
    assert.equal(await page.locator('#connect').isDisabled(), true);
    assert.equal(await page.locator('#motionCount').innerText(), '0 个动作');
    await page.locator('#handSide select').selectOption('0');
    await page.waitForFunction(() => document.querySelector('#fileNote').textContent.includes('_left.json'));
    await page.waitForFunction(() => document.querySelector('#modelNote').textContent === '');
    assert.equal(await page.locator('#motionCount').innerText(), '1 个动作');
  }
  const output = process.env.MOTION_QA_OUTPUT || '.work/a32-browser';
  await mkdir(output, {recursive:true});
  await page.screenshot({path:output + '/motion-play-withid.png',fullPage:true});
  for (const model of ['TsenseI_v3_0', 'TsenseI_v3_0_withid', 'TsenseI_v2']) {
    await page.locator('#modelSelect').selectOption(model);
    await page.waitForFunction(() => !document.querySelector('#ports').hidden);
    await page.waitForFunction(() => document.querySelector('#modelNote').textContent === '');
    assert.equal(await page.locator('#a32Motors').isVisible(), false);
  }
  assert.deepEqual(errors, []);
  assert.deepEqual(failedAssets, []);
  console.log(JSON.stringify({ok:true, models:5, a32_sides:4, save_export_playback:true, errors, failedAssets}));
} finally {
  await browser.close();
}
